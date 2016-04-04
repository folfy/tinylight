/*
 * misc_adc.c
 *
 * Created: 21.04.2014 20:36:34
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "tiny_protocol.h"
#include "mul16x16.h"
#include "set_sled.h"
#include "misc_adc.h"

//////////////////////////////////////////////////////////////////////////
/* ADC */

volatile adc_sample measure={0,0,0,0};
volatile uint_fast16_t u_min_raw=UINT_FAST16_MAX,	i_max_raw=0;

//11bit resolution factor in Q2.14 (13b = Q0.16 -> slw 16 from mul) , required shift = scale (13b) - val_res (11b) = slw 2b (with Mul16x16toH16)
const uint16_t v_scale13b=3.224*(1<<14)+0.5;	// 2047	/ 6.6	V/V		*   1	=  0.3101 U/mV -> 3.224	 mV/U
const uint16_t c_scale13b=2.035*(1<<14)+0.5;	// 2047	* 0.015	V/A		*  16	=  0.4913 U/mA -> 2.035	 mA/U
const uint16_t l_scale13b=2.035*(1<<14)+0.5;	// 2047	* 0.48  mV/Lux	* 1/2	= ~0.5	  U/Lux-> 2.0	Lux/U (high sensor tolerance)	
uint16_t t_scale13b;							// 2047/((2047 * temp_cal)>>14) = ~3.3	  U/Deg-> 0.3	  C/U (use t_scale)

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);

/* Init ADC controller */
void adc_init(void)
{	
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&ADC, &adc_conf);
	adcch_read_configuration(&ADC,ADC_CH0, &adcch_conf);
		
	adc_set_conversion_parameters(&adc_conf,ADC_SIGN_ON,ADC_RES_12, ADC_REF_BANDGAP);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_EVENT_SWEEP, 4, ADC_EVCH);
	adc_set_current_limit(&adc_conf,ADC_CURRENT_LIMIT_HIGH);
	adc_set_gain_impedance_mode(&adc_conf,ADC_GAIN_LOWIMPEDANCE);
	adc_set_clock_rate(&adc_conf,125000);
	adc_enable_internal_input(&adc_conf,ADC_INT_TEMPSENSE|ADC_INT_BANDGAP);
		
	adc_write_configuration(&ADC,&adc_conf);
		
	/* led supply voltage */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN_Volt,ADCCH_NEG_NONE,1);
	adcch_write_configuration(&ADC,ADC_CH0, &adcch_conf);
		
	/* led current */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN_Sense,ADCCH_NEG_PIN_Sense,16);
	adcch_write_configuration(&ADC,ADC_CH1,&adcch_conf);
		
	/* environment brightness */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN_Light,ADCCH_NEG_PAD_GND,1);
	adcch_write_configuration(&ADC,ADC_CH2,&adcch_conf);
		
	/* Internal temp */
	adcch_set_input(&adcch_conf,ADCCH_POS_TEMPSENSE,ADCCH_NEG_NONE,1);
	adcch_enable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADC,ADC_CH3,&adcch_conf);
		
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);
	ADC_EVCH_MUX=EVSYS_CHMUX_PRESCALER_16384_gc;	//ADC trigger:	EVSYS_CH0 -> (F_CPU / 16384: 1.95 kHz)
		
	ioport_set_pin_dir(Sens_Light_en,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(Sens_Light_en,IOPORT_PIN_LEVEL_HIGH);
	
	// invert calibration data for unsigned mode (dV=+190LSB) to signed conversion factor, T[1/10 C]=(val*temp_cal)>>14-2730,
	t_scale13b=(((uint_fast32_t)3580)<<14)/(adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2-95);
	adc_enable(&ADC);
	adc_set_callback(&ADC,ADC_int);
}

void get_max_reset(uint_fast16_t *u_min, uint_fast16_t *i_max)
{
	*u_min=MulU16X16toH16Round(u_min_raw<<2,v_scale13b);
	*i_max=MulU16X16toH16Round(i_max_raw<<2,c_scale13b);
	u_min_raw=UINT_FAST16_MAX;
	i_max_raw=0;
}

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	//increase unsigned resolution from 11 bit to 13 bit (range = (2047 * 16) >> 2 = 8192, see atmel doc8003: adc oversampling theory)
	const uint_fast8_t samples = 16, shift = 2;
	static uint_fast8_t adc_cnt = samples;
	adc_sample sample;
	static adc_sample sum={0,0,0,0};
			
	sample.voltage	= adc_get_unsigned_result(&ADC,ADC_CH0);	
	sample.current	= adc_get_signed_result  (&ADC,ADC_CH1);	
	sample.light	= adc_get_signed_result	 (&ADC,ADC_CH2);	
	sample.temp		= adc_get_unsigned_result(&ADC,ADC_CH3);
	
	if(sample.voltage<u_min_raw)
		u_min_raw	= sample.voltage;						
	if(sample.current>(int_fast16_t) i_max_raw)
		i_max_raw	= sample.current;	
			
	sum.voltage += sample.voltage;
	sum.current += sample.current;
	sum.light	+= sample.light;
	sum.temp	+= sample.temp;


	if(!--adc_cnt)
	{
			
		static adc_sample adc_mean={0,0,0,0};
			
		adc_mean.voltage	+= sum.voltage	>>shift;
		sum.voltage=0;
		adc_mean.current	+= sum.current	>>shift;
		sum.current=0;
		adc_mean.light		+= sum.light	>>shift;
		sum.light=0;
		adc_mean.temp		+= sum.temp		>>shift;
		sum.temp=0;
		adc_cnt=samples;
			
		const uint_fast8_t mean=4, mshift=2;	//13b val -> mean=15b -> srw2 -> 13b
		static uint_fast8_t mean_cnt=mean;
		if(!--mean_cnt)
		{
			measure.voltage		= MulU16X16toH16(adc_mean.voltage>>mshift,	v_scale13b);
			adc_mean.voltage=0;
			if(adc_mean.current>0)
				measure.current = MulU16X16toH16(adc_mean.current>>mshift,	c_scale13b);
			else
				measure.current = 0;
			adc_mean.current=0;
			if(adc_mean.light>0)
				measure.light	= MulU16X16toH16(adc_mean.light>>mshift,	l_scale13b); //TODO: Add light calibration
			else
				measure.light	= 0;
			adc_mean.light=0;
			measure.temp		= MulU16X16toH16(adc_mean.temp>>mshift,		t_scale13b) - 2730;
			adc_mean.temp=0;
			mean_cnt=mean;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/* Sleep */

Bool volatile sleeping = false;

void power_down(void)
{
	sleeping = true;
	adc_disable(&ADC);
	ioport_set_pin_level(Sens_Light_en,IOPORT_PIN_LEVEL_LOW);
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV_SLEEP,CONFIG_SYSCLK_PSBCDIV_SLEEP);
	while((set.mode==MODE_SLEEP)&&!udi_cdc_is_rx_ready())
		sleepmgr_enter_sleep();		
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	ioport_set_pin_level(Sens_Light_en,IOPORT_PIN_LEVEL_HIGH);
	adc_enable(&ADC);
	sleeping = false;
}

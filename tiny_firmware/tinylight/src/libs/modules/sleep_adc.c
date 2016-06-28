/*
 * sleep_adc.c
 *
 * Created: 21.04.2014 20:36:34
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "libs/math/mul16x16.h"
#include "libs/protocol/tiny_protocol.h"
#include "libs/modules/settings_sled.h"
#include "sleep_adc.h"

//////////////////////////////////////////////////////////////////////////
/* ADC */

volatile adc_sample measure={0,0,0,0};
volatile uint_fast16_t u_min_raw=UINT_FAST16_MAX,	i_max_raw=0;

// scaling factor in Q2.14 format for 11bit input (0-2047)
// with 13bit oversampled input (*4), this is equal to Q0.16 -> 16 uppermost bits represent result
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
	ADC_EVCH_MUX=EVSYS_CHMUX_PRESCALER_16384_gc;	//ADC trigger:	F_CPU / 16384: 1.95 kHz (2.93 kHz @ 48MHz)
		
	ioport_set_pin_dir(Sens_Light_en,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(Sens_Light_en,IOPORT_PIN_LEVEL_HIGH);
	
	// invert calibration data for unsigned mode (dV=+190LSB) to signed conversion factor, T[1/10 C]=(val*temp_cal)>>14-2730,
	t_scale13b=(((uint_fast32_t)3580)<<14)/(adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2-95);
	adc_enable(&ADC);
	adc_set_callback(&ADC,ADC_int); // Interrupt priority is LOW by default
}

void get_max_reset(uint_fast16_t *u_min, uint_fast16_t *i_max)
{
	*u_min=MulU16X16toH16Round(u_min_raw<<2, v_scale13b);
	*i_max=MulU16X16toH16Round(i_max_raw<<2, c_scale13b);
	u_min_raw=UINT_FAST16_MAX;
	i_max_raw=0;
}

typedef struct {
	uint_fast32_t voltage;
	int_fast32_t  current;
	int_fast32_t  light;
	uint_fast32_t temp;
} adc_sum_t;

adc_sum_t adc_sum = { 0, 0, 0, 0 };

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	//increase unsigned resolution from 11 bit to 13 bit (range = (2047 * 16) >> 2 = 8192, see atmel doc8003: adc oversampling theory)
	const  uint_fast16_t samples = 64; // 2^6
	const  uint_fast8_t  shift   = 4;
	static uint_fast8_t  adc_cnt = samples;;

    uint_fast16_t voltage = adc_get_unsigned_result(&ADC,ADC_CH0);
    int_fast16_t  current = adc_get_signed_result  (&ADC,ADC_CH1);
	int_fast16_t  light   = adc_get_signed_result  (&ADC,ADC_CH2);
    uint_fast16_t temp    = adc_get_unsigned_result(&ADC,ADC_CH3);
	
	if (voltage < u_min_raw)
		u_min_raw	= voltage;
	if (current > (int_fast16_t)i_max_raw)
		i_max_raw	= current;
			
	adc_sum.voltage += voltage;
	adc_sum.current += current;
	adc_sum.light	+= light;
	adc_sum.temp	+= temp;

	if(--adc_cnt == 0)
	{
		measure.voltage		= MulU16X16toH16(adc_sum.voltage>>shift, v_scale13b);
		if (adc_sum.current > 0)
			measure.current = MulU16X16toH16(adc_sum.current>>shift, c_scale13b);
		else
			measure.current = 0;
		if (adc_sum.light > 0)
			measure.light	= MulU16X16toH16(adc_sum.light>>shift,	 l_scale13b); //TODO: Add light calibration
		else
			measure.light	= 0;
		measure.temp		= MulU16X16toH16(adc_sum.temp>>shift,    t_scale13b) - 2730;

		memset(&adc_sum, 0, sizeof(adc_sum));
		adc_cnt=samples;
	}
}

//////////////////////////////////////////////////////////////////////////
/* Sleep */

Bool volatile sleeping = false;

void power_down(void)
{
	if ( (set.mode==MODE_SLEEP) && !udi_cdc_is_rx_ready() )
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

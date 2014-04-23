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

uint_fast16_t t_scale13b;	// Q2.14 - 11bit: 2047/((2047 * temp_cal)>>14) = ~3.3	  U/Deg-> 0.3	  C/U (use t_scale)
volatile adc_sample measure={0,0,0,0};
volatile uint_fast16_t scp_threshold, uvp_threshold;
volatile uint_fast16_t u_min_raw= UINT_FAST16_MAX,	i_max_raw	= 0,	t_max_raw	= 0;	//TODO:	scp_val & uvp_val & t_val evaluation, reset
volatile uint_fast16_t u_min=	  UINT_FAST16_MAX,	i_max		= 0,	t_max		= 0;

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);

void adc_init(void)
{	// invert calibration data for unsigned mode (dV=+190LSB) to signed conversion factor, T[1/10 C]=(val*temp_cal)>>14-2730,
	t_scale13b=(((uint_fast32_t)3580)<<14)/(adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2-95);
	uvp_scp_update();
	protection_reset();
	adc_enable(&ADC);
	adc_set_callback(&ADC,ADC_int);
}

//TODO: temp_cal25b=(((uint_fast32_t)temp_mess+2730)<<14)/temp_raw), t_scale-> volatile

void protection_reset(void)
{
	u_min_raw=UINT_FAST16_MAX;
	i_max_raw=0;
	t_max_raw=0;
	u_min=	  UINT_FAST16_MAX;
	i_max=	  0;
	t_max=	  0;
}

void uvp_scp_update(void)
{
	if(set.uvp)
		uvp_threshold=set.uvp*10+4000;
	else
		uvp_threshold=0;
	if(set.scp)
		scp_threshold=set.scp*50;
	else
		scp_threshold=UINT_FAST16_MAX;
}

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	//11bit value factor in Q2.14 (13b = Q0.16 -> slw 16 from mul) , required shift = scale (13b) - val_res (11b) = slw 2b (with Mul16x16toH16)
	const uint16_t v_scale13b=3.224*(1<<14)+0.5;	// 2047	/ 6.6	V/V		*   1	=  0.3101 U/mV -> 3.224	 mV/U
	const uint16_t c_scale13b=2.035*(1<<14)+0.5;	// 2047	* 0.015	V/A		*  16	=  0.4913 U/mA -> 2.035	 mA/U
	const uint16_t l_scale13b=2.035*(1<<14)+0.5;	// 2047	* 0.48  mV/Lux	* 1/2	= ~0.5	  U/Lux-> 2.0	Lux/U (high sensor tolerance)
	if(ch_mask==ADC_CH0)
		mode_update(MODE_ERROR_BOP);
	else
	{
		//increase unsigned resolution from 11 bit to 12 bit (range = (2047 * 16) >> 3 = 4094, see atmel doc8003: adc oversampling theory)
		const uint_fast8_t samples = 16, shift = 3;
		static uint_fast8_t adc_cnt = samples;
		adc_sample sample;
		static adc_sample sum={0,0,0,0};
			
		sample.voltage	= adc_get_unsigned_result(&ADC,ADC_CH0);	
		sample.current	= adc_get_signed_result  (&ADC,ADC_CH1);	
		sample.light	= adc_get_unsigned_result(&ADC,ADC_CH2);	
		sample.temp		= adc_get_unsigned_result(&ADC,ADC_CH3);
	
		//TODO: Test UVP, SCP, BOP
		if(sample.voltage<u_min_raw)
		{
			u_min_raw	= sample.voltage;
			u_min		= MulU16X16toH16Round(u_min_raw<<2,v_scale13b);
			if((u_min<=uvp_threshold)&&(set.mode&STATE_ON))
				mode_update(MODE_ERROR_UVP);									
		}
		if(sample.current>(int_fast16_t) i_max_raw)
		{
			i_max_raw	= sample.current;
			i_max		= MulU16X16toH16Round(i_max_raw<<2,c_scale13b);
			if((i_max>=scp_threshold)&&(set.mode&STATE_ON))
				mode_update(MODE_ERROR_SCP);				
		}
		if(sample.temp>t_max_raw)
		{
			t_max_raw	= sample.temp;
			t_max		= MulU16X16toH16Round(i_max_raw<<2,t_scale13b);
		}
			
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
			
			const uint_fast8_t mean=8, mshift=2;	//12b val -> mean=15b -> srw2 -> 13b
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
				measure.light		= MulU16X16toH16(adc_mean.light>>mshift,	l_scale13b);
				adc_mean.light=0;
				measure.temp		= MulU16X16toH16(adc_mean.temp>>mshift,		t_scale13b) - 2730;
				adc_mean.temp=0;	//TODO: make t_offset variable
				mean_cnt=mean;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/* Sleep */

void power_down(void)
{
	adc_disable(&ADC);
	sysclk_set_prescalers(SYSCLK_PSADIV_2,SYSCLK_PSBCDIV_1_1);
	while((set.mode==MODE_OFF)&&!ioport_get_pin_level(USB_VBUS))
		sleepmgr_enter_sleep();
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADC);
}

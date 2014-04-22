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

uint_fast16_t temp_cal;
int_fast16_t scp_threshold=INT_FAST16_MAX, uvp_threshold=0;	//UNDONE:	threshold calc

volatile adc_sample measure={0,0,0,0};
volatile uint_fast16_t scp_val, uvp_val;	//TODO:		scp_val & uvap_val evaluation

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);

void adc_init(void)
{
	temp_cal=adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2-100; // calibration data is for unsigned mode, which has a positive offset of about 200
	adc_enable(&ADC);
	adc_set_callback(&ADC,ADC_int);
}

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	const uint16_t vscale_11b=3.224*16384+0.5;
	const uint16_t cscale_11b=2.035*256+0.5;
	if(ch_mask==ADC_CH0)
	{
		mode_update(mode_error_BOP);
		uvp_val = MulU16X16toH16Round(adc_get_unsigned_result(&ADC,ADC_CH0)<<2,vscale_11b);
	}
	else
	{
		//increase unsigned resolution from 11 bit to 12 bit (max value = 2047 * 16 / 4 = 4094
		const uint_fast8_t samples = 16, shift = 3, average = 8;
		static uint_fast8_t adc_cnt = samples, mean_cnt = average;
		static int_fast16_t  voltage_sum = 0, current_sum = 0, light_sum = 0, temp_sum = 0;
		static adc_sample adc_mean={0,0,0,0};
		int_fast16_t voltage, current, light, temp;
			
		voltage	= adc_get_signed_result(&ADC,ADC_CH0);	// 2047	/ 6.6	V/V   *   1	= 0.3101 U/mV -> 3.224	mV/U
		current	= adc_get_signed_result(&ADC,ADC_CH1);	// 2047	* 0.015	V/A	  *  16	= 0.4913 U/mA -> 2.035	mA/U
		light	= adc_get_signed_result(&ADC,ADC_CH2);	// 2047	* 0.48  mV/Lux* 1/2 = 0.4913 U/Lux-> 2.035 Lux/U
		temp	= adc_get_signed_result(&ADC,ADC_CH3);	// * 3580 / temp_cal - 2730 = T;
	
		//TODO: Test UVP, SCP, BOP
		if(((current>=scp_threshold)||(voltage<=uvp_threshold))&&(set.mode&state_on))
		{
			if(voltage<=uvp_threshold)
				mode_update(mode_error_UVP);
			else
				mode_update(mode_error_SCP);
			uvp_val = MulU16X16toH16Round(voltage<<2,vscale_11b);
			scp_val = MulU16X16toH16Round(current<<2,cscale_11b);
		}
			
		voltage_sum += voltage;
		current_sum += current;
		light_sum	+= light;
		temp_sum	+= temp;

			
		if(!--adc_cnt)
		{
			if(voltage_sum>0)
				adc_mean.voltage	+= voltage_sum	>>shift;	// 4094 / 6.6V/V		* 1		= 1.2406 U/mV  -> 0.806 mV/U
			voltage_sum=0;
			if(current_sum>0)
			adc_mean.current	+= current_sum	>>shift;	// 8188 * 0.015V/A		* 16	= 1.9651 U/mA  -> 0.509 mA/U
			current_sum=0;
			if(light_sum>0)
			adc_mean.light		+= light_sum	>>shift;	// 8188 * 0.48mV/Lux	* 1/2	= 1.9651 U/Lux -> 0.509 Lux/U (high sensor tolerance - rough value)
			light_sum=0;
			if(temp_sum>0)
			adc_mean.temp		+= temp_sum		>>shift;	// ((uint32_t)temp_sum * 3580 / (temp_cal*shift)) - 2730;	cal=85C, 0V=0K, temperature = 1/10C
			temp_sum=0;
			adc_cnt=samples;
			//TODO: check adc oversampling, implement moving average
			if(mean_cnt)
			{
				measure.voltage = MulU16X16toH16(adc_mean.voltage,0.806*65536);
				adc_mean.voltage=0;
				measure.current = MulU16X16toH16(adc_mean.current,0.509*65536);
				adc_mean.current=0;
				measure.light = MulU16X16toH16(adc_mean.light,0.509*65536);
				adc_mean.light=0;
				measure.temp = (adc_mean.temp*3580 / (temp_cal*8)) - 2730;	// TODO: temp_cal - test (delta V)
				adc_mean.temp=0;
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
	while((set.mode==mode_off)&&!ioport_get_pin_level(USB_VBUS))
		sleepmgr_enter_sleep();
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADC);
}

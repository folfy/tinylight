/*
 * misc_adc.h
 *
 * Created: 21.04.2014 20:36:46
 *  Author: Folfy
 */ 


#ifndef MISC_ADC_H_
#define MISC_ADC_H_

/* ADC */

extern volatile adc_sample measure;

void adc_init(void);

/* Sleep */

void power_down(void);




#endif /* MISC_ADC_H_ */
/**
 * \file
 *
 * IO:
 *	PA1:	A_IN:		Light Sensor
 *	PA5:	D_OUT:		Power Mosfet
 *	PA6:	A_IN:		Current-Sensor +
 *	PA7:	A_IN:		Current-Sensor -
 *	PB3:	A_IN:		+5V-Sensor
 *	PC1:	D_OUT:		XCK-Led Stripe
 *	PC3:	D_OUT:		TX-Led Stripe
 *	PD0:	D_OUT:		Led blue
 *	PD1:	D_OUT:		Led green
 *	PD2:	D_OUT:		Led red
 *	PD3:	D_IN:		USB VBus Detection
 *	PD6:	D_INOUT:	USB Data -
 *	PD7:	D_INOUT:	USB Data +
 *	PE3:	D_IN:		Pushbutton
 *
 *	TCD0:	PWM Status Led
 *	RTC:	Blink Status Led
 *	TCD1:	DMA-Wait
 *	TCE0:	Taster
 */

#include "main.h"
#include "tiny_protocol.h"
#include <stdio.h>
#include <avr/eeprom.h>
#include <asf.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "data.h"
#include "linsin.h"

// framerate statistics
uint_fast16_t volatile FPS = 0;
uint_fast16_t volatile count = 0;

uint_fast8_t back_buffer[480];
uint_fast8_t front_buffer[480];

volatile Bool timeout_flag = false;

volatile uint_fast8_t button_cnt=0;
volatile uint_fast8_t prev_mode=mode_off;

uint_fast16_t temp_cal = 0;

volatile Bool update_frame = false;
volatile Bool usb_data_pending = false;
volatile Bool mode_select = false;
volatile Bool blink_en=false;
volatile Bool blink_state;
volatile Bool sleeping=false;

settings set;
volatile adc_sample measure={0,0,0,0};

void RTC_Alarm(uint32_t time);
void TCD1_OVF_int(void);
void ADCA_CH3_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);

void DMA_Led_int(dma_callback_t state);

//Status_LED, FPS count
void RTC_Alarm(uint32_t time)
{
	const uint_fast16_t alarm = 0.0125*RTC_cycle;
	static uint_fast32_t alarm_prev = 0;
	
	const uint_fast16_t button_long = 0.25*RTC_cycle;
	const uint_fast16_t button_reset = 7*RTC_cycle;
	static uint_fast32_t button_time = 0;
	static Bool button_mem = true;						//prevent writing button_time if button pressed on boot
	Bool button_state = ioport_get_pin_level(BUTTON);
	if(button_state != button_mem)
	{
		if(button_state)
			button_time=rtc_get_time();
		else
			if(button_time)
				button_update(rtc_get_time() >= button_time + button_long);
		button_mem=button_state;
	}
	if(button_state & ( rtc_get_time() > (button_time+button_reset) ))
		reset_do_soft_reset();
	
	const uint_fast16_t led_alarm = 0.5*RTC_cycle;
	static uint_fast16_t led_time = 0;
	led_time+=alarm;
	if((led_time >= led_alarm) && blink_en)
	{
		status_led_blink();
		led_time=0;
	}
	
	static uint_fast16_t fps_time = 0;
	fps_time+=alarm;
	if(fps_time >= 1*RTC_cycle)
	{
		FPS = count;
		fps_time = 0;
	}
	
	if(time > (UINT32_MAX-RTC_cycle*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&state_on) || (time >= (UINT32_MAX-alarm)) )	// delay as long as required or possible
			reset_do_soft_reset();
	}
	rtc_set_alarm(alarm_prev+=alarm);
}

void ADCA_CH3_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	const uint_fast8_t mean = 128, mshift = 7;	// 128 =  2^7
	static uint_fast8_t adc_cnt = mean;
	static int_fast32_t  led_voltage_sum = 0, led_current_sum = 0, light_sum = 0, temp_sum = 0;

	led_voltage_sum += adc_get_unsigned_result(&ADCA,ADC_CH0);
	led_current_sum += adc_get_unsigned_result(&ADCA,ADC_CH1);
	light_sum		+= adc_get_unsigned_result(&ADCA,ADC_CH2);
	temp_sum		+= adc_get_unsigned_result(&ADCA,ADC_CH3);
	
	adc_cnt--;
	if (!adc_cnt)
	{
		if(led_voltage_sum>0)
			measure.voltage = (led_voltage_sum * 3301)		>> (mshift+10);		//2047 / 6.6V/V		* 1		= 0.3102 U/mV  -> 3.224 mV/U * 1024	= 3,301
		else
			measure.voltage = 0;
		led_voltage_sum = 0;
		
		if((led_current_sum>0) && set.mode)
			measure.current = (led_current_sum * 2084)		>> (mshift+10);		//2047 * 0.015V/A	* 16	= 0.4913 U/mA  -> 2.036 mA/U * 1024 = 2,084
		else
			measure.current = 0;
		led_current_sum = 0;			
		
		if(light_sum>0)
			measure.light	= light_sum						>> (mshift-1);		//2047 * 0.48mV/Lux * 1/2	= 0.4913 U/Lux -> 2.035 Lux/U (high sensor tolerance - rough value)
		else
			measure.light	= 0;
		light_sum = 0;
										
		if(temp_sum>0)
			measure.temp	= ((temp_sum * 3580 / temp_cal) >> mshift) - 2730;	//cal=85C, 0V=0K, temperature = 1/10C
		else
			measure.temp	= 0;
		temp_sum = 0;
		
		adc_cnt = mean;
	}
}


int main (void)
{
	board_init();
	read_settings();
	dma_init();
	callback_init();
	temp_cal=adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2; // calibration data is for unsigned mode, which has a positive offset of about 200
	adc_enable(&ADCA);
	gamma_calc();
	udc_start();
	if (!udc_include_vbus_monitoring())
	{
		if(ioport_get_pin_level(USB_VBUS))
			udc_attach();
		PORTD_INTCTRL = 1;
	}
	if (set.default_mode >= mode_prev_offset)
		mode_update(set.default_mode - mode_prev_offset);
	else
		mode_update(set.default_mode);	
		
	while(1)
	{
		if (usb_data_pending) 
		{
			if (read_USB() & !timeout_flag)		
				udi_cdc_putc(ack);
			else 
			{ 
				timeout_flag = false;
				udi_cdc_putc(nack);
			}
			usb_data_pending = false;
			while(udi_cdc_is_rx_ready()) udi_cdc_getc();
		}
		switch (set.mode)
		{
			//case mode_light_bar:	status_bar(measure.light, 4000, set.count); SPI_start(); break;
			case mode_mood_lamp:	Mood_Lamp(set.count); gamma_map(); SPI_start(); break;
			case mode_rainbow:		Rainbow(set.count); gamma_map(); SPI_start(); break;
			case mode_colorswirl:	Colorswirl(set.count); gamma_map(); SPI_start(); break;
			case mode_off:			power_down(); break;	
		}
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~ 	MISC		~~~~~~~~~~~~~~~~~~~~~~~ */

void callback_init(void)
{
	dma_set_callback(DMA_CHANNEL_LED, DMA_Led_int);
	tc_set_overflow_interrupt_callback(&TCD1,TCD1_OVF_int);
	adc_set_callback(&ADCA,ADCA_CH3_int);
	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	cpu_irq_enable();
}

void power_down(void)
{
	sleeping=true;
	adc_disable(&ADCA); 
	sysclk_set_prescalers(SYSCLK_PSADIV_2,SYSCLK_PSBCDIV_1_1);
	while(set.mode==mode_off)
		sleepmgr_enter_sleep(); 
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADCA);
	sleeping=false;
}

void button_update(Bool key_state)
{
	if(set.mode==mode_off)
	{
		if (set.default_mode >= mode_prev_offset)
		{
			if(prev_mode)
				mode_update(prev_mode);
			else
				mode_update(set.default_mode-mode_prev_offset);	
		}
		else
			mode_update(set.default_mode);
	}
	else
	{
		if(key_state)
		{
			//long
			if (set.mode != mode_off)
			{
				mode_select ^= true;
			}
		}
		else
		{
			//short
			if(mode_select)
			{
				//Change Mode
				switch(set.mode)
				{
					case mode_mood_lamp:	mode_update(mode_rainbow); break;
					case mode_rainbow:		mode_update(mode_colorswirl); break;
					//case mode_colorswirl:	mode_update(mode_mood_lamp); break;
					default:				mode_update(mode_mood_lamp); break;
				}
			}
			else
				mode_update(mode_off);
		}
	}
}

void mode_update(uint_fast8_t mode)
{
	if(set.mode != mode)
	{
		prev_mode=set.mode;
		set.mode = mode;
		if (set.mode&state_on)	
			PORTA_OUTCLR = PIN5_bm;
		else
			PORTA_OUTSET = PIN5_bm;
			SetupDMA(set.mode&state_multi);
		status_led_update();
	}
}

void status_led_update(void)
{

	if (mode_select)
	{
		tc_write_cc(&TCD0,TC_CCA,set.stat_LED/3);
		tc_write_cc(&TCD0,TC_CCB,set.stat_LED/2);
		tc_write_cc(&TCD0,TC_CCC,set.stat_LED/2);
		tc_enable_cc_channels(&TCD0,TC_CCAEN);
		tc_enable_cc_channels(&TCD0,TC_CCBEN);
		tc_enable_cc_channels(&TCD0,TC_CCCEN);
	}
	else
	{
		if(set.mode&state_on)
		{
			blink_en=false;
			#warning "undefined state: set.stat_LED=0!"
			if(set.stat_LED)
			{
				if(set.mode&state_usb)
				{
					tc_enable_cc_channels(&TCD0,TC_CCAEN);
					tc_disable_cc_channels(&TCD0,TC_CCBEN);
					tc_disable_cc_channels(&TCD0,TC_CCCEN);
					tc_write_cc(&TCD0, TC_CCA, set.stat_LED);
				}
				else
				{
					tc_disable_cc_channels(&TCD0,TC_CCAEN);
					tc_enable_cc_channels(&TCD0,TC_CCBEN);
					tc_disable_cc_channels(&TCD0,TC_CCCEN);
					tc_write_cc(&TCD0, TC_CCB, set.stat_LED);
				}
			}
		}
		else
		{
			if(set.mode&state_error)
			{
				blink_en=true;
				blink_state=false;
				tc_write_cc(&TCD0, TC_CCC, 0xFF);
			}
			else
			{
				#warning "undefined state: set.stb_LED=0!"
				if(set.stb_LED)
				{
					blink_en=false;
					tc_disable_cc_channels(&TCD0,TC_CCAEN);
					tc_enable_cc_channels(&TCD0,TC_CCBEN);
					tc_enable_cc_channels(&TCD0,TC_CCCEN);
					tc_write_cc(&TCD0, TC_CCB, set.stb_LED/4);
					tc_write_cc(&TCD0, TC_CCC, set.stb_LED);
				}
			}
		}
	}
}

void status_led_blink(void) //Error blinking
{
	if(blink_state)
	{
		tc_disable_cc_channels(&TCD0,TC_CCCEN);
		blink_state=true;
	}
	else
	{
		tc_enable_cc_channels(&TCD0,TC_CCCEN);
		blink_state=false;
	}	
}

/* ~~~~~~~~~~~~~~~~~~~~~~~ 	EEPROM		~~~~~~~~~~~~~~~~~~~~~~~ */

void read_settings(void)
{
	nvm_eeprom_read_buffer(set_EE_offset, &set, sizeof(set));
};

void save_settings(void)
{
	nvm_eeprom_flush_buffer();
	for (uint8_t i = 0; i < sizeof(set); i++) 
	{
		nvm_eeprom_load_byte_to_buffer(i, &set);
	}
	nvm_eeprom_atomic_write_page(set_EE_offset);
}

/*
void save_settings(void)
{
	nvm_eeprom_flush_buffer();
	const uint8_t *values = (const uint8_t *) &set;
	for (uint8_t i = 0; i < sizeof(set); ++i) {
		nvm_eeprom_load_byte_to_buffer(i, *values);
		++values;
	}
	nvm_eeprom_atomic_write_page(set_EE_offset);
}*/

/* ~~~~~~~~~~~~~~~~~~~~~~~	LED-MODE	~~~~~~~~~~~~~~~~~~~~~~~ */
uint_fast16_t gamma_lut[256];

void gamma_map(void)
{
	static uint_fast8_t run=0;
	for(uint_fast16_t n=0;n<set.count*3;n++)
	{
		uint_fast8_t val=back_buffer[n];
		uint_fast16_t val_corr=gamma_lut[val];
 		front_buffer[n]=val_corr>>2;
   		if(val_corr%4>run)				//oversample pwm for increased resolution
  			front_buffer[n]++;
	}
	run++;
	if(run==4)
		run=0;
}

void gamma_calc(void)
{
	//split gamma in full exponentiations and 10th roots
	uint_fast8_t gamma_dec=set.gamma;
	uint_fast8_t gamma_pot=0;
	while(gamma_dec>=10)
	{
		gamma_pot++;
		gamma_dec-=10;
	}
	
	uint_fast8_t x=0;
	for(;;)
	{
		uint_fast16_t x2=x*257;	//a marginal error by shifting (equals div 256) instead of dividing by 255 is left
		uint_fast16_t y=65535;
		uint_fast16_t root=(nvm_flash_read_byte(&root_10[x]+1)<<8)+nvm_flash_read_byte(&root_10[x]);
		
		for(uint_fast8_t cnt=gamma_pot;cnt;cnt--)
			y=(y*x2)>>16;
		for(uint_fast8_t cnt=gamma_dec;cnt;cnt--)
			y=(y*root)>>16;
		
		gamma_lut[x]=((y+32)*1020)>>16;	//improve rounding error by adding 32
		
		if(x==255)
			break;
		x++;
	}
}

uint_fast16_t hue1 = 0;

void Mood_Lamp(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		hsv_to_rgb(hue1,&back_buffer[0]);
		hue1 = (hue1 + 2) % 1536;
	}
}

void Rainbow(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		uint_fast16_t hue2 = hue1;
		for (uint_fast8_t k=0; k<anzahl_Leds; k++)
		{
			hsv_to_rgb(hue2, &back_buffer[k*3]);
			hue2  += 40;
		}
		hue1 = (hue1 + 4) % 1536;
	}
}

void Colorswirl(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		/* x=linsin(alpha); x e ]-1...1]= -32767...32768; alpha e [0...259.99]=65535 (1deg=256) */
		/* sine1, sine2 - 1deg=128 - prevent overflow at 260deg */
		const uint_fast16_t deg360 = 46080, rad_3 = 17.2*128, rad_03 = 1.72*128;
		static uint_fast16_t sine1 = 0;
	
		uint_fast16_t sine2 = sine1;
		uint_fast16_t hue2 = hue1;

		for (uint_fast8_t k=0; k<anzahl_Leds; k++)
		{
			uint_fast8_t rgb_buffer[3];
			uint_fast8_t bright;
			hsv_to_rgb(hue2, rgb_buffer);
			if(sine2 >= deg360)
				sine2-= deg360;
			bright = linsin_360(sine2)+127;
		
			back_buffer[k*3]	=	bright * rgb_buffer[0] >> 8;		//R
			back_buffer[k*3+1]	=	bright * rgb_buffer[1] >> 8;		//G
			back_buffer[k*3+2]	=	bright * rgb_buffer[2] >> 8;		//B
		
			hue2  += 40;
			sine2 += rad_3;	// 0.3 rad
		}
		hue1   = (hue1 + 4) % 1536;
		sine1 += rad_03;		// 0.03 rad
		if(sine1>=deg360)
			sine1-=deg360;
	}
};

void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t rgb_buffer[])
{
	uint_fast8_t lo = hue & 255;
	switch((hue >> 8) % 6) {
		case 0:
		rgb_buffer[0] = 255;
		rgb_buffer[1] = lo;
		rgb_buffer[2] = 0;
		return;
		case 1:
		rgb_buffer[0] = 255 - lo;
		rgb_buffer[1] = 255;
		rgb_buffer[2] = 0;
		return;
		case 2:
		rgb_buffer[0] = 0;
		rgb_buffer[1] = 255;
		rgb_buffer[2] = lo;
		return;
		case 3:
		rgb_buffer[0] = 0;
		rgb_buffer[1] = 255 - lo;
		rgb_buffer[2] = 255;
		return;
		case 4:
		rgb_buffer[0] = lo;
		rgb_buffer[1] = 0;
		rgb_buffer[2] = 255;
		return;
		default:
		rgb_buffer[0] = 255;
		rgb_buffer[1] = 0;
		rgb_buffer[2] = 255 - lo;
		return;
	}
}

void status_bar(uint_fast16_t val, uint_fast16_t range, uint_fast8_t anzahl_Leds)
{
	static uint8_t R=255, G=255, B=255;
	range = range / anzahl_Leds;
	for (uint_fast8_t k=0;k<=(anzahl_Leds*3 - 3);k+=3)
	{
		uint_fast16_t temp = ((k/3)+1) * range;
		if ( val / temp > 0)
		{
			back_buffer[k]=R;		//R
			back_buffer[k+1]=G;		//G
			back_buffer[k+2]=B;		//B
		}
		else
		{
			back_buffer[k]=0;
			back_buffer[k+1]=0;
			back_buffer[k+2]=0;
		}
	}
};

/*	~~~~~~~~~~~~~~~~~~~~~~~	 DMA/LED	~~~~~~~~~~~~~~~~~~~~~~~ */

struct dma_channel_config dmach_conf_single;
struct dma_channel_config dmach_conf_multi;

/* Init DMA setup struct */
void dma_init(void)
{
	//single
	memset(&dmach_conf_single, 0, sizeof(dmach_conf_single));

	dma_channel_set_burst_length(&dmach_conf_single, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count(&dmach_conf_single, 3);
	dma_channel_set_repeats(&dmach_conf_single, set.count);

	dma_channel_set_src_reload_mode(&dmach_conf_single, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode(&dmach_conf_single, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address(&dmach_conf_single, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode(&dmach_conf_single, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode(&dmach_conf_single, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address(&dmach_conf_single, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source(&dmach_conf_single, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot(&dmach_conf_single);
	
	dma_channel_set_interrupt_level(&dmach_conf_single, DMA_INT_LVL_MED);
	
	//multi
	memset(&dmach_conf_multi, 0, sizeof(dmach_conf_multi));
	
	dma_channel_set_burst_length(&dmach_conf_multi, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count(&dmach_conf_multi, set.count*3);

	dma_channel_set_src_reload_mode(&dmach_conf_multi, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode(&dmach_conf_multi, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address(&dmach_conf_multi, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode(&dmach_conf_multi, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode(&dmach_conf_multi, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address(&dmach_conf_multi, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source(&dmach_conf_multi, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot(&dmach_conf_multi);
	
	dma_channel_set_interrupt_level(&dmach_conf_multi, DMA_INT_LVL_MED);
};

/* Config DMA in single / multi LED mode */
void SetupDMA(Bool multi)
{
	while (dma_channel_is_busy(DMA_CHANNEL_LED));
	dma_enable();
	if (multi)	dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_multi);
	else		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
	dma_channel_enable(DMA_CHANNEL_LED);
};

/* Start DMA Transfer */
void SPI_start(void)
{
	if (dma_channel_is_busy(DMA_CHANNEL_LED) | TCD1.CTRLA)
		update_frame = true;
	else
		dma_channel_enable(DMA_CHANNEL_LED);
}

//Latch delay DMA (LED)
void TCD1_OVF_int(void)
{
	TCD1.CTRLA = 0; //Stop Timer
	if (update_frame)
	{
		dma_channel_enable(DMA_CHANNEL_LED);
		update_frame = false;
	}
	count +=1;
}

void DMA_Led_int(dma_callback_t state)
{
	DMA.INTFLAGS = 0xff;
	if((set.mode & state_on) && !(set.mode & state_multi))	
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
	tc_restart(&TCD1);
	tc_set_resolution(&TCD1,500000);
}

/*	~~~~~~~~~~~~~~~~~~~~~~~	 USB		~~~~~~~~~~~~~~~~~~~~~~~ */

//VBus detection
ISR (PORTD_INT0_vect)
{
	if(ioport_get_pin_level(USB_VBUS))
		udc_attach();
	else
		udc_detach();
}

void main_cdc_rx_notify(uint8_t port)
{
	usb_data_pending = true;
}

Bool read_USB(void)
{
	uint_fast8_t set_addr;
	uint_fast8_t set_value;
	char usb_rx=get_USB_char();
	if(usb_rx==preamble[0])
	for(uint_fast8_t i=1;i<sizeof(preamble);i++)
	{
		if (get_USB_char() != preamble[i])
		return false;
	}
	else if(usb_rx==pre_ada[0])
	{
		for(uint_fast8_t i=1;i<sizeof(pre_ada);i++)
		{
			if (get_USB_char() != pre_ada[i])
			return false;
		}
		udi_cdc_getc();
		char temp=udi_cdc_getc();
		if(temp!=set.count)
		{
			set.count=udi_cdc_getc();
			SetupDMA(true);
		}
		set.mode=mode_usb_ada;
		return true;
	}
	
	switch (get_USB_char())
	{
		case cmd_test:		udi_cdc_write_buf(&response,sizeof(response));
							udi_cdc_putc(set.mode);
							udi_cdc_putc(Version);
							break;
		case cmd_raw_data:	//if(set.mode == mode_single_led)
		switch(set.mode)
		{	case mode_usb_single:	udi_cdc_read_buf(&back_buffer,3);
			SPI_start();
			break;
			case mode_usb_multi:	return false;
			break;
			default: return false;

		}
		//if (color_buffer[0] == 'A' & color_buffer[1] == 'd' & color_buffer[2] == 'a')
		//{
		//uint_fast8_t temp;
		//temp = udi_cdc_getc();
		//temp = udi_cdc_getc() + 1;
		//if (Led_anzahl != temp)
		//{
		//Led_anzahl = temp;
		//SetupDMA_CH1();
		//}
		//temp = udi_cdc_getc();
		//}
		//else i=3;
		//for(;i<Led_anzahl*3;i++)
		//{
		//color_buffer[i] = udi_cdc_getc() / brightness;
		//}
		break;
		case cmd_measure:	udi_cdc_putc(measure.voltage >> 8);
		udi_cdc_putc(measure.voltage);
		udi_cdc_putc(measure.current >> 8);
		udi_cdc_putc(measure.current);
		udi_cdc_putc(measure.light >> 8);
		udi_cdc_putc(measure.light);
		udi_cdc_putc(measure.temp);
		break;
		case cmd_set_read:	switch(get_USB_char())
							{
								case set_mode:				udi_cdc_putc(set.mode);			break;
								case set_default_mode:		udi_cdc_putc(set.default_mode);	break;
								case set_timeout_mode:		udi_cdc_putc(set.timeout_mode);	break;
								case set_timeout_time:		udi_cdc_putc(set.timeout_time);	break;
								case set_alpha:				udi_cdc_putc(set.alpha);		break;
								case set_default_alpha:		udi_cdc_putc(set.default_alpha);break;
								case set_gamma:				udi_cdc_putc(set.gamma);		break;
								case set_smooth_time:		udi_cdc_putc(set.smooth_time);	break;
								case set_alpha_min:			udi_cdc_putc(set.alpha_min);	break;
								case set_lux_max:			udi_cdc_putc(set.lux_max);		break;
								case set_stat_Led:			udi_cdc_putc(set.stat_LED);		break;
								case set_stb_Led:			udi_cdc_putc(set.stb_LED);		break;
								case set_count:				udi_cdc_putc(set.count);		break;
								case set_OCP:				udi_cdc_putc(set.OCP);			break;
								case set_OCP_time:			udi_cdc_putc(set.OCP_time);		break;
								case set_SCP:				udi_cdc_putc(set.SCP);			break;
								case set_UVP:				udi_cdc_putc(set.UVP);			break;
							}
							break;
		case cmd_set_write:	set_addr  = get_USB_char(); if (set_addr  != get_USB_char()) return false;
							set_value = get_USB_char(); if (set_value != get_USB_char()) return false;
							switch(set_addr)
							{
								case set_mode:				mode_update(set_value);			break;
								case set_default_mode:		set.default_mode = set_value;	break;
								case set_timeout_mode:		set.timeout_mode = set_value;	break;
								case set_timeout_time:		set.timeout_time = set_value;	break;
								case set_alpha:				set.alpha = set_value;			break;
								case set_default_alpha:		set.default_alpha = set_value;	break;
								case set_gamma:				set.gamma = set_value;			break;
								case set_smooth_time:		set.smooth_time = set_value;	break;
								case set_alpha_min:			set.alpha_min = set_value;		break;
								case set_lux_max:			set.lux_max = set_value;		break;
								case set_stat_Led:			set.stat_LED = set_value;		break;
								case set_stb_Led:			set.stb_LED = set_value;		break;
								case set_count:				set.count = set_value;			break;
								case set_OCP:				set.OCP = set_value;			break;
								case set_OCP_time:			set.OCP_time = set_value;		break;
								case set_SCP:				set.SCP = set_value;			break;
								case set_UVP:				set.UVP = set_value;			break;
								default: return false;
							}
							break;
		case cmd_set_save:	eeprom_write_block(&set, (uint8_t *) set_EE_offset, sizeof(set)); break;
		default:			return false;
	}
	return true;
};

#define CMD_timeout 0xfffe

uint_fast8_t get_USB_char(void)
{
	if (!timeout_flag)
	{
		for (uint_fast16_t i = 0; i < CMD_timeout; i++)
		{
			if(udi_cdc_is_rx_ready()) {timeout_flag = false; return udi_cdc_getc();}
		}
	}
	timeout_flag = true;
	return 0;
};
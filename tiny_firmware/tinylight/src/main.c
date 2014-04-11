/**
 * \file
 *
 * IO:
 *	PA1:	A_IN:		Light Sensor
 *	PA5:	D_OUT:		Power Mosfet
 *	PA6:	A_IN:		Current-Sensor +
 *	PA7:	A_IN:		Current-Sensor -
  *	PB1:	D_OUT:		IR_enable
 *	PB2:	D_IN:		IR-In
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
 */

#define IR_avail

#include "main.h"
#include "tiny_protocol.h"
#include <stdio.h>
#include <asf.h>
#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
#include "mul16x16.h"
#include "data.h"
#include "linsin.h"

#ifdef IR_avail
uint_fast8_t get_ir_bit(uint_fast16_t time);
static void IR_init(void);
void TCC0_OVF_int(void);
void remote_key(uint8_t cmd);

#define TCC0_cycle 32000000/64
#define nec_start_max 0.015*TCC0_cycle
#define nec_start_min 0.012*TCC0_cycle
#define nec_repeat_max 0.012*TCC0_cycle
#define nec_one_max 0.003*TCC0_cycle
#define nec_zero_max 0.002*TCC0_cycle
#endif


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
volatile Bool gamma_update = false;
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
		count=0;
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
	const uint_fast8_t mean = 128, mshift = 7;	// 128 =  2^7 (+ 2^9 = 2^16 - avoid shifting)
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
			measure.voltage = (led_voltage_sum * 1651)	>> (mshift+9);	//2047 / 6.6V/V		* 1		= 0.3102 U/mV  -> 3.224 mV/U * 512 (2^9)	= 1,651
		else
			measure.voltage = 0;
		led_voltage_sum = 0;
		
		if((led_current_sum>0) && set.mode)
			measure.current = (led_current_sum * 1042)	>> (mshift+9);	//2047 * 0.015V/A	* 16	= 0.4913 U/mA  -> 2.036 mA/U * 512 (2^9)	= 1,042
		else
			measure.current = 0;
		led_current_sum = 0;			
		
		if(light_sum>0)
			measure.light	= light_sum					>> (mshift-1);	//2047 * 0.48mV/Lux * 1/2	= 0.4913 U/Lux -> 2.035 Lux/U (high sensor tolerance - rough value)
		else
			measure.light	= 0;
		light_sum = 0;
										
		if(temp_sum>0)
			measure.temp	= (temp_sum * 3580 / (temp_cal+mean)) - 2730;	//cal=85C, 0V=0K, temperature = 1/10C
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
	#ifdef IR_avail
		IR_init();
	#endif
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
	if (set.default_mode & mode_prev)
		mode_update(set.default_mode & !mode_prev);
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
		if(set.mode==mode_off)
			power_down();
		else
		{
			switch (set.mode)
			{
				//case mode_light_bar:	status_bar(measure.light, 4000, set.count); SPI_start(); break;
				case mode_mood_lamp:	Mood_Lamp(set.count); break;
				case mode_rainbow:		Rainbow(set.count);	break;
				case mode_colorswirl:	Colorswirl(set.count); break;
			}
			frame_update(false);		
		}
			
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~ 	MISC		~~~~~~~~~~~~~~~~~~~~~~~ */

void callback_init(void)
{
	dma_set_callback(DMA_CHANNEL_LED,(dma_callback_t) DMA_Led_int);
	tc_set_overflow_interrupt_callback(&TCD1,TCD1_OVF_int);
	adc_set_callback(&ADCA,ADCA_CH3_int);
	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	cpu_irq_enable();
}

void power_down(void)
{
	adc_disable(&ADCA); 
	sysclk_set_prescalers(SYSCLK_PSADIV_2,SYSCLK_PSBCDIV_1_1);
	while((set.mode==mode_off)&&!ioport_get_pin_level(USB_VBUS))
		sleepmgr_enter_sleep(); 
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADCA);
}

void button_update(Bool key_state)
{
	if(set.mode==mode_off)
	{
		if (set.default_mode & mode_prev)
		{
			if(prev_mode)
				mode_update(prev_mode);
			else
				mode_update(set.default_mode & !mode_prev);	
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
				status_led_update();
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
		{
			ioport_set_pin_level(MOSFET_en,HIGH);
			SetupDMA(set.mode&state_multi);
		}
		else
			ioport_set_pin_level(MOSFET_en,LOW);
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
			else
				status_led_off();
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
				if(set.stb_LED)
				{
					blink_en=false;
					tc_disable_cc_channels(&TCD0,TC_CCAEN);
					tc_enable_cc_channels(&TCD0,TC_CCBEN);
					tc_enable_cc_channels(&TCD0,TC_CCCEN);
					tc_write_cc(&TCD0, TC_CCB, set.stb_LED/4);
					tc_write_cc(&TCD0, TC_CCC, set.stb_LED);
				}
				else
					status_led_off();
			}
		}
	}
}

void status_led_off(void)
{
	tc_disable_cc_channels(&TCD0,TC_CCAEN);
	tc_disable_cc_channels(&TCD0,TC_CCBEN);
	tc_disable_cc_channels(&TCD0,TC_CCCEN);	
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

#ifdef IR_avail
volatile uint8_t ir_state = 0;

void TCC0_OVF_int(void)
{
	ir_state = 0;
	tc_write_clock_source(&TCC0,TC_CLKSEL_OFF_gc);
};

static void IR_init(void)
{
	ioport_set_pin_dir(IR_in,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(IR_in,IOPORT_MODE_TOTEM);
	ioport_set_pin_dir(IR_en,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(IR_en,true);
	ioport_set_pin_sense_mode(IR_in,IOPORT_SENSE_FALLING);
	tc_enable(&TCC0);
	tc_write_period(&TCC0,65000); //130ms @ 32MHz / 64
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	tc_set_overflow_interrupt_callback(&TCC0,TCC0_OVF_int);
	PORTB_INTCTRL = 1;
	PORTB_INT0MASK = PIN2_bm;
};

uint_fast8_t get_ir_bit(uint_fast16_t time)
{
	if (time < nec_zero_max) return 0;
	else if (time < nec_one_max) return 1;
};

ISR (PORTB_INT0_vect)
{
	static uint_fast8_t addr = 0;
	static uint_fast8_t addr_not = 0;
	static uint_fast8_t cmd = 0;
	static uint_fast8_t cmd_not = 0;

	static uint16_t ir_prev_time = 0;
	
	if (ir_state == 0) {tc_restart(&TCC0); tc_write_clock_source(&TCC0,TC_CLKSEL_DIV64_gc);}
	
	volatile uint16_t ir_time = tc_read_count(&TCC0) - ir_prev_time;
	ir_prev_time = tc_read_count(&TCC0);
	
	if (ir_state == 1) //Start Bit end
	{
		if ((nec_start_min) < ir_time && ir_time < (nec_start_max))
		{
			addr = 0; addr_not = 0; cmd = 0; cmd_not = 0;
		}
		else
		{
			ir_state = 0;
			tc_write_clock_source(&TCC0,TC_CLKSEL_OFF_gc);
			return;
		}
	}
	
	if (ir_state > 1 && ir_state < 10)
	{
		addr = addr << 1;
		addr |= get_ir_bit(ir_time);
	}
	if (ir_state > 9 && ir_state < 18)
	{
		addr_not = addr_not << 1;
		addr_not |= get_ir_bit(ir_time);
	}
	if (ir_state > 17 && ir_state < 26)
	{
		cmd = cmd << 1;
		cmd |= get_ir_bit(ir_time);
	}
	if (ir_state > 25 && ir_state < 34)
	{
		cmd_not = cmd_not << 1;
		cmd_not |= get_ir_bit(ir_time);
	}
	if (ir_state == 34)
	{
		addr_not = ~addr_not;
		cmd_not = ~cmd_not;
		if ((addr == addr_not) && (cmd == cmd_not))
		{
			remote_key(cmd);
		}
	}
	if (ir_state > 35)
	{
		if(ir_time < nec_repeat_max)
		{
			if ((addr == addr_not) && (cmd == cmd_not))
			{
				remote_key(cmd);
			}
			ir_state-=2;
			tc_restart(&TCC0);
		}
	}
	ir_state++;
};

#define IR_red_key 26
#define IR_green_key 154
#define IR_blue_key 162
#define IR_white_key 34
#define IR_off_key 2

void remote_key(uint8_t cmd)
{
	switch(cmd)
	{
		case IR_red_key:	back_buffer[0] = 255;
							back_buffer[1] = 0;
							back_buffer[2] = 0;
							break;
		case IR_green_key:	back_buffer[0] = 0;
							back_buffer[1] = 255;
							back_buffer[2] = 0;
							break;
		case IR_blue_key:	back_buffer[0] = 0;
							back_buffer[1] = 0;
							back_buffer[2] = 255;
							break;
		case IR_off_key:	if(set.mode==mode_off)
							{
								if (set.default_mode & mode_prev)
								{
									if(prev_mode)
										mode_update(prev_mode);
									else
										mode_update(set.default_mode & !mode_prev);	
								}
								else
									mode_update(set.default_mode);
							}
							else
							mode_update(mode_off);
							return;
		default: udi_cdc_putc(cmd);
	}
	mode_update(mode_usb_single);
	frame_update(true);
};

#endif
/* ~~~~~~~~~~~~~~~~~~~~~~~ 	EEPROM		~~~~~~~~~~~~~~~~~~~~~~~ */

void read_settings(void)
{
	nvm_eeprom_read_buffer(set_EE_offset*EEPROM_PAGE_SIZE, &set, sizeof(set));
};

void save_settings(void)
{
	nvm_eeprom_flush_buffer();
	uint_fast8_t *values = (uint_fast8_t *) &set;
	nvm_wait_until_ready();
	eeprom_enable_mapping();
	for (uint8_t i = 0; i < sizeof(set); i++) 
	{
		uint8_t value = *values;
		*(uint8_t*)(i + MAPPED_EEPROM_START) = value;
		values++;
	}
	eeprom_disable_mapping();
	nvm_eeprom_atomic_write_page(set_EE_offset);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~	LED-MODE	~~~~~~~~~~~~~~~~~~~~~~~ */

uint_fast16_t gamma_lut[256];

void frame_update(Bool buffer_update)
{
	if(buffer_update)
	{
		if(set.gamma)
			gamma_update=true;
		else
		{
			for(uint_fast16_t n=0;n<set.count*3;n++)
			{
				front_buffer[n]=back_buffer[n];
			}
			SPI_start();
		}
	}
	else if(gamma_update)
	{
		gamma_update=false;
		if(set.gamma&&!dma_channel_is_busy(DMA_CHANNEL_LED))
		{
			SPI_start(); // avoid delay if mapping takes longer than 500�s						
			gamma_map();
		}
	}
}

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
	if(++run==4)
		run=0;
}

uint_fast16_t multi(uint_fast16_t a, uint_fast16_t b)
{
	uint_fast16_t result;
	MultiU16X16toH16Round(result,a,b);
	return result;
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
		uint_fast16_t y=65535;
		uint_fast16_t root=(nvm_flash_read_byte( (flash_addr_t) &root_10[x]+1) <<8)+nvm_flash_read_byte( (flash_addr_t) &root_10[x]);
		
		for(uint_fast8_t cnt=gamma_pot;cnt;cnt--)
		y=multi(y,x*257);	//marginal error by shifting (equals div 256) instead of dividing by 255 is left
		for(uint_fast8_t cnt=gamma_dec;cnt;cnt--)
		y=multi(y,root);
		
		gamma_lut[x]=multi(y,1020);
		
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
		frame_update(true);
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
		frame_update(true);
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
		frame_update(true);
	}
}

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
			back_buffer[k]	=R;		//R
			back_buffer[k+1]=G;		//G
			back_buffer[k+2]=B;		//B
		}
		else
		{
			back_buffer[k]	=0;
			back_buffer[k+1]=0;
			back_buffer[k+2]=0;
		}
	}
	frame_update(true);
}

/*	~~~~~~~~~~~~~~~~~~~~~~~	 DMA/LED	~~~~~~~~~~~~~~~~~~~~~~~ */

struct dma_channel_config dmach_conf_single;
struct dma_channel_config dmach_conf_multi;

/* Init DMA setup struct */
void dma_init(void)
{
	//single
	memset(&dmach_conf_single, 0, sizeof(dmach_conf_single));

	dma_channel_set_burst_length		(&dmach_conf_single, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_single, 3);
	dma_channel_set_repeats				(&dmach_conf_single, set.count);

	dma_channel_set_src_reload_mode		(&dmach_conf_single, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_single, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_single, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_single, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_single, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_single, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_single, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot			(&dmach_conf_single);
	
	dma_channel_set_interrupt_level		(&dmach_conf_single, DMA_INT_LVL_MED);
	
	//multi
	memset(&dmach_conf_multi, 0, sizeof(dmach_conf_multi));
	
	dma_channel_set_burst_length		(&dmach_conf_multi, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_multi, set.count*3);

	dma_channel_set_src_reload_mode		(&dmach_conf_multi, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_multi, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_multi, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_multi, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_multi, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_multi, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_multi, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot			(&dmach_conf_multi);
	
	dma_channel_set_interrupt_level		(&dmach_conf_multi, DMA_INT_LVL_MED);
};

/* Config DMA in single / multi LED mode */
void SetupDMA(Bool multi)
{
	while (dma_channel_is_busy(DMA_CHANNEL_LED));
		dma_enable();
	if (multi)	
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_multi);
	else		
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
	dma_channel_enable(DMA_CHANNEL_LED);
}

/* Start DMA Transfer */
void SPI_start(void)
{
	if (dma_channel_is_busy(DMA_CHANNEL_LED) || TCD1.CTRLA)
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
	if(!(set.mode & state_multi) && (set.mode & state_on))
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
	tc_restart(&TCD1);
	tc_set_resolution(&TCD1,500000);
	gamma_update=true;
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
		get_USB_char();
		char temp=get_USB_char();
		if(temp!=set.count)
		{
			set.count=get_USB_char();
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
									frame_update(true);
									break;
			case mode_usb_multi:	return false;
									break;
			default:				return false;

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
		case cmd_set_save:	save_settings(); /*eeprom_write_block(&set, (uint8_t *) set_EE_offset, sizeof(set));*/ break;
		default:			return false;
	}
	return true;
}

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
}
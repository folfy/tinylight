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
 *	TCD1:	DMA-Wait
 */

#include "main.h"
#include "tiny_protocol.h"
#include <stdio.h>
#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "mul16x16.h"
#include "data.h"
#include "linsin.h"


// framerate statistics
uint_fast16_t volatile FPS = 0;
uint_fast16_t volatile frame_count = 0;

uint_fast8_t back_buffer[buffer_size*3];
uint_fast8_t front_buffer[buffer_size*3];

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
void ADCA_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);
#ifdef IR_avail
	static void IR_init(void);
#endif
void DMA_Led_int(dma_callback_t state);

//Button, Status_LED, FPS count
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
			button_time=time;
		else
			if(button_time)
				button_update(time >= button_time + button_long);
		button_mem=button_state;
	}
	if(button_state & ( time > (button_time+button_reset) ))
		wdt_reset_mcu();
	
	const uint_fast16_t led_alarm = 0.5*RTC_cycle;
	static uint_fast16_t led_time = 0;
	if(led_time >= led_alarm)
	{
		if(blink_en)
		{
			status_led_blink();
			led_time=alarm;			
		}
	}
	else
		led_time+=alarm;
	
	static uint_fast16_t fps_time = 0;
	fps_time+=alarm;
	if(fps_time >= 1*RTC_cycle)
	{
		FPS = frame_count;
		fps_time = 0;
		frame_count=0;
	}
	
	const uint_fast16_t ack_alarm = 1*RTC_cycle;
	static uint_fast16_t ack_time = 0;
	if(ack_time >= ack_alarm)
	{
		if(set.mode==mode_usb_ada)
		{
			udi_cdc_write_buf(&ack_ada,sizeof(ack_ada));
			ack_time = alarm;			
		}
	}
	else
		ack_time+=alarm;
	
	if(time > (UINT32_MAX-RTC_cycle*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&state_on) || (time >= (UINT32_MAX-alarm)) )	// delay as long as required or possible
			reset_do_soft_reset();
	}
	rtc_set_alarm(alarm_prev+=alarm);
}

void ADCA_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	if(ch_mask==ADC_CH0)
		mode_update(mode_error_BOP); //TODO: Test BOP, return measured voltage
	else
	{
		//increase unsigned resolution from 11 bit to 13 bit (max value = 2047 * 16 / 4 = 8,188
		const uint_fast8_t samples = 16, shift = 2, average = 8;
		static uint_fast8_t adc_cnt = samples, mean_cnt = average;	
		static int_fast16_t  voltage_sum = 0, current_sum = 0, light_sum = 0, temp_sum = 0;
		static adc_sample adc_mean={0,0,0,0};

		voltage_sum += adc_get_unsigned_result(&ADCA,ADC_CH0);
		current_sum += adc_get_unsigned_result(&ADCA,ADC_CH1);
		light_sum	+= adc_get_unsigned_result(&ADCA,ADC_CH2);
		temp_sum	+= adc_get_unsigned_result(&ADCA,ADC_CH3);
	
		if(!--adc_cnt)
		{
			if(voltage_sum>0)
				adc_mean.voltage	+= voltage_sum	>>shift;	// 8188 / 6.6V/V		* 1		= 1.2406 U/mV  -> 0.806 mV/U
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
				measure.temp = (adc_mean.temp*3580 / (temp_cal*8)) - 2730;	// TODO: temp_cal - test for delta V
				adc_mean.temp=0;
			}
		}
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
	mode_update(set.default_mode & !mode_prev);
		
	while(1)
	{
		handle_usb();
		if(set.mode==mode_off)
			power_down();
		else
		{
			switch (set.mode)
			{
				case mode_mood_lamp:	Mood_Lamp(set.count);	break;
				case mode_rainbow:		Rainbow(set.count);		break;
				case mode_colorswirl:	Colorswirl(set.count);	break;
			}
			handle_led_refresh();	
		}
			
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~ 	MISC		~~~~~~~~~~~~~~~~~~~~~~~ */

void callback_init(void)
{
	dma_set_callback(DMA_CHANNEL_LED,(dma_callback_t) DMA_Led_int);
	tc_set_overflow_interrupt_callback(&TCD1,TCD1_OVF_int);
	adc_set_callback(&ADCA,ADCA_int);
	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	cpu_irq_enable();
}

void power_down(void)
{
	sleeping = true;
	adc_disable(&ADCA); 
	sysclk_set_prescalers(SYSCLK_PSADIV_2,SYSCLK_PSBCDIV_1_1);
	while((set.mode==mode_off)&&!ioport_get_pin_level(USB_VBUS))
		sleepmgr_enter_sleep();
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADCA);
	sleeping = false;
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
					case mode_colorswirl:	mode_update(mode_usb_ada); break;
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

void count_update(uint_fast8_t count)
{
	if(set.count!=count)
	{
		set.count=count;
		dma_init();
		SetupDMA(set.mode&state_multi);
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

uint_fast8_t get_ir_bit(uint_fast16_t time);
void TCC0_OVF_int(void);
void handle_remote_key(uint_fast8_t addr, uint_fast8_t cmd, bool repeat);
void change_color(uint_fast8_t id, uint_fast8_t rgb_buffer[],bool save);

#define TCC0_cycle 32000000/64
#define nec_start_max 0.015*TCC0_cycle
#define nec_start_min 0.012*TCC0_cycle
#define nec_repeat_max 0.012*TCC0_cycle
#define nec_one_max 0.003*TCC0_cycle
#define nec_zero_max 0.002*TCC0_cycle

#define IR_addr 0
#define IR_colors_EE_offset 64

#define IR_off_key 2
#define IR_play_pause_key 130
#define IR_brightness_plus_key 58
#define IR_brightness_minus_key 186

#define IR_red_color_key 26
#define IR_dark_orange_color_key 42
#define IR_orange_color_key 10
#define IR_dark_yellow_color_key 56
#define IR_yellow_color_key 24

#define IR_green_color_key 154
#define IR_light_green_color_key 170
#define IR_bluegreen_color_key 138
#define IR_cyan_color_key 184
#define IR_petrol_color_key 152

#define IR_blue_color_key 162
#define IR_cobalt_blue_color_key 146
#define IR_blueviolet_color_key 178
#define IR_violet_color_key 120
#define IR_pink_color_key 88

#define IR_white_color_key 34
#define IR_rosa_color_key 18
#define IR_rosa_2_color_key 50
#define IR_skyblue_color_key 248
#define IR_skyblue_2_color_key 216

#define IR_red_plus_key 40
#define IR_red_minus_key 8
#define IR_green_plus_key 168
#define IR_green_minus_key 136
#define IR_blue_plus_key 104
#define IR_blue_minus_key 72

#define IR_DIY1_key 48
#define IR_DIY2_key 176
#define IR_DIY3_key 112
#define IR_DIY4_key 16
#define IR_DIY5_key 144
#define IR_DIY6_key 80

#define IR_jump3_key 32
#define IR_jump7_key 160
#define IR_fade3_key 96
#define IR_fade7_key 224

#define IR_flash_key 208
#define IR_auto_key 240
#define IR_speed_plus 232
#define IR_speed_minus 200

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
	tc_write_period(&TCC0,65534); //131ms @ 32MHz / 64
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	tc_set_overflow_interrupt_callback(&TCC0,TCC0_OVF_int);
	PORTB_INTCTRL = 1;
	PORTB_INT0MASK = PIN2_bm;
};

uint_fast8_t get_ir_bit(uint_fast16_t time)
{
	if (time < nec_zero_max) return 0;
	else if (time < nec_one_max) return 1;
	return 0xff; //ERROR
};

ISR (PORTB_INT0_vect)
{
	
	if (sleeping)
	{
		sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	}
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
	if (ir_state == 33)
	{
		addr_not = ~addr_not;
		cmd_not = ~cmd_not;
		if ((addr == addr_not) && (cmd == cmd_not))
		{
			handle_remote_key(addr, cmd, false);
		}
		else
		{
			ir_state = 0;
			tc_write_clock_source(&TCC0,TC_CLKSEL_OFF_gc);
			return;
		}		
	}
	if (ir_state > 34)
	{
		if(ir_time < nec_repeat_max)
		{
			handle_remote_key(addr, cmd, true);
			ir_state-=2;
			tc_restart(&TCC0);
		}
	}
	ir_state++;
};
	
void handle_remote_key(uint_fast8_t addr, uint_fast8_t cmd, bool repeat)
{
	uint_fast8_t new_mode = mode_usb_single;
	static uint_fast8_t last_cmd;
	bool save = false;
	if (addr != IR_addr) {return;} 
	if(!repeat)
	{
		if(last_cmd == cmd) save = true;
			switch(cmd)
			{
			case IR_red_color_key:			change_color(0,&back_buffer[0], save); break;
			case IR_green_color_key:		change_color(1,&back_buffer[0], save); break;
			case IR_blue_color_key:			change_color(2,&back_buffer[0], save); break;
			case IR_white_color_key:		change_color(3,&back_buffer[0], save); break;
			
			case IR_dark_orange_color_key:	change_color(4,&back_buffer[0], save); break;
			case IR_light_green_color_key:	change_color(5,&back_buffer[0], save); break;
			case IR_cobalt_blue_color_key:	change_color(6,&back_buffer[0], save); break;
			case IR_rosa_color_key:			change_color(7,&back_buffer[0], save); break;
			
			case IR_orange_color_key:		change_color(8,&back_buffer[0], save); break;
			case IR_bluegreen_color_key:	change_color(9,&back_buffer[0], save); break;
			case IR_blueviolet_color_key:	change_color(10,&back_buffer[0], save); break;
			case IR_rosa_2_color_key:		change_color(11,&back_buffer[0], save); break;
			
			case IR_dark_yellow_color_key:	change_color(12,&back_buffer[0], save); break;
			case IR_cyan_color_key:			change_color(13,&back_buffer[0], save); break;
			case IR_violet_color_key:		change_color(14,&back_buffer[0], save); break;
			case IR_skyblue_color_key:		change_color(15,&back_buffer[0], save); break;
			
			case IR_yellow_color_key:		change_color(16,&back_buffer[0], save); break;
			case IR_petrol_color_key:		change_color(17,&back_buffer[0], save); break;
			case IR_pink_color_key:			change_color(18,&back_buffer[0], save); break;
			case IR_skyblue_2_color_key:	change_color(19,&back_buffer[0], save); break;
					
			case IR_jump7_key:			new_mode = mode_rainbow; break;
			case IR_fade3_key:			new_mode = mode_mood_lamp; break;
			case IR_fade7_key:			new_mode = mode_colorswirl; break;
			case IR_off_key:	if(set.mode==mode_off)	{
									if (set.default_mode & mode_prev) {	if(prev_mode) { new_mode = prev_mode;} else { new_mode = set.default_mode & !mode_prev;} }
									else { new_mode = set.default_mode;	}
								} else	{
									new_mode = mode_off;
								}
								break;
			}
			mode_update(new_mode);
	}
	switch(cmd)
	{
		case IR_red_plus_key:		if (back_buffer[0] < 255)	back_buffer[0]++;	break;
		case IR_red_minus_key:		if (back_buffer[0] > 0)		back_buffer[0]--;	break;
		case IR_green_plus_key:		if (back_buffer[1] < 255)	back_buffer[1]++;	break;
		case IR_green_minus_key:	if (back_buffer[1] > 0)		back_buffer[1]--;	break;	
		case IR_blue_plus_key:		if (back_buffer[2] < 255)	back_buffer[2]++;	break;
		case IR_blue_minus_key:		if (back_buffer[2] > 0)		back_buffer[2]--;	break;
		default: last_cmd = cmd;
	}
	if (set.mode == mode_usb_single)	frame_update(true);
};

void change_color(uint_fast8_t id, uint_fast8_t rgb_buffer[], bool save)
{
	uint_fast8_t addr = id*3 + IR_colors_EE_offset;
	if (!save)
	{
		nvm_eeprom_read_buffer(addr,rgb_buffer,3);
	}
	else
	{
		nvm_eeprom_write_byte(addr,rgb_buffer[0]);
		nvm_eeprom_write_byte(addr+1,rgb_buffer[1]);
		nvm_eeprom_write_byte(addr+2,rgb_buffer[2]);
	}
}

#endif
/* ~~~~~~~~~~~~~~~~~~~~~~~ 	EEPROM		~~~~~~~~~~~~~~~~~~~~~~~ */

#define set_EE_offset		1    // page wise

void read_settings(void)
{
	nvm_eeprom_read_buffer(set_EE_offset*EEPROM_PAGE_SIZE, &set, sizeof(set));
	set.mode=mode_off;				//prevent undefined state during startup
	if(set.default_mode==0xFF)
	{
		set.default_mode	=	mode_prev;
		set.timeout_mode	=	mode_off;
		set.timeout_time	=	timeout_vbus;
		set.oversample		=	oversample_x4;			//4x oversample
		set.default_alpha	=	0xFF;
		set.gamma			=	2.2		*10;	//alpha=2.2
		set.smooth_time		=	5		*2;		//time=5s
		set.alpha_min		=	0;
		set.lux_max			=	1000	/40;	//brightness=1000lux
		set.stat_LED		=	50		*2.55;	//50%
		set.stb_LED			=	5		*2.55;	//5%
		set.count			=	80;
		set.OCP				=	ocp_off;
		set.OCP_time		=	0;
		set.SCP				=	scp_off;
		set.UVP				=	uvp_off;
		save_settings();
	}
	set.alpha=set.default_alpha;
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

uint_fast8_t gamma_lut[256];
uint_fast8_t gamma_ms[256];
uint_fast8_t ms_mask;

void frame_update(void)
{
	gamma_update=true;
}

void handle_led_refresh(void)
{
	if(gamma_update&&!dma_channel_is_busy(DMA_CHANNEL_LED))
	{
		gamma_update=false;
		SPI_start(); // avoid delay if mapping takes longer than 500µs
		gamma_map();
	}
}

void gamma_map(void)
{
	if(set.gamma)
	{
		static uint_fast8_t run=0;
		for(uint_fast16_t n=0;n<set.count*3;n++)
		{
			uint_fast8_t val=back_buffer[n];
			front_buffer[n]=gamma_lut[val];
			if(gamma_ms[val]>run)				//oversample pwm for increased resolution
				front_buffer[n]++;
		}
		run=(run+1)&ms_mask;		
	}
	else
	{
		for(uint_fast16_t n=0;n<set.count*3;n++)
			front_buffer[n]=back_buffer[n];
	}
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

	uint_fast16_t val_max=255<<set.oversample;

	switch(set.oversample)
	{
		case 0x01:	ms_mask=0x01;
		break;
		case 0x02:	ms_mask=0x03;
		break;
		case 0x03:	ms_mask=0x07;
		break;
		default:	ms_mask=0x00;
		break;
	}
	uint_fast16_t val_mask=ms_mask;
	
	for(uint8_t x=0;x++<255;)
	{
		uint_fast16_t y=65280;
		uint_fast16_t root=(nvm_flash_read_byte( (flash_addr_t) &root_10[x]+1) <<8)+nvm_flash_read_byte( (flash_addr_t) &root_10[x]);
		
		for(uint_fast8_t cnt=gamma_pot;cnt;cnt--)
		y=MulU16X16toH16Round(y,x*257);	//marginal error by shifting (equals div 256) instead of dividing by 255 is left
		for(uint_fast8_t cnt=gamma_dec;cnt;cnt--)
		y=MulU16X16toH16Round(y,root);

		y=MulU16X16toH16Round(y,val_max);
		gamma_lut[x]=y>>set.oversample;
		gamma_ms[x]=y&val_mask;
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
		frame_update();
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
		frame_update();
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
		sine1 -= rad_03;		// 0.03 rad
		if(sine1>=deg360)
			sine1-=(UINT_FAST16_MAX-deg360);
		frame_update();
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
	while (dma_channel_is_busy(DMA_CHANNEL_LED))
		;
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
	frame_count++;
}

void DMA_Led_int(dma_callback_t state)
{
	if(!(set.mode & state_multi))
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
	{
		udc_detach();
		mode_update(set.timeout_mode);
	}
}

void main_cdc_rx_notify(uint8_t port)
{
	usb_data_pending = true;
}

void handle_usb(void)
{
	//finite state machine
	static uint_fast8_t usb_state=0;
	uint8_t usb_buff[4];
	static uint_fast16_t buffer_pos=0;
	
	static uint_fast32_t usb_rx_time=0;
	static uint_fast8_t usb_state_prev=0; 
	uint_fast32_t time=rtc_get_time();
	if(usb_state_prev!=usb_state)
	{
		usb_rx_time=time;
		usb_state_prev=usb_state;
	}
	if((usb_state!=usb_state_idle)	&& (time>=(usb_rx_time+0.2*RTC_cycle)))
		usb_state=nack_flush(nack_timeout|usb_state);
	if(set.timeout_time)
	{
		if((set.mode&state_usb)		&& (time>=(usb_rx_time+set.timeout_time*RTC_cycle/10)))
		mode_update(set.timeout_mode);
	}
		
	if (usb_data_pending)
	{
		switch(usb_state)
		{
			case usb_state_idle:	if(udi_cdc_get_nb_received_data()>=3)
										{
											udi_cdc_read_buf(&usb_buff,3);
											if		(string_parser(usb_buff, preamble, sizeof(preamble)))
												usb_state=usb_state_cmd;
											else if	(string_parser(usb_buff, pre_ada, sizeof(pre_ada)))
												usb_state=usb_state_ada_header;
											else
												usb_state=nack_flush(nack_preamble);									
										}
										break;
			case usb_state_ada_header:	if(udi_cdc_get_nb_received_data()>=3)
										{
											udi_cdc_read_buf(&usb_buff,3);
											if((usb_buff[0]^usb_buff[1]^0x55)==usb_buff[2])
											{
												if(!usb_buff[0] && (usb_buff[1]<=buffer_size))
												{
													count_update(usb_buff[1]);
													mode_update(mode_usb_ada);
													usb_state=usb_state_ada_raw;
												}
												else
													usb_state=nack_flush(nack_ada_length);
											}
											else
												usb_state=nack_flush(nack_ada_crc);
										}
										break;
			case usb_state_ada_raw:		
										
										break;
			case usb_state_cmd:			switch(udi_cdc_getc())
										{
											case cmd_test:		udi_cdc_write_buf(&response,sizeof(response));
																udi_cdc_putc(protocol_rev);
																udi_cdc_putc(software_rev);
																udi_cdc_putc(board_rev);
																udi_cdc_putc(set.mode);
																usb_state=usb_state_idle;
																break;
											case cmd_raw_data:	if(set.mode==mode_usb_single)
																	usb_state=send_ack(usb_state_raw_single);
																else if(set.mode==mode_usb_multi)
																	usb_state=send_ack(usb_state_raw_multi);
																else
																	usb_state=nack_flush(nack_raw_mode);
																break;
											case cmd_measure:	udi_cdc_write_buf((adc_sample*)&measure,sizeof(measure));
																udi_cdc_putc(FPS >> 8);
																udi_cdc_putc(FPS);
																usb_state=usb_state_idle;
																break;
											case cmd_set_read:	usb_state=usb_state_set_read;
																break;
											case cmd_set_write:	usb_state=usb_state_set_write;
																break;
											case cmd_set_save:	save_settings();
																usb_state=send_ack(usb_state_idle);
																break;
											default:			usb_state=nack_flush(nack_command);
																break;
										}
										break;
			case usb_state_raw_single:	if(udi_cdc_get_nb_received_data()>=3)
										{
											udi_cdc_read_buf(&back_buffer,3);
											frame_update();									
										}
										break;
			case usb_state_raw_multi:	if((udi_cdc_get_nb_received_data()+buffer_pos)>=(set.count*3))
										{
											udi_cdc_read_buf(&back_buffer[buffer_pos],(set.count*3-buffer_pos));
											buffer_pos=0;
											usb_state=send_ack(usb_state_idle);
											frame_update();
										}	
										else if(udi_cdc_get_nb_received_data()>=64)	
											udi_cdc_read_buf(&back_buffer[buffer_pos],udi_cdc_get_nb_received_data());
										break;
			case usb_state_set_read:	switch(udi_cdc_getc())
										{
											case set_mode:				udi_cdc_putc(set.mode);					break;
											case set_default_mode:		udi_cdc_putc(set.default_mode);			break;
											case set_timeout_mode:		udi_cdc_putc(set.timeout_mode);			break;
											case set_timeout_time:		udi_cdc_putc(set.timeout_time);			break;
											case set_alpha:				udi_cdc_putc(set.alpha);				break;
											case set_default_alpha:		udi_cdc_putc(set.default_alpha);		break;
											case set_gamma:				udi_cdc_putc(set.gamma);				break;
											case set_smooth_time:		udi_cdc_putc(set.smooth_time);			break;
											case set_alpha_min:			udi_cdc_putc(set.alpha_min);			break;
											case set_lux_max:			udi_cdc_putc(set.lux_max);				break;
											case set_stat_Led:			udi_cdc_putc(set.stat_LED);				break;
											case set_stb_Led:			udi_cdc_putc(set.stb_LED);				break;
											case set_count:				udi_cdc_putc(set.count);				break;
											case set_OCP:				udi_cdc_putc(set.OCP);					break;
											case set_OCP_time:			udi_cdc_putc(set.OCP_time);				break;
											case set_SCP:				udi_cdc_putc(set.SCP);					break;
											case set_UVP:				udi_cdc_putc(set.UVP);					break;
											default:					usb_state=nack_flush(nack_read_add);	break;
										}
										break;
			case usb_state_set_write:	if(udi_cdc_get_nb_received_data()>=4)
										{
											udi_cdc_read_buf(&usb_buff,4);
											if((usb_buff[0]==usb_buff[1])&&(usb_buff[2]==usb_buff[3]))
												switch(usb_buff[0])
												{
													case set_mode:				mode_update(usb_buff[2]);				break;
													case set_default_mode:		set.default_mode = usb_buff[2];			break;
													case set_timeout_mode:		set.timeout_mode = usb_buff[2];			break;
													case set_timeout_time:		set.timeout_time = usb_buff[2];			break;
													case set_alpha:				set.alpha = usb_buff[2];				break;
													case set_default_alpha:		set.default_alpha = usb_buff[2];		break;
													case set_gamma:				set.gamma = usb_buff[2]; gamma_calc();	break;
													case set_smooth_time:		set.smooth_time = usb_buff[2];			break;
													case set_alpha_min:			set.alpha_min = usb_buff[2];			break;
													case set_lux_max:			set.lux_max = usb_buff[2];				break;
													case set_stat_Led:			set.stat_LED = usb_buff[2];				break;
													case set_stb_Led:			set.stb_LED = usb_buff[2];				break;
													case set_count:				count_update(usb_buff[2]);				break;
													case set_OCP:				set.OCP = usb_buff[2];					break;
													case set_OCP_time:			set.OCP_time = usb_buff[2];				break;
													case set_SCP:				set.SCP = usb_buff[2];					break;
													case set_UVP:				set.UVP = usb_buff[2];					break;
													default:					usb_state=nack_flush(nack_write_add);	break;
												}
											else
												usb_state=nack_flush(nack_write_crc);
										}										
										break;
		}
		usb_data_pending=udi_cdc_is_rx_ready();
	}
}

uint_fast8_t send_ack(uint_fast8_t new_mode)
{
	udi_cdc_putc(ack);
	return new_mode;
}

uint_fast8_t nack_flush(uint_fast8_t fault_code)
{
	udi_cdc_putc(nack_preamble);
	while(udi_cdc_is_rx_ready())
		udi_cdc_getc();
	return usb_state_idle;
}

Bool string_parser(uint8_t buff_a[], const uint8_t buff_b[], uint_fast8_t length)
{
	uint_fast8_t index;
	for(index=0;index<length;index++)
	{
		if(buff_a[index]!=buff_b[index])
			return false;
	}
	return true;
}

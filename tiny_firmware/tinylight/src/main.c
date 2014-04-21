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

#include <stdio.h>
#include <asf.h>
#include "tiny_protocol.h"
#include "led.h"
#include "mul16x16.h"
#include "usb.h"
#include "main.h"

uint_fast16_t temp_cal = 0;

volatile Bool mode_select = false;
volatile Bool blink_en=false;
volatile Bool blink_state;
volatile Bool sleeping=false;

settings set;
volatile adc_sample measure={0,0,0,0};

static void RTC_Alarm(uint32_t time);
static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result);
#ifdef IR_avail
	static void IR_init(void);
#endif
static void callback_init(void);
static void rtc_button(uint32_t time);
static void rtc_sled(void);

//Button, Status_LED, FPS count
static void RTC_Alarm(uint32_t time)
{
	const uint_fast16_t alarm_cycle = (RTC_time*RTC_freq+0.5);
	static uint_fast32_t alarm_prev = 0;
	
	rtc_button(time);
	
	rtc_sled();
	
	rtc_fps();
	
	if(time > (UINT32_MAX-RTC_freq*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&state_on) || (time >= (UINT32_MAX-alarm_cycle)) )	// delay as long as required or possible
			reset_do_soft_reset();	// TODO: Review RTC_Overflow
	}
	rtc_set_alarm(alarm_prev+=alarm_cycle);
}

static void ADC_int(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
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

		voltage_sum += adc_get_unsigned_result(&ADC,ADC_CH0);
		current_sum += adc_get_unsigned_result(&ADC,ADC_CH1);
		light_sum	+= adc_get_unsigned_result(&ADC,ADC_CH2);
		temp_sum	+= adc_get_unsigned_result(&ADC,ADC_CH3);
	
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
	dma_init(set.count);
	callback_init();
	#ifdef IR_avail
		IR_init();
	#endif
	temp_cal=adc_get_calibration_data(ADC_CAL_TEMPSENSE)/2; // calibration data is for unsigned mode, which has a positive offset of about 200
	adc_enable(&ADC);
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
				case mode_mood_lamp:	Mood_Lamp();	break;
				case mode_rainbow:		Rainbow();		break;
				case mode_colorswirl:	Colorswirl();	break;
			}
			handle_led_refresh();	
		}
			
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~ 	MISC		~~~~~~~~~~~~~~~~~~~~~~~ */

static void callback_init(void)
{
	extern void SPI_TIMER_OVF_int(void);
	extern void SPI_DMA_int(dma_callback_t state);
	
	dma_set_callback(DMA_CHANNEL_LED,(dma_callback_t) SPI_DMA_int);
	tc_set_overflow_interrupt_callback(&SPI_TIMER,SPI_TIMER_OVF_int);
	adc_set_callback(&ADC,ADC_int);
	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	cpu_irq_enable();
}

void power_down(void)
{
	sleeping = true;
	adc_disable(&ADC); 
	sysclk_set_prescalers(SYSCLK_PSADIV_2,SYSCLK_PSBCDIV_1_1);
	while((set.mode==mode_off)&&!ioport_get_pin_level(USB_VBUS))
		sleepmgr_enter_sleep();
	sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	adc_enable(&ADC);
	sleeping = false;
}

static void rtc_button(uint32_t time)
{
	const uint_fast16_t button_long = 0.25*RTC_freq;
	const uint_fast16_t button_reset = 7*RTC_freq;
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
}

void button_update(Bool key_state)
{
	if(set.mode==mode_off)
	{
		if (set.default_mode & mode_prev)
		{
			if(!mode_set_prev())
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

volatile uint_fast8_t prev_mode=mode_off;

Bool mode_set_prev(void)
{
	if(prev_mode)
	{
		mode_update(prev_mode);
		return true;
	}
	else
		return false;
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
		dma_init(set.count);
		SetupDMA(set.mode&state_multi);
	}
}

void status_led_update(void)
{

	if (mode_select)
	{
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_R,set.stat_LED/2);
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_G,set.stat_LED/2);
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_B,set.stat_LED/3);
		tc_enable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_R);
		tc_enable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_G);
		tc_enable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_B);
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
					tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
					tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_G);
					tc_enable_cc_channels( &SLED_TIMER,	SLED_TC_CCEN_B);
					tc_write_cc(&SLED_TIMER, SLED_TC_CC_B, set.stat_LED);
				}
				else
				{
					tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
					tc_enable_cc_channels( &SLED_TIMER,	SLED_TC_CCEN_G);
					tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_B);
					tc_write_cc(&SLED_TIMER, SLED_TC_CC_G, set.stat_LED);
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
				tc_write_cc(&SLED_TIMER, SLED_TC_CC_R, 0xFF);
			}
			else
			{
				if(set.stb_LED)
				{
					blink_en=false;
					tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
					tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_G);
					tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_B);
					tc_write_cc(&SLED_TIMER, SLED_TC_CC_R, set.stb_LED);
					tc_write_cc(&SLED_TIMER, SLED_TC_CC_G, set.stb_LED/4);
				}
				else
					status_led_off();
			}
		}
	}
}

void status_led_off(void)
{
	tc_disable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_R);
	tc_disable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_G);
	tc_disable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_B);	
}

static void status_led_blink(void);

void rtc_sled(void)
{
	const uint_fast8_t led_prescaler = 0.5/RTC_time+0.5;	//Round
	static uint_fast8_t led_cycle=led_prescaler;
	if(!--led_cycle)
	{
		if(blink_en)
		{
			status_led_blink();
			led_cycle=led_prescaler;
		}
	}
}

static void status_led_blink(void) //Error blinking
{
	if(blink_state)
	{
		tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
		blink_state=true;
	}
	else
	{
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
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
									if (set.default_mode & mode_prev) {	if(!mode_set_prev()) mode_update(set.default_mode & !mode_prev); }
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




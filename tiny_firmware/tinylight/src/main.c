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
#include "usb.h"
#include "set_sled.h"
#include "misc_adc.h"

#ifdef IR_avail
static void IR_init(void);
#endif

static void RTC_Alarm(uint32_t time);

int main (void)
{
	board_init();
	read_settings();
	dma_init();
	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	#ifdef IR_avail
		IR_init();
	#endif
	adc_init();
	gamma_calc();
	usb_init();
		
	while(1)
	{
		handle_usb();
		if(set.mode==MODE_OFF)
			power_down();
		else
		{
			switch (set.mode)
			{
				case MODE_MOODLAMP:	Mood_Lamp();	break;
				case MODE_RAINBOW:		Rainbow();		break;
				case MODE_COLORSWIRL:	Colorswirl();	break;
				default: break;
			}
			handle_led_refresh();	
		}
			
	}
}

//Button, Status_LED, FPS count
static void RTC_Alarm(uint32_t time)
{
	const uint_fast16_t alarm_cycle = (RTC_TIME*RTC_FREQ+0.5);
	static uint_fast32_t alarm_prev = 0;
	
	rtc_button(time);
	rtc_sled();
	rtc_fps();
	
	if(time > (UINT32_MAX-RTC_FREQ*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&STATE_ON) || (time >= (UINT32_MAX-alarm_cycle)) )	// delay as long as required or possible
			reset_do_soft_reset();	// TODO: Review RTC_Overflow
	}
	rtc_set_alarm(alarm_prev+=alarm_cycle);
}

#define EEMEM __attribute__((section(".eeprom")))
settings set_preset EEMEM =
{
	/*set.mode			=*/ MODE_OFF,
	/*set.mode_default	=*/	MODE_DEF_PREV_OFF,
	/*set.timeout_mode	=*/	MODE_OFF,
	/*set.timeout_time	=*/	TIMEOUT_VBUS,
	/*set.oversample	=*/	OVERSAMPLE_X4,			//4x oversample
	/*set.alpha			=*/ 0xFF,
	/*set.default_alpha	=*/	0xFF,
	/*set.gamma			=*/	2.2				*10,	//alpha=2.2
	/*set.smooth_time	=*/	5				*2,		//time=5s
	/*set.alpha_min		=*/	0,
	/*set.lux_max		=*/	1000			/40,	//brightness=1000lux
	/*set.stat_LED		=*/	50				*2.55,	//50%
	/*set.stb_LED		=*/	5				*2.55,	//5%
	/*set.count			=*/	80,
	/*set.SCP			=*/	SCP_OFF,
	/*set.UVP			=*/	UVP_OFF
};

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
	uint_fast8_t new_mode = MODE_USB_SINGLE;
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
					
			case IR_jump7_key:			new_mode = MODE_RAINBOW; break;
			case IR_fade3_key:			new_mode = MODE_MOODLAMP; break;
			case IR_fade7_key:			new_mode = MODE_COLORSWIRL; break;
			case IR_off_key:	if(set.mode==MODE_OFF)	{
									if (set.default_mode & STATE_PREV) {	if(!mode_set_prev()) mode_update(set.default_mode & !STATE_PREV); }
									else { new_mode = set.default_mode;	}
								} else	{
									new_mode = MODE_OFF;
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
	if (set.mode == MODE_USB_SINGLE)	frame_update(true);
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






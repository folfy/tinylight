/*
 * set_sled.c
 *
 * Created: 21.04.2014 20:03:58
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "tiny_protocol.h"
#include "led.h"
#include "set_sled.h"

volatile Bool mode_select = false;
volatile Bool blink_en = false;
volatile Bool blink_state;

settings set;

#define set_EE_offset		1    // page wise

static void button_update(Bool key_state);

//////////////////////////////////////////////////////////////////////////
/* EEPROM */

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

volatile uint_fast8_t prev_mode=mode_off;

//UNDONE: implement error mode handling
void mode_reset(void)
{
	set.mode=mode_off;
	status_led_update();
}

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

Bool mode_update(uint_fast8_t mode)
{
	if(set.mode&state_error)
		return false;
	if(set.mode == mode)
		return true;
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
	return true;
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

void rtc_button(uint32_t time)
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

static void button_update(Bool key_state)
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

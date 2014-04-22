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

settings set;

static void button_update(Bool key_state);

//////////////////////////////////////////////////////////////////////////
/* EEPROM */

void read_settings(void)
{
	nvm_eeprom_read_buffer(0, &set, sizeof(set));
	set.mode=mode_off;
	status_led_update();							//force sled update if new mode
	mode_update(set.default_mode&!state_prev);

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
	nvm_eeprom_atomic_write_page(0);
}

volatile enum mode_t prev_mode=mode_mood_lamp;

//UNDONE: implement error mode handling
void mode_reset(void)
{
	set.mode=mode_off;
	status_led_update();
}

void mode_set_prev(void)
{
	if(set.default_mode&state_prev)
		mode_update(prev_mode);
	else
		mode_update(set.default_mode);
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
		SetupDMA();
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
		SetupDMA();
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
		else if(button_time)
			button_update(time >= (button_time + button_long));
		button_mem=button_state;
	}
	if(button_state & ( time > (button_time+button_reset) ))
		wdt_reset_mcu();
}

static void button_update(Bool key_state)
{
	if(set.mode==mode_off)
		mode_set_prev();
	else
	{
		if(key_state)
		{
			//long
			if(set.mode&state_error)
				mode_reset();
			else if (set.mode != mode_off)
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


uint_fast8_t blink = 0; //TODO: make blink nicer

const uint_fast8_t led_prescaler = 0.5/RTC_time+0.5;	//Round
volatile uint_fast8_t led_cycle;

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
			blink=0;
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
				blink=1;
				led_cycle=led_prescaler;
				tc_enable_cc_channels( &SLED_TIMER,	SLED_TC_CCEN_R);
				tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_G);
				tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_B);
				tc_write_cc(&SLED_TIMER, SLED_TC_CC_R, 0xFF);
			}
			else
			{
				if(set.stb_LED)
				{
					blink=0;
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
	if(blink)
	{
		if(!--led_cycle)
		{
			status_led_blink();
			led_cycle=led_prescaler;
		}
	}
}

static void status_led_blink(void) //Error blinking
{
	if(blink==0x01)
	{
		tc_disable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
		blink=0x03;
	}
	else
	{
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
		blink=0x01;
	}
}

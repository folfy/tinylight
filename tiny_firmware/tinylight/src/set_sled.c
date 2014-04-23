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
#include "misc_adc.h"
#include "set_sled.h"

volatile Bool mode_select = false;

settings set;

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

static void button_update(Bool key_state);

//////////////////////////////////////////////////////////////////////////
/* EEPROM */

void read_settings(void)
{
	nvm_eeprom_read_buffer(0, &set, sizeof(set));
	set.mode=MODE_OFF;
	status_led_update();							//force sled update if new mode
	mode_update(set.default_mode&!STATE_PREV);

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

//////////////////////////////////////////////////////////////////////////
/* Settings access */

Bool write_set(enum set_address_t address, uint8_t val)
{
	switch(address)//UNDONE: Write write_val functions
	{
		case SET_MODE:				mode_update(val);					break;
		case SET_DEFAULT_MODE:		set.default_mode = val;				break;
		case SET_TIMEOUT_MODE:		set.timeout_mode = val;				break;
		case SET_TIMEOUT_TIME:		set.timeout_time = val;				break;
		case SET_ALPHA:				set.alpha = val;					break;
		case SET_DEFAULT_ALPHA:		set.default_alpha = val;			break;
		case SET_GAMMA:				write_gamma(val);					break;
		case SET_SMOOTH_TIME:		set.smooth_time = val;				break;
		case SET_ALPHA_MIN:			set.alpha_min = val;				break;
		case SET_LUX_MAX:			set.lux_max = val;					break;
		case SET_SLED_BRIGHT:		set.stat_LED = val;					break;
		case SET_SLED_DIM:			set.stb_LED = val;					break;
		case SET_COUNT:				write_count(val);					break;
		case SET_SCP:				write_scp(val);						break;
		case SET_UVP:				write_uvp(val);						break;
		default:					return false;
	}
	return true;
};

Bool read_set(uint8_t address, uint8_t *val)
{		
	if(address<sizeof(set))								
	{	
		*val=(uint8_t)*(&set.mode+address);
		return true;
	}
	return false;
}

volatile enum mode_t prev_mode=MODE_MOODLAMP;

//TODO: add error_reset function
void mode_error_reset(void)
{
	protection_reset();
	set.mode=MODE_OFF;
	status_led_update();
}

void mode_set_prev(void)
{
	if(set.default_mode&STATE_PREV)
		mode_update(prev_mode);
	else
		mode_update(set.default_mode);
}

Bool mode_update(uint_fast8_t mode)
{
	if(set.mode&STATE_ERROR)
		return false;
	if(set.mode == mode)
		return true;
	prev_mode=set.mode;
	set.mode = mode;
	if (set.mode&STATE_ON)
	{
		ioport_set_pin_level(MOSFET_en,HIGH);
		SetupDMA();
	}
	else
		ioport_set_pin_level(MOSFET_en,LOW);
	status_led_update();
	return true;
}

void write_gamma(uint_fast8_t gamma)
{
	set.gamma=gamma;
	gamma_calc();
}

void write_count(uint_fast8_t count)
{
	if(set.count!=count)
	{
		set.count=count;
		dma_update();
	}
}

void write_uvp(uint_fast8_t uvp_lim)
{
	set.uvp=uvp_lim;
	uvp_scp_update();	
}

void write_scp(uint_fast8_t scp_lim)
{
	set.scp=scp_lim;
	uvp_scp_update();	
}


//void write_UVP

//////////////////////////////////////////////////////////////////////////
/* Button */

void rtc_button(uint32_t time)
{
	const uint_fast16_t button_long = 0.25*RTC_FREQ;
	const uint_fast16_t button_reset = 7*RTC_FREQ;
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
	if(set.mode==MODE_OFF)
		mode_set_prev();
	else
	{
		if(key_state)
		{
			//long
			if(set.mode&STATE_ERROR)
				mode_error_reset();
			else if (set.mode != MODE_OFF)
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
					case MODE_MOODLAMP:		mode_update(MODE_RAINBOW); break;
					case MODE_RAINBOW:		mode_update(MODE_COLORSWIRL); break;
					//case mode_colorswirl:	mode_update(MODE_MOODLAMP); break;
					default:				mode_update(MODE_MOODLAMP); break;
				}
			}
			else
				mode_update(MODE_OFF);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/* SLED */

uint_fast8_t blink = 0; //TODO: make blink nicer

const uint_fast8_t led_prescaler = 0.5/RTC_TIME+0.5;	//round
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
		if(set.mode&STATE_ON)
		{
			blink=0;
			if(set.stat_LED)
			{
				if(set.mode&STATE_USB)
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
			if(set.mode&STATE_ERROR)
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

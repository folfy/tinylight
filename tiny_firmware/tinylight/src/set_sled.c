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

settings set;

static void button_update(Bool key_state);

//////////////////////////////////////////////////////////////////////////
/* EEPROM */

void read_settings(void)
{
	nvm_eeprom_read_buffer(0, &set, sizeof(set));
	if(set.default_mode==0xFF)						//Default settings if EEPROM is not programmed or erased by flip
	{
	set.default_mode	=	MODE_DEF_PREV_OFF;
	set.timeout_mode	=	MODE_OFF;
	set.timeout_time	=	TIMEOUT_VBUS;
	set.default_alpha	=	0xFF;
	set.oversample		=	OVERSAMPLE_X4;
	set.gamma			=	2.2				*10;	//alpha=2.2
	set.smooth_time		=	5				*2;		//time=5s
	set.alpha_min		=	0;
	set.lux_max			=	1000			/40;	//brightness=1000lux
	set.sled_bright		=	8				*2.55;	//8%
	set.sled_dim		=	2				*2.55;	//2%
	set.count			=	80;
	save_settings();
	}
	set.mode=MODE_OFF;
	sled_update();							//force sled update (no update if new mode = set.mode)
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
	switch(address)
	{
		case SET_MODE:				mode_update(val);					break;
		case SET_DEFAULT_MODE:		set.default_mode = val;				break;
		case SET_TIMEOUT_MODE:		set.timeout_mode = val;				break;
		case SET_TIMEOUT_TIME:		set.timeout_time = val;				break;
		case SET_OVERSAMPLE:		write_oversample(val);				break;
		case SET_ALPHA:				set.alpha = val;					break;
		case SET_DEFAULT_ALPHA:		set.default_alpha = val;			break;
		case SET_GAMMA:				write_gamma(val);					break;
		case SET_SMOOTH_TIME:		set.smooth_time = val;				break;
		case SET_ALPHA_MIN:			set.alpha_min = val;				break;
		case SET_LUX_MAX:			set.lux_max = val;					break;
		case SET_SLED_BRIGHT:		write_sled_bright(val);				break;
		case SET_SLED_DIM:			write_sled_dim(val);				break;
		case SET_COUNT:				write_count(val);					break;
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

void mode_set_prev(void)
{
	if(set.default_mode&STATE_PREV)
		mode_update(prev_mode);
	else
		mode_update(set.default_mode);
}

uint_fast8_t ada_mem[3];

void mode_update(uint_fast8_t mode)
{
	if(set.mode == mode)
		return;
	prev_mode = set.mode;
	set.mode = mode;
	if (set.mode&STATE_ON)
	{
		ioport_set_pin_level(MOSFET_en,HIGH);
		SetupDMA();
	}
	else
		ioport_set_pin_level(MOSFET_en,LOW);
	if(prev_mode==MODE_USB_ADA)
	{
		write_gamma(ada_mem[0]);
		write_count(ada_mem[1]);
	}
	else if(mode==MODE_USB_ADA)
	{
		ada_mem[0]=set.gamma;
		ada_mem[1]=set.count;
		write_gamma(GAMMA_OFF);
	}
	sled_update();
}

void write_oversample(uint_fast8_t oversamples)
{
	if(set.oversample==oversamples)
		return;
	set.oversample=oversamples;
	if(set.gamma)
		gamma_calc();
}

void write_gamma(uint_fast8_t gamma)
{
	if(set.gamma==gamma)
		return;
	set.gamma=gamma;
	if(set.gamma)
		gamma_calc();
}

void write_sled_bright(uint_fast8_t brightness)
{
	if(set.sled_bright==brightness)
		return;
	set.sled_bright=brightness;
	sled_update();
}

void write_sled_dim(uint_fast8_t brightness)
{
	if(set.sled_dim==brightness)
		return;
	set.sled_dim=brightness;
	sled_update();
}

void write_count(uint_fast8_t count)
{
	if(set.count==count)
		return;
	if(set.count>BUFFER_SIZE)
		count=BUFFER_SIZE;
	if(set.count!=count)
	{
		set.count=count;
		dma_update_count();
	}
}

//////////////////////////////////////////////////////////////////////////
/* Button */

void rtc_button(uint32_t time)
{
	const uint_fast16_t button_long = 0.25*RTC_FREQ;
	const uint_fast16_t button_reset = 4*RTC_FREQ;
	static uint_fast32_t button_time;
	static Bool button_mem = false;
	Bool button_state = ioport_get_pin_level(BUTTON);
	if(button_state != button_mem)
	{
		if(button_state)
			button_time=time;
		else
			button_update(time >= (button_time + button_long));
		button_mem=button_state;
	}
	#ifdef debug
	if(button_state & ( time > (button_time+button_reset) ))
		wdt_reset_mcu();
	#endif
}

volatile Bool mode_select = false;

static void button_update(Bool key_state)
{
	if(set.mode==MODE_OFF)
		mode_set_prev();
	else
	{
		if(key_state)	//TODO: improve mode_select -  implement bright_adjust
		{
			//long
			if (set.mode != MODE_OFF)
			{
				mode_select ^= true;
				sled_update();
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

uint_fast8_t blink = 0;

const uint_fast8_t led_prescaler = 0.5/RTC_TIME+0.5;	//round
volatile uint_fast8_t led_cycle;

static void sled_set(uint_fast8_t red, uint_fast8_t green, uint_fast8_t blue);

void sled_update(void)
{

	if (mode_select)
		sled_set(set.sled_bright/2,set.sled_bright/2,set.sled_bright/4);
	else
	{
		if(set.mode&STATE_ON)
		{
			blink=0;
			if(set.mode&STATE_USB)
				sled_set(0,0,set.sled_bright);
			else
				sled_set(0,set.sled_bright,0);
		}
		else
		{
			if(set.mode&STATE_ERROR)	//TODO: error handling (sled, blink,...)
			{
				blink=1;
				led_cycle=led_prescaler;
				sled_set(255,0,0);
			}
			else
			{
				blink=0;
				sled_set(set.sled_dim,set.sled_dim/4,0);
			}
		}
	}
}

static void sled_set(uint_fast8_t red, uint_fast8_t green, uint_fast8_t blue)
{
	if(red)
	{
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R);
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_R, red);
	}
	else
		tc_disable_cc_channels(&SLED_TIMER,SLED_TC_CCEN_R);
	
	if(green)
	{
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_G);
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_G, green);		
	}
	else
		tc_disable_cc_channels(&SLED_TIMER,SLED_TC_CCEN_G);	
	
	if(blue)
	{
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_B);
		tc_write_cc(&SLED_TIMER, SLED_TC_CC_B, blue);
	}
	else
		tc_disable_cc_channels(&SLED_TIMER,SLED_TC_CCEN_B);
}

static void sled_blink(void);

void rtc_sled(void)
{
	if(blink)
	{
		if(!--led_cycle)
		{
			sled_blink();
			led_cycle=led_prescaler;
		}
	}
}

static void sled_blink(void) //Error blinking
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

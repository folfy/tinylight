/*
 * settings_sled.c
 *
 * Created: 21.04.2014 20:03:58
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "libs/protocol/tiny_protocol.h"
#include "libs/interfaces/led.h"
#include "libs/modules/sleep_adc.h"
#include "settings_sled.h"

settings set;

static void button_update(Bool key_state);

//////////////////////////////////////////////////////////////////////////
/* EEPROM */

void read_settings(void)
{
	nvm_eeprom_read_buffer(0, &set, sizeof(set));
	if(set.default_mode==0xFF)						//Default settings if EEPROM is not programmed or erased by flip
	{
	set.default_mode	=	MODE_DEF_PREV_SLEEP;
	set.timeout_mode	=	MODE_SLEEP;
	set.timeout_time	=	TIMEOUT_VBUS;
	set.default_alpha	=	0xFF;
	set.oversample		=	OVERSAMPLE_X4;
	set.gamma			=	2.20			*20;	//alpha=2.2
	set.smooth_time		=	5				*2;		//time=5s
	set.alpha_min		=	0;
	set.lux_max			=	1000			/40;	//brightness=1000lux
	set.sled_bright		=	8				*2.55;	//8%
	set.sled_dim		=	2				*2.55;	//2%
	set.count			=	80;
	set.fps_lim			=	60;
	save_settings();
	}
	set.mode=MODE_SLEEP;
	sled_update();							//force sled update (no update if new mode = set.mode)
	mode_update(set.default_mode&!STATE_PREV);
	gamma_calc();
	fps_lim_update();
	set.alpha=set.default_alpha;
}

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
		case SET_FPS_LIM:			write_fps_lim(val);					break;
		case SET_DEBUG:													break;
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

void mode_update(uint_fast8_t mode)
{
	if(set.mode == mode)
		return;
	if((set.mode&STATE_ON)&&!(set.mode&STATE_USB))
		prev_mode = set.mode;
		
	set.mode = mode;
	if (mode&STATE_ON)
	{
		ioport_set_pin_level(MOSFET_en,HIGH);
		SetupDMA();
	}
	else
		ioport_set_pin_level(MOSFET_en,LOW);
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
	if(gamma)
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

void write_fps_lim(uint_fast8_t fps)
{
	if(set.fps_lim==fps)
	return;
	if(!fps)
	fps=1;
	set.fps_lim=fps;
	fps_lim_update();
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
	#ifdef DEBUG
	if(button_state & ( time > (button_time+button_reset) ))
		wdt_reset_mcu();
	#endif
}

typedef enum {
	BUTTON_DEFAULT     = 0,
	BUTTON_MODE_SEL    = 1,
	BUTTON_BRIGHT_ADJ  = 2
} button_mode_t;

volatile button_mode_t button_mode = BUTTON_DEFAULT;

static void button_mode_set(button_mode_t new_mode);

static void button_update(Bool key_long)
{
	if(set.mode==MODE_SLEEP)
		mode_set_prev();
	else
	{
		switch (button_mode) {
			case BUTTON_DEFAULT: 
				if (key_long) {
					button_mode=BUTTON_MODE_SEL;
				} else {
					mode_update(MODE_SLEEP);
				} 
				break;
			case BUTTON_MODE_SEL:
				//Change Mode
				switch(set.mode)
				{
					case MODE_MOODLAMP:		button_mode_set(MODE_RAINBOW); break;
					case MODE_RAINBOW:		button_mode_set(MODE_COLORSWIRL); break;
					//case mode_colorswirl:	button_mode_set(MODE_MOODLAMP); break;
					default:				button_mode_set(MODE_MOODLAMP); break;
				}
				break;
			default: break;
		}
	}
}

static void button_mode_set(button_mode_t new_mode)
{
	button_mode=new_mode;
	sled_update();
}

//////////////////////////////////////////////////////////////////////////
/* SLED */

bool blink_state = 0;

const uint_fast8_t led_prescaler = 0.5/RTC_TIME+0.5;	//round
volatile uint_fast8_t led_cycle;

/* Init status LED */
void sled_init(void)
{
	ioport_set_pin_dir(SLED_R, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SLED_G, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SLED_B, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(SLED_R, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_mode(SLED_G, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_mode(SLED_B, IOPORT_MODE_INVERT_PIN);
	
	tc_enable(&SLED_TIMER);
	tc_set_wgm(&SLED_TIMER, TC_WG_SS);		//Single Slope PWM
	tc_write_period(&SLED_TIMER, 0xFF);		//8 Bit PWM
	tc_set_resolution(&SLED_TIMER, 1000000);//3.92 kHz PWM freq
	
	tc_write_cc(&SLED_TIMER,SLED_TC_CC_B,255);		//SLED: Cyan
	tc_write_cc(&SLED_TIMER,SLED_TC_CC_G,255);
	tc_enable_cc_channels(&SLED_TIMER,SLED_TC_CCEN_B|SLED_TC_CCEN_G);
}

static void sled_set(uint_fast8_t red, uint_fast8_t green, uint_fast8_t blue, bool blinking);

void sled_update(void)
{

	if (button_mode != BUTTON_DEFAULT)
		switch (button_mode) {
			case BUTTON_BRIGHT_ADJ: sled_set(set.sled_bright/2, set.sled_bright/2, set.sled_bright/4, true);  break;
			default:                sled_set(set.sled_bright/2, set.sled_bright/2, set.sled_bright/4, false); break;
		}
	else
	{
		if(set.mode&STATE_ON)
		{
			if(set.mode&STATE_USB)
				sled_set(0, 0, set.sled_bright, false);
			else
				sled_set(0, set.sled_bright, 0, false);
		}
		else
		{
			if(set.mode&STATE_RES) {	//TODO: error handling (sled, generic blink,...)
				sled_set(set.sled_bright, 0, 0, true);
			} else {
				sled_set(set.sled_dim, set.sled_dim/4, 0, false);
			}
		}
	}
}

static inline void sled_ch_set(enum tc_cc_channel_mask_enable_t TC_CCEN, enum tc_cc_channel_t TC_CC, uint_fast8_t value);

static void sled_set(uint_fast8_t red, uint_fast8_t green, uint_fast8_t blue, bool blinking)
{
	if (blinking) {
		blink_state = true;
		led_cycle=led_prescaler;
	} else {
		led_cycle=0;
	}
	sled_ch_set(SLED_TC_CCEN_R, SLED_TC_CC_R, red);
	sled_ch_set(SLED_TC_CCEN_G, SLED_TC_CC_G, green);
	sled_ch_set(SLED_TC_CCEN_B, SLED_TC_CC_B, blue);
}

static inline void sled_ch_set(enum tc_cc_channel_mask_enable_t TC_CCEN, enum tc_cc_channel_t TC_CC, uint_fast8_t value)
{
	if (value == 0) {
		tc_disable_cc_channels(&SLED_TIMER, TC_CCEN);
		} else {
		tc_enable_cc_channels(&SLED_TIMER, TC_CCEN);
		tc_write_cc(&SLED_TIMER, TC_CC, value);
	}
}

static void sled_blink(void);

void rtc_sled(void)
{
	if (led_cycle != 0) {
		if(--led_cycle == 0)
		{
			sled_blink();
			led_cycle=led_prescaler;
		}
	}
}

static void sled_blink(void) //Error blinking
{
	blink_state ^= true;
	if(blink_state) {
		tc_enable_cc_channels(&SLED_TIMER,	SLED_TC_CCEN_R | SLED_TC_CCEN_G | SLED_TC_CCEN_B);
		} else {
		tc_disable_cc_channels(&SLED_TIMER, SLED_TC_CCEN_R | SLED_TC_CCEN_G | SLED_TC_CCEN_B);
	}
}


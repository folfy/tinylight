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
<<<<<<< HEAD
#ifdef RF_avail
#if		RF_avail==2
	static void BT_init(void);
#endif
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
		FPS = frame_count;
		fps_time = 0;
		frame_count=0;
	}
	
	if(time > (UINT32_MAX-RTC_cycle*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&state_on) || (time >= (UINT32_MAX-alarm)) )	// delay as long as required or possible
			reset_do_soft_reset();
	}
	rtc_set_alarm(alarm_prev+=alarm);
}
=======
>>>>>>> dev

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
<<<<<<< HEAD
	udc_start();
	if (!udc_include_vbus_monitoring())
	{
		if(ioport_get_pin_level(USB_VBUS))
			udc_attach();
		PORTD_INTCTRL = 1;
	}
	mode_update(set.default_mode & !mode_prev);
	
	#ifdef RF_avail
		#if		RF_avail==2
			BT_init();
		#endif
	#endif
=======
	usb_init();
		
>>>>>>> dev
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
	ioport_set_pin_level(IR_en,HIGH);
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
<<<<<<< HEAD

#ifdef RF_avail
#if		RF_avail==2
static void BT_init(void)
{
	ioport_set_pin_dir(BT_DSR,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BT_DSR,IOPORT_MODE_TOTEM);
	
	ioport_set_pin_dir(BT_EN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BT_EN,HIGH);
	
	ioport_set_pin_dir(BT_RST,IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(BT_RST_UART,IOPORT_DIR_OUTPUT);
		
	ioport_set_pin_dir(BT_RX,IOPORT_DIR_INPUT);
	
	ioport_set_pin_dir(BT_TX,IOPORT_DIR_OUTPUT);
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
	      .baudrate = 9600,
	      .charlength = USART_CHSIZE_8BIT_gc,
	      .paritytype = USART_PMODE_DISABLED_gc,
	      .stopbits = false
	};
	sysclk_enable_module(SYSCLK_PORT_C, PR_USART1_bm);
	usart_init_rs232(BT_USART, &USART_SERIAL_OPTIONS);
	usart_set_rx_interrupt_level(BT_USART, USART_INT_LVL_LO);
};

ISR(BT_USART_vect)
{
	udi_cdc_putc(usart_getchar(BT_USART));
};

#endif
#endif
/* ~~~~~~~~~~~~~~~~~~~~~~~ 	EEPROM		~~~~~~~~~~~~~~~~~~~~~~~ */

void read_settings(void)
{
	nvm_eeprom_read_buffer(set_EE_offset*EEPROM_PAGE_SIZE, &set, sizeof(set));
	if(set.default_mode==0xFF)
	{
		set.default_mode	=	mode_prev;
		set.timeout_mode	=	mode_off;
		set.timeout_time	=	timeout_vbus;
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
=======
>>>>>>> dev





<<<<<<< HEAD
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
		sine1 -= rad_03;		// 0.03 rad
		if(sine1>=deg360)
			sine1-=(UINT_FAST16_MAX-deg360);
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
	frame_count++;
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
		char cnt_h=get_USB_char();
		char cnt_l=get_USB_char();
		if((cnt_l^cnt_h)!=get_USB_char())
			return false;
		set.count=cnt_l;
		SetupDMA(true);
		set.mode=mode_usb_ada;
		return true;
	}
	#ifdef RF_avail
	#if		RF_avail==2
		else if(usb_rx==pre_BT[0])
		{
			for(uint_fast8_t i=1;i<sizeof(pre_BT);i++)
			{
				if (get_USB_char() != pre_BT[i])
				return false;
			}
			while(udi_cdc_is_rx_ready())
			{
				usart_putchar(BT_USART,get_USB_char());
			}
			return true;
		}
	#endif
	#endif
	
	switch (get_USB_char())
	{
		case cmd_test:		udi_cdc_write_buf(&response,sizeof(response));
							udi_cdc_putc(protocol_rev);
							udi_cdc_putc(software_rev);
							udi_cdc_putc(board_rev);
							udi_cdc_putc(set.mode);
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
							udi_cdc_putc(FPS >> 8);
							udi_cdc_putc(FPS);
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
								case set_mode:				mode_update(set_value);					break;
								case set_default_mode:		set.default_mode = set_value;			break;
								case set_timeout_mode:		set.timeout_mode = set_value;			break;
								case set_timeout_time:		set.timeout_time = set_value;			break;
								case set_alpha:				set.alpha = set_value;					break;
								case set_default_alpha:		set.default_alpha = set_value;			break;
								case set_gamma:				set.gamma = set_value; gamma_calc();	break;
								case set_smooth_time:		set.smooth_time = set_value;			break;
								case set_alpha_min:			set.alpha_min = set_value;				break;
								case set_lux_max:			set.lux_max = set_value;				break;
								case set_stat_Led:			set.stat_LED = set_value;				break;
								case set_stb_Led:			set.stb_LED = set_value;				break;
								case set_count:				set.count = set_value;					break;
								case set_OCP:				set.OCP = set_value;					break;
								case set_OCP_time:			set.OCP_time = set_value;				break;
								case set_SCP:				set.SCP = set_value;					break;
								case set_UVP:				set.UVP = set_value;					break;
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
=======
>>>>>>> dev

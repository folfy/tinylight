/*
 * IR.c
 *
 * Created: 29.04.2014 16:40:37
 *  Author: Martin
 */ 

//#ifdef IR_avail

#include <stdio.h>
#include <asf.h>
#include "tiny_protocol.h"
#include "IR.h"
#include "led.h"
#include "set_sled.h"

volatile uint8_t ir_state = 0;

void TCC0_OVF_int(void)
{
	ir_state = 0;
	tc_write_clock_source(&IR_TIMER,TC_CLKSEL_OFF_gc);
};

void IR_init(void)
{
	ioport_set_pin_dir(IR_in,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(IR_in,IOPORT_MODE_TOTEM);
	ioport_set_pin_dir(IR_en,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(IR_en,HIGH);
	ioport_set_pin_sense_mode(IR_in,IOPORT_SENSE_FALLING);
	tc_enable(&IR_TIMER);
	tc_write_period(&IR_TIMER,65534); //131ms @ 32MHz / 64
	tc_set_overflow_interrupt_level(&IR_TIMER, TC_INT_LVL_LO);
	tc_set_overflow_interrupt_callback(&IR_TIMER,TCC0_OVF_int);
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
	//if (sleeping)
	//{
		sysclk_set_prescalers(CONFIG_SYSCLK_PSADIV,CONFIG_SYSCLK_PSBCDIV);
	//}
	static uint_fast8_t addr = 0;
	static uint_fast8_t addr_not = 0;
	static uint_fast8_t cmd = 0;
	static uint_fast8_t cmd_not = 0;

	static uint16_t ir_prev_time = 0;
	
	if (ir_state == 0) {tc_restart(&IR_TIMER); tc_write_clock_source(&IR_TIMER,TC_CLKSEL_DIV64_gc);}
	
	volatile uint16_t ir_time = tc_read_count(&IR_TIMER) - ir_prev_time;
	ir_prev_time = tc_read_count(&IR_TIMER);
	
	if (ir_state == 1) //Start Bit end
	{
		if ((nec_start_min) < ir_time && ir_time < (nec_start_max))
		{
			addr = 0; addr_not = 0; cmd = 0; cmd_not = 0;
		}
		else
		{
			ir_state = 0;
			tc_write_clock_source(&IR_TIMER,TC_CLKSEL_OFF_gc);
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
			tc_write_clock_source(&IR_TIMER,TC_CLKSEL_OFF_gc);
			return;
		}
	}
	if (ir_state > 34)
	{
		if(ir_time < nec_repeat_max)
		{
			handle_remote_key(addr, cmd, true);
			ir_state-=2;
			tc_restart(&IR_TIMER);
		}
	}
	ir_state++;
};

void handle_remote_key(uint_fast8_t addr, enum ir_key_t cmd, bool repeat)
{
	static uint_fast8_t last_cmd;
	bool save = false;
	bool set_single_mode = true;
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
			
			case IR_DIY1_key:				change_color(20,&back_buffer[0], save); break;
			case IR_DIY2_key:				change_color(21,&back_buffer[0], save); break;
			case IR_DIY3_key:				change_color(22,&back_buffer[0], save); break;
			case IR_DIY4_key:				change_color(23,&back_buffer[0], save); break;
			case IR_DIY5_key:				change_color(24,&back_buffer[0], save); break;
			case IR_DIY6_key:				change_color(25,&back_buffer[0], save); break;
			
			case IR_jump7_key:				mode_update(MODE_RAINBOW); break;
			case IR_fade3_key:				mode_update(MODE_MOODLAMP); break;
			case IR_fade7_key:				mode_update(MODE_COLORSWIRL); break;
			case IR_off_key:				set_single_mode = false; if(set.mode==MODE_OFF) { mode_set_prev(); } else { mode_update(MODE_OFF);} break;
			default:						set_single_mode = false;
		}
		if (set_single_mode) mode_update(MODE_SINGLE);
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
	if (set.mode == MODE_SINGLE)	frame_update();
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
};

//#endif
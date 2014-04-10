/*
 * IR.c
 *
 * Created: 10.04.2014 12:48:52
 *  Author: Martin
 */ 

#ifndef IR_C_
#define IR_C_

#include <avr/io.h>
#include "asf.h"
#include "IR.h"

uint_fast8_t get_ir_byte(uint_fast16_t time)
{
	if (time < nec_zero_max) return 0;
	else if (time < nec_one_max) return 1;
	
};

ISR (PORTB_INT0_vect)
{
	static uint_fast8_t addr = 0;
	static uint_fast8_t addr_not = 0;
	static uint_fast8_t cmd = 0;
	static uint_fast8_t cmd_not = 0;
	
	static volatile uint8_t ir_state = 0;
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
		addr |= get_ir_byte(ir_time);
	}
	if (ir_state > 9 && ir_state < 18)
	{
		addr_not = addr_not << 1;
		addr_not |= get_ir_byte(ir_time);
	}
	if (ir_state > 17 && ir_state < 26)
	{
		cmd = cmd << 1;
		cmd |= get_ir_byte(ir_time);
	}
	if (ir_state > 25 && ir_state < 34)
	{
		cmd_not = cmd_not << 1;
		cmd_not |= get_ir_byte(ir_time);
	}
	if (ir_state == 34)
	{
		if (addr == ~addr_not && cmd == ~cmd_not)
		{
			udi_cdc_putc(cmd);
		}
	}
	if (ir_state > 35)
	{
		if(ir_time < nec_repeat_max)
		{
				if (addr == ~addr_not && cmd == ~cmd_not)
				{
					udi_cdc_putc(cmd);
				}
				ir_state-=2;
		}
	}
	ir_state++;
};



#endif
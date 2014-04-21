/*
 * usb.c
 *
 * Created: 21.04.2014 11:59:50
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "tiny_protocol.h"
#include "led.h"
#include "set_sled.h"
#include "usb.h"

extern adc_sample measure;

static uint_fast8_t send_ack(uint_fast8_t new_mode);
static uint_fast8_t nack_flush(uint_fast8_t fault_code);
static Bool string_parser(uint8_t buff_a[], const uint8_t buff_b[], uint_fast8_t length);

//////////////////////////////////////////////////////////////////////////
/* USB */

//VBus detection
ISR (Vbus_INT0_vect)
{
	if(ioport_get_pin_level(USB_VBUS))
	udc_attach();
	else
	{
		udc_detach();
		mode_update(set.timeout_mode);
	}
}

void usb_init()
{	
	udc_start();
	if (!udc_include_vbus_monitoring())
	{
		if(ioport_get_pin_level(USB_VBUS))
			udc_attach();
		PORTD_INTCTRL = 1;
	}
}

void handle_usb(void)
{
	//finite state machine
	static uint_fast8_t usb_state=0;
	uint8_t usb_buff[4];
	static uint_fast16_t buffer_pos=0;
	
	if (udi_cdc_is_rx_ready())
	{
		switch(usb_state)
		{
			case usb_state_idle:	if(udi_cdc_get_nb_received_data()>=3)
			{
				udi_cdc_read_buf(&usb_buff,3);
				if		(string_parser(usb_buff, preamble, sizeof(preamble)))
				usb_state=usb_state_cmd;
				else if	(string_parser(usb_buff, pre_ada, sizeof(pre_ada)))
				usb_state=usb_state_ada_header;
				else
				usb_state=nack_flush(nack_preamble);
			}
			break;
			case usb_state_ada_header:	if(udi_cdc_get_nb_received_data()>=3)
			{
				udi_cdc_read_buf(&usb_buff,3);
				if((usb_buff[0]^usb_buff[1]^0x55)==usb_buff[2])
				{
					if(!usb_buff[0] && (usb_buff[1]<=buffer_size))
					{
						count_update(usb_buff[1]);
						mode_update(mode_usb_ada);
						usb_state=usb_state_ada_raw;
					}
					else
					usb_state=nack_flush(nack_ada_length);
				}
				else
				usb_state=nack_flush(nack_ada_crc);
			}
			break;
			case usb_state_ada_raw:
			
			break;
			case usb_state_cmd:			switch(udi_cdc_getc())
			{
				case cmd_test:		udi_cdc_write_buf(&response,sizeof(response));
				udi_cdc_putc(protocol_rev);
				udi_cdc_putc(software_rev);
				udi_cdc_putc(board_rev);
				udi_cdc_putc(set.mode);
				usb_state=usb_state_idle;
				break;
				case cmd_raw_data:	if(set.mode==mode_usb_single)
				usb_state=send_ack(usb_state_raw_single);
				else if(set.mode==mode_usb_multi)
				usb_state=send_ack(usb_state_raw_multi);
				else
				usb_state=nack_flush(nack_raw_mode);
				break;
				case cmd_measure:	udi_cdc_write_buf((adc_sample*)&measure,sizeof(measure));
				udi_cdc_putc(FPS>>8);
				udi_cdc_putc(FPS);
				usb_state=usb_state_idle;
				break;
				case cmd_set_read:	usb_state=usb_state_set_read;
				break;
				case cmd_set_write:	usb_state=usb_state_set_write;
				break;
				case cmd_set_save:	save_settings();
				usb_state=send_ack(usb_state_idle);
				break;
				default:			usb_state=nack_flush(nack_command);
				break;
			}
			break;
			case usb_state_raw_single:	if(udi_cdc_get_nb_received_data()>=3)
			{
				udi_cdc_read_buf(&back_buffer,3);
				frame_update();
			}
			break;
			case usb_state_raw_multi:	if((udi_cdc_get_nb_received_data()+buffer_pos)>=(set.count*3))
			{
				udi_cdc_read_buf(&back_buffer[buffer_pos],(set.count*3-buffer_pos));
				buffer_pos=0;
				usb_state=send_ack(usb_state_idle);
				frame_update();
			}
			else if(udi_cdc_get_nb_received_data()>=64)
			udi_cdc_read_buf(&back_buffer[buffer_pos],udi_cdc_get_nb_received_data());
			break;
			case usb_state_set_read:	switch(udi_cdc_getc())
			{
				case set_mode:				udi_cdc_putc(set.mode);					break;
				case set_default_mode:		udi_cdc_putc(set.default_mode);			break;
				case set_timeout_mode:		udi_cdc_putc(set.timeout_mode);			break;
				case set_timeout_time:		udi_cdc_putc(set.timeout_time);			break;
				case set_alpha:				udi_cdc_putc(set.alpha);				break;
				case set_default_alpha:		udi_cdc_putc(set.default_alpha);		break;
				case set_gamma:				udi_cdc_putc(set.gamma);				break;
				case set_smooth_time:		udi_cdc_putc(set.smooth_time);			break;
				case set_alpha_min:			udi_cdc_putc(set.alpha_min);			break;
				case set_lux_max:			udi_cdc_putc(set.lux_max);				break;
				case set_stat_Led:			udi_cdc_putc(set.stat_LED);				break;
				case set_stb_Led:			udi_cdc_putc(set.stb_LED);				break;
				case set_count:				udi_cdc_putc(set.count);				break;
				case set_OCP:				udi_cdc_putc(set.OCP);					break;
				case set_OCP_time:			udi_cdc_putc(set.OCP_time);				break;
				case set_SCP:				udi_cdc_putc(set.SCP);					break;
				case set_UVP:				udi_cdc_putc(set.UVP);					break;
				default:					usb_state=nack_flush(nack_read_add);	break;
			}
			break;
			case usb_state_set_write:	if(udi_cdc_get_nb_received_data()>=4)
			{
				udi_cdc_read_buf(&usb_buff,4);
				if((usb_buff[0]==usb_buff[1])&&(usb_buff[2]==usb_buff[3]))
				switch(usb_buff[0])
				{
					case set_mode:				mode_update(usb_buff[2]);					break;
					case set_default_mode:		set.default_mode = usb_buff[2];				break;
					case set_timeout_mode:		set.timeout_mode = usb_buff[2];				break;
					case set_timeout_time:		set.timeout_time = usb_buff[2];				break;
					case set_alpha:				set.alpha = usb_buff[2];					break;
					case set_default_alpha:		set.default_alpha = usb_buff[2];			break;
					case set_gamma:				set.gamma = usb_buff[2]; gamma_calc();		break;
					case set_smooth_time:		set.smooth_time = usb_buff[2];				break;
					case set_alpha_min:			set.alpha_min = usb_buff[2];				break;
					case set_lux_max:			set.lux_max = usb_buff[2];					break;
					case set_stat_Led:			set.stat_LED = usb_buff[2];					break;
					case set_stb_Led:			set.stb_LED = usb_buff[2];					break;
					case set_count:				count_update(usb_buff[2]);					break;
					case set_OCP:				set.OCP = usb_buff[2];						break;
					case set_OCP_time:			set.OCP_time = usb_buff[2];					break;
					case set_SCP:				set.SCP = usb_buff[2];						break;
					case set_UVP:				set.UVP = usb_buff[2];						break;
					default:					usb_state=nack_flush(nack_write_add);		break;
				}
				else
				usb_state=nack_flush(nack_write_crc);
			}
			break;
		}
	}
	
	static uint_fast32_t usb_rx_time=0;
	static uint_fast8_t usb_state_prev=0;
	uint_fast32_t time=rtc_get_time();
	
	if(usb_state_prev!=usb_state)
	{
		usb_rx_time=time;
		usb_state_prev=usb_state;
	}
	
	if((usb_state!=usb_state_idle)	&& (time>=(usb_rx_time+0.2*RTC_freq)))
		usb_state=nack_flush(nack_timeout|usb_state);
	if(set.timeout_time)
	{
		if((set.mode&state_usb)		&& (time>=(usb_rx_time+set.timeout_time*RTC_freq/10)))
			mode_update(set.timeout_mode);
	}
	if(set.mode==mode_usb_ada)
		udi_cdc_write_buf(&ack_ada,sizeof(ack_ada));	//UNDONE: ada - send ping, disable timeout and gamma
}

static uint_fast8_t send_ack(uint_fast8_t new_mode)
{
	udi_cdc_putc(ack);
	return new_mode;
}

static uint_fast8_t nack_flush(uint_fast8_t fault_code)
{
	udi_cdc_putc(nack_preamble);
	while(udi_cdc_is_rx_ready())
	udi_cdc_getc();
	return usb_state_idle;
}

static Bool string_parser(uint8_t buff_a[], const uint8_t buff_b[], uint_fast8_t length)
{
	uint_fast8_t index;
	for(index=0;index<length;index++)
	{
		if(buff_a[index]!=buff_b[index])
		return false;
	}
	return true;
}

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
		if(!set.timeout_time)
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

enum usb_state_t {
	USB_STATE_IDLE			=	0,
	USB_STATE_CMD			=	1,
	USB_STATE_RAW_SINGLE	=	2,
	USB_STATE_RAW_MULTI		=	3,
	USB_STATE_SET_READ		=	4,
	USB_STATE_SET_WRITE		=	5,
	USB_STATE_ADA_HEADER	=	6,
	USB_STATE_ADA_RAW		=	7
};

void handle_usb(void)
{
	//finite state machine
	static enum usb_state_t usb_state=USB_STATE_IDLE;
	uint8_t usb_buff[4];
	static uint_fast16_t buffer_pos=0;
	iram_size_t rx_count=udi_cdc_get_nb_received_data();
	
	if (rx_count)
	{
		switch(usb_state)
		{
			case USB_STATE_IDLE:		if(rx_count>=3)
										{
											udi_cdc_read_buf(&usb_buff,3);
											if		(string_parser(usb_buff, preamble, sizeof(preamble)))
												usb_state=USB_STATE_CMD;
											else if	(string_parser(usb_buff, pre_ada, sizeof(pre_ada)))
											{
												udi_cdc_write_buf(&ack_ada,sizeof(ack_ada));
												usb_state=USB_STATE_ADA_HEADER;
											}
											else
												usb_state=nack_flush(NACK_PREAMPLE);
										}
										break;
			case USB_STATE_ADA_HEADER:	if(rx_count>=3)
										{
											udi_cdc_read_buf(&usb_buff,3);
											if((usb_buff[0]^usb_buff[1]^0x55)==usb_buff[2])
											{
												if(!usb_buff[0] && (usb_buff[1]<=BUFFER_SIZE))
												{
													write_count(usb_buff[1]);
													mode_update(MODE_USB_ADA);
													usb_state=USB_STATE_ADA_RAW;
												}
												else
													usb_state=nack_flush(NACK_ADA_LENGTH);
											}
											else
												usb_state=nack_flush(NACK_ADA_CRC);
										}
										break;
			case USB_STATE_ADA_RAW:		if((rx_count+buffer_pos)>=(set.count*3))	//BUG: ada loosing connection
										{
											udi_cdc_read_buf(&back_buffer[buffer_pos],(set.count*3-buffer_pos));	//TODO: move ada to front_buffer and skip gamma
											buffer_pos=0;
											frame_update();
											usb_state=USB_STATE_IDLE;
										}
										else if(rx_count>=32)
										{
											udi_cdc_read_buf(&back_buffer[buffer_pos],rx_count);
											buffer_pos+=rx_count;
										}
										break;
			case USB_STATE_CMD:			switch(udi_cdc_getc())
										{
											case CMD_TEST:		udi_cdc_write_buf(&response,sizeof(response));
																udi_cdc_putc(PROTOCOL_REV);
																udi_cdc_putc(SOFTWARE_REV);
																udi_cdc_putc(BOARD_REV);
																udi_cdc_putc(set.mode);
																usb_state=USB_STATE_IDLE;
																break;
											case CMD_RAW_DATA:	if(set.mode==MODE_USB_SINGLE)
																	usb_state=send_ack(USB_STATE_RAW_SINGLE);
																else if(set.mode==MODE_USB_MULTI)
																	usb_state=send_ack(USB_STATE_RAW_MULTI);
																else
																	usb_state=nack_flush(NACK_RAW_MODE);
																break;
											case CMD_MEASURE:	udi_cdc_write_buf((adc_sample*)&measure,sizeof(measure));
																udi_cdc_putc(FPS>>8);
																udi_cdc_putc(FPS);
																usb_state=USB_STATE_IDLE;
																break;
											case CMD_SET_READ:	usb_state=USB_STATE_SET_READ;
																break;
											case CMD_SET_WRITE:	usb_state=USB_STATE_SET_WRITE;
																break;
											case CMD_SET_SAVE:	save_settings();
																usb_state=send_ack(USB_STATE_IDLE);
																break;
											default:			usb_state=nack_flush(NACK_COMMAND);
																break;
										}
										break;
			case USB_STATE_RAW_SINGLE:	if(rx_count>=3)
										{
											usb_state=send_ack(USB_STATE_IDLE);
											udi_cdc_read_buf(&back_buffer,3);
											frame_update();
										}
										break;
			case USB_STATE_RAW_MULTI:	if((rx_count+buffer_pos)>=(set.count*3))
										{
											usb_state=send_ack(USB_STATE_IDLE);
											udi_cdc_read_buf(&back_buffer[buffer_pos],(set.count*3-buffer_pos));
											buffer_pos=0;
											frame_update();
										}
										else if(rx_count>=32)
										{
											udi_cdc_read_buf(&back_buffer[buffer_pos],rx_count);
											buffer_pos+=rx_count;
										}
										break;
			case USB_STATE_SET_READ:	if(read_set(udi_cdc_getc(), &usb_buff[0]))
											udi_cdc_putc(usb_buff[0]);
										else
											usb_state=nack_flush(NACK_READ_ADD);
										break;
			case USB_STATE_SET_WRITE:	if(udi_cdc_get_nb_received_data()>=4)
										{
											udi_cdc_read_buf(&usb_buff,4);
											if((usb_buff[0]==usb_buff[1])&&(usb_buff[2]==usb_buff[3]))
											{
												if (write_set(usb_buff[0], usb_buff[2]))
													usb_state=send_ack(USB_STATE_IDLE);
												else
													usb_state=nack_flush(NACK_WRITE_ADD);
											}
											else
												usb_state=nack_flush(NACK_WRITE_CRC);
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
	
	if((usb_state!=USB_STATE_IDLE)	&& (time>=(usb_rx_time+0.5*RTC_FREQ)))
	{
		usb_state=nack_flush(NACK_TIMEOUT|usb_state);
		buffer_pos=0;
	}
	if(set.timeout_time&&(set.timeout_time!=0xFF))
	{
		if((set.mode&STATE_USB)		&& (time>=(usb_rx_time+set.timeout_time*RTC_FREQ/10)))
			mode_update(set.timeout_mode);
	}
}

static uint_fast8_t send_ack(uint_fast8_t new_mode)
{
	udi_cdc_putc(ACK);
	return new_mode;
}

static uint_fast8_t nack_flush(uint_fast8_t fault_code)
{
	udi_cdc_putc(NACK_PREAMPLE);
	while(udi_cdc_is_rx_ready())
	udi_cdc_getc();
	return USB_STATE_IDLE;
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
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
#include "misc_adc.h"
#include "usb.h"


static void ack_idle(void);
static void nack_flush(uint_fast8_t fault_code);
static void usb_update(uint_fast8_t state);
static Bool string_parser(uint8_t buff_a[], const uint8_t buff_b[], uint_fast8_t length);

//////////////////////////////////////////////////////////////////////////
/* USB */

/* Init VBus detection io */
void Vbus_init(void)
{
	ioport_set_pin_dir(USB_VBUS,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(USB_VBUS,IOPORT_MODE_TOTEM);
	ioport_set_pin_sense_mode(USB_VBUS,IOPORT_SENSE_BOTHEDGES);
	VBus_INTMSK = VBus_Pin_bm;
}

//VBus detection
ISR (Vbus_INT_vect)
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
		PORTD_INTCTRL = 1;
		if(ioport_get_pin_level(USB_VBUS))
			udc_attach();
		else
			udc_detach();
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

//TODO: make protocol handler, semaphore
enum usb_state_t usb_state=USB_STATE_IDLE;
uint_fast8_t rx_lim=3;
uint_fast16_t buffer_pos=0;
uint_fast32_t usb_rx_time=0;

void handle_usb(void)
{
	//finite state machine
	while(udi_cdc_get_nb_received_data()>=rx_lim)
	{
	uint8_t usb_buff[6];
	usb_rx_time=rtc_get_time();
	
		switch(usb_state)
		{
			case USB_STATE_IDLE:		udi_cdc_read_buf(&usb_buff,3);
										if		(string_parser(usb_buff, preamble, sizeof(preamble)))
											usb_update(USB_STATE_CMD);
										else if	(string_parser(usb_buff, pre_ada, sizeof(pre_ada)))
										{
											udi_cdc_write_buf(&ack_ada,sizeof(ack_ada));
											usb_update(USB_STATE_ADA_HEADER);
										}
										else
											nack_flush(NACK_PREAMPLE);
										break;
			case USB_STATE_ADA_HEADER:	udi_cdc_read_buf(&usb_buff,3);
										if((usb_buff[0]^usb_buff[1]^0x55)==usb_buff[2])
										{
											if(!usb_buff[0] && (usb_buff[1]<=BUFFER_SIZE))
											{
												mode_update(MODE_USB_ADA);
												write_count(usb_buff[1] + 1);
												usb_update(USB_STATE_ADA_RAW);
											}
											else
												nack_flush(NACK_ADA_LENGTH);
										}
										else
											nack_flush(NACK_ADA_CRC);
										break;
			case USB_STATE_ADA_RAW:		udi_cdc_read_buf(&back_buffer[buffer_pos],rx_lim);
										if((buffer_pos+=rx_lim)==set.count*3)
										{
											buffer_pos=0;
											usb_update(USB_STATE_IDLE);
											frame_update();
										}
										else
											usb_update(USB_STATE_ADA_RAW);
										break;
			case USB_STATE_CMD:			switch(udi_cdc_getc())
										{
											case CMD_TEST:		ack_idle();
																udi_cdc_write_buf(&response,sizeof(response));
																usb_buff[0]=PROTOCOL_REV;
																usb_buff[1]=SOFTWARE_REV;
																usb_buff[2]=BOARD_REV;
																usb_buff[3]=set.mode;
																udi_cdc_write_buf(usb_buff,4);
																break;
											case CMD_RAW_DATA:	if(set.mode==MODE_USB_SINGLE)
																{
																	udi_cdc_putc(ACK);
																	usb_update(USB_STATE_RAW_SINGLE);
																}
																else if(set.mode==MODE_USB_MULTI)
																{
																	udi_cdc_putc(ACK);
																	usb_update(USB_STATE_RAW_MULTI);
																}
																else
																	nack_flush(NACK_RAW_MODE);
																break;
											case CMD_MEASURE:	ack_idle();
																udi_cdc_write_buf((adc_sample*)&measure,sizeof(measure));
																usb_buff[0]=FPS;
																usb_buff[1]=FPS>>8;
																uint_fast16_t u_min, i_max;
																get_max_reset(&u_min,&i_max);
																usb_buff[2]=u_min;
																usb_buff[3]=u_min>>8;
																usb_buff[4]=i_max;
																usb_buff[5]=i_max>>8;
																udi_cdc_write_buf(usb_buff,6);
																break;
											case CMD_SET_READ:	usb_update(USB_STATE_SET_READ);
																break;
											case CMD_SET_WRITE:	usb_update(USB_STATE_SET_WRITE);
																break;
											case CMD_SET_SAVE:	ack_idle();
																save_settings();
																break;
											default:			nack_flush(NACK_COMMAND);
																break;
										}
										break;
			case USB_STATE_RAW_SINGLE:	ack_idle();
										udi_cdc_read_buf(&back_buffer,3);
										frame_update();
										break;
			case USB_STATE_RAW_MULTI:	udi_cdc_read_buf(&back_buffer[buffer_pos],rx_lim);
										if((buffer_pos+=rx_lim)==set.count*3)
										{										
											ack_idle();
											frame_update();
										}
										else
											usb_update(USB_STATE_ADA_RAW);
										break;
			case USB_STATE_SET_READ:	usb_buff[0]=udi_cdc_getc();
										if(usb_buff[0]==SET_READ_ALL)
										{
											ack_idle();
											udi_cdc_write_buf(&set,sizeof(set));
										}
										else
										{
											if(read_set(usb_buff[0], &usb_buff[1]))
											{
												ack_idle();
												udi_cdc_write_buf(usb_buff,2);
											}
											else
												nack_flush(NACK_READ_ADD);
										}
										break;
			case USB_STATE_SET_WRITE:	udi_cdc_read_buf(usb_buff,2);
										if (write_set(usb_buff[0], usb_buff[1]))
										{
											ack_idle();
											udi_cdc_write_buf(usb_buff,2);
										}
										else
											nack_flush(NACK_WRITE_ADD);
										break;
		}
	}
}

void rtc_usb(uint32_t time)
{
	if((usb_state!=USB_STATE_IDLE)	&& (time>=(usb_rx_time+0.2*RTC_FREQ)))
		nack_flush(NACK_TIMEOUT|usb_state);
	if(set.timeout_time)
	{
		if((set.mode&STATE_USB)		&& (time>=(usb_rx_time+set.timeout_time*RTC_FREQ/10)))
		{
			nack_flush(NACK_TIMEOUT);
			mode_update(set.timeout_mode);
		}
	}
}

static void ack_idle(void)
{
	udi_cdc_putc(ACK);
	usb_update(USB_STATE_IDLE);
}

static void nack_flush(uint_fast8_t fault_code)
{
	if(set.mode!=MODE_USB_ADA)
		udi_cdc_putc(fault_code);
	while(udi_cdc_is_rx_ready())
		udi_cdc_getc();
	usb_update(USB_STATE_IDLE);
}

static void usb_update(uint_fast8_t state)
{
	const Byte read_seg=32;
	switch(state)
	{
		case USB_STATE_IDLE:		rx_lim=3;	
									buffer_pos=0; 
									break;
		case USB_STATE_CMD:			rx_lim=1;	
									break;
		case USB_STATE_RAW_SINGLE:	rx_lim=3;	
									break;
		case USB_STATE_RAW_MULTI:	rx_lim=set.count;/*if(buffer_pos+read_seg<=(set.count*3))
										rx_lim=read_seg;
									else
										rx_lim=set.count*3-buffer_pos;*/
									break;
		case USB_STATE_SET_READ:	rx_lim=1;	
									break;
		case USB_STATE_SET_WRITE:	rx_lim=2;	
									break;
		case USB_STATE_ADA_HEADER:	rx_lim=3;	
									break;
		case USB_STATE_ADA_RAW:		rx_lim=set.count;/*if(buffer_pos+read_seg<=(set.count*3))
										rx_lim=read_seg;
									else
										rx_lim=set.count*3-buffer_pos;*/
									break;
	}
	usb_state=state;
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
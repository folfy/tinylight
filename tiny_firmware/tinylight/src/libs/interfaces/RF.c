/*
 * RF.c
 *
 * Created: 23.05.2016 18:41:45
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "libs/protocol/tiny_protocol.h"
#include "libs/interfaces/led.h"
#include "libs/modules/settings_sled.h"
#include "libs/modules/sleep_adc.h"
#include "libs/interfaces/nrf24/fifo.h"
#include "libs/interfaces/nrf24/nrf24.h"
#include "RF.h"

static void nack_flush(uint_fast8_t fault_code);
static void usb_update(uint_fast8_t state);
static Bool string_parser(uint8_t buff_a[], const uint8_t buff_b[], uint_fast8_t length);

//////////////////////////////////////////////////////////////////////////
/* RF */

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

uint8_t buffer[64];
FIFO_BUFFER fifo;

static enum usb_state_t usb_state=USB_STATE_IDLE;
static uint_fast8_t rx_lim=3;
static uint_fast16_t buffer_pos=0;
static uint_fast32_t usb_rx_time=0;

// init RF USART
void rf_init(void)
{
	FIFO_Init(&fifo,buffer,sizeof(buffer));
	nrf24_init(0, 16, 0xE7E7E700, 0xE7E7E70F);
	nrf24_mode_rx();
}

void handle_rf(void)
{
	if(nrf24_is_tx()) {
		// wait till data has been sent
		uint8_t nrf_status=nrf24_status();
		if (nrf_status & (NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS)) {
			// check status, clear flash, and flush tx FIFO in case of an transmission error
			if (nrf_status & NRF_STATUS_TX_DS) {
				nrf24_reg_write(NRF_REG_STATUS, NRF_STATUS_TX_DS);
			} else if (nrf_status & NRF_STATUS_MAX_RT) {
				nrf24_flush_tx();
				nrf24_reg_write(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
			}
			nrf24_mode_rx();
		}
	}
	while(nrf24_data_avail()) {
		while(FIFO_Free(&fifo)>=32) {
			uint8_t rx_data[32], count;
			count=nrf24_data_get(rx_data);
			FIFO_Putn(&fifo,rx_data,count);
		}
		//finite state machine
		while(FIFO_Count(&fifo)>=rx_lim)
		{
			uint8_t usb_buff[16], i;
			usb_rx_time=rtc_get_time();
	
			switch(usb_state)
			{
				case USB_STATE_IDLE:		FIFO_Getn(&fifo, usb_buff, 3);
											if		(string_parser(usb_buff, preamble, sizeof(preamble)))
												usb_update(USB_STATE_CMD);
											else if	(string_parser(usb_buff, pre_ada, sizeof(pre_ada)))
											{
												nrf24_data_send((uint8_t*)ack_ada, sizeof(ack_ada));
												usb_update(USB_STATE_ADA_HEADER);
											}
											else
												nack_flush(NACK_PREAMPLE);
											break;
				case USB_STATE_ADA_HEADER:	FIFO_Getn(&fifo, usb_buff, 3);
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
				case USB_STATE_ADA_RAW:		FIFO_Getn(&fifo, &back_buffer[buffer_pos], rx_lim);
											if((buffer_pos+=rx_lim)==set.count*3)
											{
												buffer_pos=0;
												usb_update(USB_STATE_IDLE);
												frame_update();
											}
											else
												usb_update(USB_STATE_ADA_RAW);
											break;
				case USB_STATE_CMD:			switch(FIFO_Get(&fifo))
											{
												case CMD_TEST:		usb_update(USB_STATE_IDLE);
																	usb_buff[0]=ACK;
																	for(i=0; i<sizeof(response);i++) {
																		usb_buff[i]=response[i];
																	}
																	usb_buff[i++]=PROTOCOL_REV;
																	usb_buff[i++]=SOFTWARE_REV;
																	usb_buff[i++]=BOARD_REV;
																	usb_buff[i++]=set.mode;
																	nrf24_data_send(usb_buff, i);
																	break;
												case CMD_RAW_DATA:	if(set.mode==MODE_USB_SINGLE)
																	{
																		usb_buff[0]=ACK;
																		nrf24_data_send(usb_buff, 1);
																		usb_update(USB_STATE_RAW_SINGLE);
																	}
																	else if(set.mode==MODE_USB_MULTI)
																	{
																		usb_buff[0]=ACK;
																		nrf24_data_send(usb_buff, 1);
																		usb_update(USB_STATE_RAW_MULTI);
																	}
																	else
																		nack_flush(NACK_RAW_MODE);
																	break;
												case CMD_MEASURE:	usb_update(USB_STATE_IDLE);
																	usb_buff[0]=ACK;
																	uint8_t *bytePtr = (uint8_t*)&measure;
																	for(i=1; i<sizeof(measure);i++) {
																		usb_buff[i]=bytePtr[i];
																	}
																	usb_buff[i++]=FPS;
																	usb_buff[i++]=FPS>>8;
																	uint_fast16_t u_min, i_max;
																	get_max_reset(&u_min,&i_max);
																	usb_buff[i++]=u_min;
																	usb_buff[i++]=u_min>>8;
																	usb_buff[i++]=i_max;
																	usb_buff[i++]=i_max>>8;
																	nrf24_data_send(usb_buff, i);
																	break;
												case CMD_SET_READ:	usb_update(USB_STATE_SET_READ);
																	break;
												case CMD_SET_WRITE:	usb_update(USB_STATE_SET_WRITE);
																	break;
												case CMD_SET_SAVE:	usb_update(USB_STATE_IDLE);
																	usb_buff[0]=ACK;
																	nrf24_data_send(usb_buff, 1);
																	save_settings();
																	break;
												default:			nack_flush(NACK_COMMAND);
																	break;
											}
											break;
				case USB_STATE_RAW_SINGLE:	usb_update(USB_STATE_IDLE);
											usb_buff[0]=ACK;
											nrf24_data_send(usb_buff, 1);
											FIFO_Getn(&fifo, back_buffer,3);
											frame_update();
											break;
				case USB_STATE_RAW_MULTI:	FIFO_Getn(&fifo, &back_buffer[buffer_pos], rx_lim);
											if((buffer_pos+=rx_lim)==set.count*3)
											{										
												usb_update(USB_STATE_IDLE);
												usb_buff[0]=ACK;
												nrf24_data_send(usb_buff, 1);
												frame_update();
											}
											else
												usb_update(USB_STATE_ADA_RAW);
											break;
				case USB_STATE_SET_READ:	usb_buff[1]=FIFO_Get(&fifo);
											if(usb_buff[1]==SET_READ_ALL)
											{
												usb_update(USB_STATE_IDLE);
												usb_buff[0]=ACK;
												uint8_t *bytePtr = (uint8_t*)&set;
												for(i=1; i<sizeof(set);i++) {
													usb_buff[i]=bytePtr[i];
												}
												nrf24_data_send(usb_buff, i);
											}
											else
											{
												if(read_set(usb_buff[1], &usb_buff[2]))
												{
													usb_update(USB_STATE_IDLE);
													usb_buff[0]=ACK;
													nrf24_data_send(usb_buff, 3);
												}
												else
													nack_flush(NACK_READ_ADD);
											}
											break;
				case USB_STATE_SET_WRITE:	FIFO_Getn(&fifo, &usb_buff[1], 2);
											if (write_set(usb_buff[1], usb_buff[2]))
											{
												usb_update(USB_STATE_IDLE);
												usb_buff[0]=ACK;
												nrf24_data_send(usb_buff, 3);
											}
											else
												nack_flush(NACK_WRITE_ADD);
											break;
			}
		}
	}
}

void rtc_rf(uint32_t time)
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

static void nack_flush(uint_fast8_t fault_code)
{
	if(set.mode!=MODE_USB_ADA)
		nrf24_data_send(&fault_code, 1);
	while(!FIFO_Empty(&fifo)) {
		FIFO_Get(&fifo);
	}
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
		case USB_STATE_RAW_MULTI:	if(buffer_pos+read_seg<=(set.count*3))
										rx_lim=read_seg;
									else
										rx_lim=set.count*3-buffer_pos;
									break;
		case USB_STATE_SET_READ:	rx_lim=1;	
									break;
		case USB_STATE_SET_WRITE:	rx_lim=2;	
									break;
		case USB_STATE_ADA_HEADER:	rx_lim=3;	
									break;
		case USB_STATE_ADA_RAW:		if(buffer_pos+read_seg<=(set.count*3))
										rx_lim=read_seg;
									else
										rx_lim=set.count*3-buffer_pos;
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
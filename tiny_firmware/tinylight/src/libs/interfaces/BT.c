/*
 * BT.c
 *
 * Created: 29.04.2014 19:04:40
 *  Author: Folfy
 */ 


#include <asf.h>
#include "BT.h"

#if	RF_avail==2
	
	uint_fast8_t BT_rx_buffer[BT_RX_BUFFER_SIZE];
	uint_fast8_t BT_rx_buffer_pos_write = 0;
	uint_fast8_t BT_rx_buffer_pos_read = 0;
	bool BT_rx_buffer_overrun = false;
	
void BT_init(void)
{
	ioport_set_pin_dir(BT_EN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(BT_EN,HIGH);
	ioport_set_pin_dir(BT_RX,IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BT_TX,IOPORT_DIR_OUTPUT);
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = 9600,
		.charlength = USART_CHSIZE_8BIT_gc,
		.paritytype = USART_PMODE_DISABLED_gc,
		.stopbits = false
	};
	usart_init_rs232(BT_USART, &USART_SERIAL_OPTIONS);
	usart_set_rx_interrupt_level(BT_USART, BT_USART_INT_LVL);
};

ISR(BT_USART_RX_vect)
{
	BT_rx_buffer_pos_write++;
	if(BT_rx_buffer_pos_write >= BT_RX_BUFFER_SIZE)
	{
		BT_rx_buffer_overrun = true;
		BT_rx_buffer_pos_write = 0;
	}
	if(BT_rx_buffer_overrun & (BT_rx_buffer_pos_read <= BT_rx_buffer_pos_write))
	{
		return;		//OVERRUN
	}
	BT_rx_buffer[BT_rx_buffer_pos_write] = usart_getchar(BT_USART);
};

uint_fast8_t BT_getc()
{
	while(BT_get_nb_received_data()==0);
	BT_rx_buffer_pos_read++;
	if(BT_rx_buffer_pos_read >= BT_RX_BUFFER_SIZE)
	{
		BT_rx_buffer_overrun = false;
		BT_rx_buffer_pos_read = 0;
	}
	return BT_rx_buffer[BT_rx_buffer_pos_read++];
};

void BT_read_buf(uint_fast8_t buf[], uint_fast8_t size)
{
	for (uint_fast8_t i=0;i<size;i++)
	{
		buf[i] = BT_getc();
	}
};

uint_fast8_t BT_get_nb_received_data()
{
	if(BT_rx_buffer_overrun)	return BT_rx_buffer_pos_write + (BT_RX_BUFFER_SIZE - BT_rx_buffer_pos_read - 1);
	else					return BT_rx_buffer_pos_write - BT_rx_buffer_pos_read;
};

void BT_putc(int value)
{
	usart_putchar(BT_USART,value);
};

void BT_write_buf(uint_fast8_t buf[], uint_fast8_t size)
{
	for (uint_fast8_t i=0;i<size;i++)
	{
		usart_putchar(BT_USART, buf[i]);
	}
};

#endif
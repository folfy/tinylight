/*
 * BT.c
 *
 * Created: 29.04.2014 19:04:40
 *  Author: Folfy
 */ 


#if		RF_avail==2
void BT_init(void)
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
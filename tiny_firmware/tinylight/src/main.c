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
	#include "IR.h"
	//extern void IR_init(void);
#endif

#if	RF_avail==2
	static void BT_init(void);
#endif

//Button, Status_LED, FPS count
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
	cpu_irq_enable();
	usb_init();
	
	#if	RF_avail==2
	BT_init();
	#endif
	
	while(1)
	{
		if(set.mode==MODE_OFF)
			power_down();

		uint32_t time=rtc_get_time();
			
		handle_usb();
		handle_auto_modes();
			
		//if((rtc_get_time()>=time+0.01*RTC_FREQ)&&set.mode!=MODE_OFF)
		//UNDONE: implement new error handling 
		//TODO: add rms calc -> main
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
	rtc_usb(time);
	
	if(time > (UINT32_MAX-RTC_FREQ*3600*24))	//prevent rtc overflow if there are less than 24h remaining
	{
		if( !(set.mode&STATE_ON) || (time >= (UINT32_MAX-alarm_cycle)) )	// delay as long as required or possible
			reset_do_soft_reset();	// TODO: Review RTC_Overflow
	}
	rtc_set_alarm(alarm_prev+=alarm_cycle);
}

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
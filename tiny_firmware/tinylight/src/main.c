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
 *
 *  EVCH0:  ADC
 */

#include <stdio.h>
#include <asf.h>
#include "libs/protocol/tiny_protocol.h"
#include "libs/interfaces/led.h"
#include "libs/interfaces/usb.h"
#include "libs/interfaces/IR.h"
#include "libs/modules/settings_sled.h"
#include "libs/modules/sleep_adc.h"

#if	RF_avail==2
	#include "BT.h"
#endif

//Button, Status_LED, FPS count
static void RTC_Alarm(uint32_t time);

int main (void)
{
	#if RF_avail==3
	PORTCFG_VPCTRLA=PORTCFG_VP02MAP_PORTC_gc;
	VPORT0_DIR|=PIN0_bm|PIN2_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm;
	#endif
	/* Init system functions (clock, sleepmgr) */
	sysclk_init();
	sleepmgr_init();
	
	/* Init mosfet_io */
	ioport_init();
	ioport_set_pin_dir(MOSFET_en, IOPORT_DIR_OUTPUT);

	sled_init();	// Init status LED
	Vbus_init();	// Init VBus detection
	ioport_set_pin_mode(BUTTON,IOPORT_MODE_PULLUP|IOPORT_MODE_INVERT_PIN);	// Init button IO
	rtc_init();		// Init RTC
	pmic_init();	// Init interrupt controller
	
	/* led init requires settings to be initialized (dma_multi-flag) */
	read_settings();
	led_init();		// Init LED strip USART as SPI Master and led latch delay timer


	rtc_set_callback(RTC_Alarm);
	rtc_set_alarm_relative(0);
	adc_init();
	cpu_irq_enable();
	usb_init();

	IR_init();
	#if	RF_avail==2
		BT_init();
	#endif
	
	while(1)
	{
		if(set.mode==MODE_SLEEP)
			power_down();
			
		#ifdef DEBUG
		uint32_t time=rtc_get_time();
		#endif
			
		handle_usb();
		handle_auto_modes();
		
		#ifdef DEBUG
		if((rtc_get_time()>=time+0.01*RTC_FREQ)&&set.mode!=MODE_SLEEP)
		;//UNDONE: implement new error handling 
		#endif
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
		{
			save_settings();
			reset_do_soft_reset();	// TODO: RTC_Overflow - time shift
		}
	}
	rtc_set_alarm(alarm_prev+=alarm_cycle);
}
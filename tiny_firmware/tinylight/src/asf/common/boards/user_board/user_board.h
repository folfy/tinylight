/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

//#define IR_avail

#include <conf_board.h>

#define	Sens_Light		IOPORT_CREATE_PIN(PORTA,1)
//#define Sens_Light_en IOPORT_CREATE_PIN(PORTA,3) //init -enable
//#define Sens_Light	IOPORT_CREATE_PIN(PORTA,4) //init -ADC_ch
#define MOSFET_en		IOPORT_CREATE_PIN(PORTA,5)
#define Sens_I_P		IOPORT_CREATE_PIN(PORTA,6)
#define Sens_I_N		IOPORT_CREATE_PIN(PORTA,7)
//#define MISC_JP2_P3		IOPORT_CREATE_PIN(PORTB,0)
//#define MISC_JP2_P2		IOPORT_CREATE_PIN(PORTB,1)
//#define MISC_JP2_P1		IOPORT_CREATE_PIN(PORTB,2)
#ifdef IR_avail
#define IR_en			IOPORT_CREATE_PIN(PORTB,1)
#define IR_in			IOPORT_CREATE_PIN(PORTB,2)
#endif
#define Sens_Uin		IOPORT_CREATE_PIN(PORTB,3)
#define	LED_CLK			IOPORT_CREATE_PIN(PORTC,1)
#define	LED_TX			IOPORT_CREATE_PIN(PORTC,3)
#define MISC_JP3_P5		IOPORT_CREATE_PIN(PORTC,5)
#define MISC_JP3_P4		IOPORT_CREATE_PIN(PORTC,6)
#define MISC_JP3_P3		IOPORT_CREATE_PIN(PORTC,7)
#define SLED_B			IOPORT_CREATE_PIN(PORTD,0)	//TCD0_CCA
#define SLED_G			IOPORT_CREATE_PIN(PORTD,1)	//TCD0_CCB
#define SLED_R			IOPORT_CREATE_PIN(PORTD,2)	//TCD0_CCC
#define USB_VBUS		IOPORT_CREATE_PIN(PORTD,3)
#define USB_D_N			IOPORT_CREATE_PIN(PORTD,6)
#define USB_D_P			IOPORT_CREATE_PIN(PORTD,7)
#define BUTTON			IOPORT_CREATE_PIN(PORTE,3)

#define DMA_CHANNEL_LED 0

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000


#endif // USER_BOARD_H

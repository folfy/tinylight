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

#include <conf_board.h>

#define BOARD_REV 0x10			//rev 0x01 = debug board
/*	uncomment definition to enable additional hardware modules	*/
//#define IR_avail				//IR-Receiver
#define RF_avail 1  			//RF-Transceiver
//#define RF_avail 2  			//BT-Transceiver
//#define RF_avail 3  			//Debug Output (remove RF module first!), 
//#define UART_avail			//UART-Port
#define DEBUG					//enables reset to bootloader via button

#define SOFTWARE_REV			0x03
#define RTC_TIME				0.02	//20ms
#define BUFFER_SIZE				160

//////////////////////////////////////////////////////////////////////////
/* Internal Hardware */

#ifndef BOARD_REV
#error "BOARD_REV undefined"
#endif

#define DMA_CHANNEL_LED			0
#define SLED_TIMER				TCD0
#define SPI_TIMER				TCD1
#ifdef IR_avail
#define IR_TIMER				TCC0
#endif
#define ADC						ADCA
#define ADC_EVCH				0
#define ADC_EVCH_MUX			EVSYS_CH0MUX

//////////////////////////////////////////////////////////////////////////
/* Sens_Light */

#if		BOARD_REV == 0x10
#define Sens_Light_en			IOPORT_CREATE_PIN(PORTA,3)
#define Sens_Light				IOPORT_CREATE_PIN(PORTA,4)
#define ADCCH_POS_PIN_Light		ADCCH_POS_PIN4
#elif	BOARD_REV == 0x01
#define	Sens_Light				IOPORT_CREATE_PIN(PORTA,1)
#endif

//////////////////////////////////////////////////////////////////////////
/* MOSFET, Sens_I */

#define MOSFET_en				IOPORT_CREATE_PIN(PORTA,5)

#define Sens_I_P				IOPORT_CREATE_PIN(PORTA,6)
#define ADCCH_POS_PIN_Sense		ADCCH_POS_PIN6
#define Sens_I_N				IOPORT_CREATE_PIN(PORTA,7)
#define ADCCH_NEG_PIN_Sense		ADCCH_NEG_PIN7

//////////////////////////////////////////////////////////////////////////
/* IR, Sens_Uin */

#ifdef IR_avail
#define IR_en					IOPORT_CREATE_PIN(PORTB,2)
#define IR_in					IOPORT_CREATE_PIN(PORTB,1)
#define IR_in_int				PIN1_bm
#endif

#define Sens_Uin				IOPORT_CREATE_PIN(PORTB,3)
#define ADCCH_POS_PIN_Volt		ADCCH_POS_PIN11

//////////////////////////////////////////////////////////////////////////
/* LED_UART */

#define	LED_CLK					IOPORT_CREATE_PIN(PORTC,1)
#define	LED_TX					IOPORT_CREATE_PIN(PORTC,3)

#define LED_USART				USARTC0
#define LED_USART_DATA			USARTC0_DATA
#define LED_USART_DMA_TRIG_DRE	DMA_CH_TRIGSRC_USARTC0_DRE_gc

//////////////////////////////////////////////////////////////////////////
/* RF Module */

#if		RF_avail==1
#define RF_CE					IOPORT_CREATE_PIN(PORTC,0)
#define RF_CSN					IOPORT_CREATE_PIN(PORTC,2)
#define RF_IRQ					IOPORT_CREATE_PIN(PORTC,4)
#define RF_INT_vect				PORTC_INT0_vect
#define RF_INTMSK				PORTC_INT0MASK
#define RF_Pin_bm				PIN4_bm
#define RF_MOSI					IOPORT_CREATE_PIN(PORTC,5)
#define RF_MISO					IOPORT_CREATE_PIN(PORTC,6)
#define RF_SCK					IOPORT_CREATE_PIN(PORTC,7)
#define RF_USART				USARTC1
#define RF_USART_INT_LVL		USART_INT_LVL_LO
#define RF_USART_RXC_vect		USARTC1_RXC_vect
#define RF_USART_DRE_vect		USARTC1_DRE_vect
#define RF_USART_TXC_vect		USARTC1_TXC_vect

#elif	RF_avail==2
#define BT_EN					IOPORT_CREATE_PIN(PORTC,0)
#define BT_RX					IOPORT_CREATE_PIN(PORTC,6)
#define BT_TX					IOPORT_CREATE_PIN(PORTC,7)
#define BT_RX_BUFFER_SIZE		32
#define BT_USART				&USARTC1
#define BT_USART_INT_LVL		USART_INT_LVL_LO
#define BT_USART_RX_vect		USARTC1_RXC_vect

#elif	RF_avail==3
#define set_P3					asm volatile ("sbi 0x11,0");
#define	clr_P3					asm volatile ("cbi 0x11,0");
#define set_P4					asm volatile ("sbi 0x11,2");
#define	clr_P4					asm volatile ("cbi 0x11,2");
#define set_P8					asm volatile ("sbi 0x11,4");
#define	clr_P8					asm volatile ("cbi 0x11,4");
#define set_P6					asm volatile ("sbi 0x11,5");
#define	clr_P6					asm volatile ("cbi 0x11,5");
#define set_P7					asm volatile ("sbi 0x11,6");
#define	clr_P7					asm volatile ("cbi 0x11,6");
#define set_P5					asm volatile ("sbi 0x11,7");
#define	clr_P5					asm volatile ("cbi 0x11,7");
#endif

//////////////////////////////////////////////////////////////////////////
/* SLED */

//		SLED_TIMER
#define SLED_B					IOPORT_CREATE_PIN(PORTD,0)
#define SLED_TC_CC_B			TC_CCA
#define SLED_TC_CCEN_B			TC_CCAEN

#define SLED_G					IOPORT_CREATE_PIN(PORTD,1)
#define SLED_TC_CC_G			TC_CCB
#define SLED_TC_CCEN_G			TC_CCBEN

#define SLED_R					IOPORT_CREATE_PIN(PORTD,2)
#define SLED_TC_CC_R			TC_CCC
#define SLED_TC_CCEN_R			TC_CCCEN

//////////////////////////////////////////////////////////////////////////
/* USB */

#define USB_VBUS				IOPORT_CREATE_PIN(PORTD,3)
#define Vbus_INT_vect			PORTD_INT0_vect
#define VBus_INTMSK				PORTD_INT0MASK
#define VBus_Pin_bm				PIN3_bm

#define USB_D_N					IOPORT_CREATE_PIN(PORTD,6)
#define USB_D_P					IOPORT_CREATE_PIN(PORTD,7)

//////////////////////////////////////////////////////////////////////////
/* Button, UART */

#if		BOARD_REV == 0x10
#define BUTTON					IOPORT_CREATE_PIN(PORTR,0)
#elif	BOARD_REV == 0x01
#define BUTTON					IOPORT_CREATE_PIN(PORTE,3)
#endif

#ifdef UART_avail
#define UART_RX					IOPORT_CREATE_PIN(PORTE,2)
#define UART_TX					IOPORT_CREATE_PIN(PORTE,3)
#endif

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

/**
 * \file
 *
 * \brief Tinylight board initialization
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

static void Vbus_init(void);
static void led_init(void);
static void ADC_init(void);
static void sled_init(void);

void usart_init_spi_pull_up(USART_t *usart, const usart_spi_options_t *opt);

void board_init()
{
	/* Init clocks*/
	sysclk_init();
	
	/* Initialize the sleep manager */
	sleepmgr_init();
	
	/* Init mosfet_io */
	ioport_init();
	ioport_set_pin_dir(MOSFET_en, IOPORT_DIR_OUTPUT);

	sled_init();	// Init status LED	
	Vbus_init();	// Init VBus detection
	led_init();		// Init LED strip USART as SPI Master
	ADC_init();		// Init ADC controller
	ioport_set_pin_mode(BUTTON,IOPORT_MODE_PULLUP|IOPORT_MODE_INVERT_PIN);	// Init button IO
	rtc_init();		// Init RTC
	
	/* Init interrupt controller */
	pmic_init();
	
	/* Init SPI latch delay timer */
	tc_enable(&TCD1);
	tc_set_overflow_interrupt_level(&TCD1,TC_INT_LVL_MED);
	tc_write_period(&TCD1,500);
}

/* Init VBus detection io */
static void Vbus_init(void)
{
	ioport_set_pin_dir(USB_VBUS,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(USB_VBUS,IOPORT_MODE_TOTEM);
	ioport_set_pin_sense_mode(USB_VBUS,IOPORT_SENSE_BOTHEDGES);
	PORTD_INT0MASK = PIN3_bm;
}

/* Init status LED */
static void sled_init(void)
{
	ioport_set_pin_dir(SLED_R, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SLED_G, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SLED_B, IOPORT_DIR_OUTPUT);	
	ioport_set_pin_mode(SLED_R, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_mode(SLED_G, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_mode(SLED_B, IOPORT_MODE_INVERT_PIN);
	
	tc_enable(&TCD0);
	tc_set_wgm(&TCD0, TC_WG_SS);		//Single Slope PWM
	tc_write_period(&TCD0, 0xFF);		//8 Bit PWM
	tc_set_resolution(&TCD0, 32000000); //32 Mhz Clk
	
	tc_write_cc(&TCD0,TC_CCA,255);		//SLED: Cyan
	tc_write_cc(&TCD0,TC_CCB,255);
	tc_enable_cc_channels(&TCD0,TC_CCAEN|TC_CCBEN);
}

/* Init LED strip USART as SPI Master */
static void led_init(void)
{
	static usart_spi_options_t USART_SPI_OPTIONS = {
		.spimode	= USART_CMODE_MSPI_gc,
		.baudrate	= 1000000,		//1Mhz Clock
		.data_order	= 0				//SPI Mode 0
	};
	usart_init_spi_pull_up(&USARTC0,&USART_SPI_OPTIONS);
	ioport_set_pin_mode(LED_TX, IOPORT_MODE_WIREDANDPULL|IOPORT_MODE_SLEW_RATE_LIMIT|IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_dir(LED_TX,IOPORT_DIR_OUTPUT);
}

/* Init ADC controller */
static void ADC_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&ADCA, &adc_conf);
	adcch_read_configuration(&ADCA,ADC_CH0, &adcch_conf);
	
	adc_set_conversion_parameters(&adc_conf,ADC_SIGN_ON,ADC_RES_12, ADC_REF_BANDGAP);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 4, 0);
	adc_set_current_limit(&adc_conf,ADC_CURRENT_LIMIT_HIGH);
	adc_set_gain_impedance_mode(&adc_conf,ADC_GAIN_LOWIMPEDANCE);
	adc_set_clock_rate(&adc_conf,125000);
	adc_enable_internal_input(&adc_conf,ADC_INT_TEMPSENSE|ADC_INT_BANDGAP);
	
	adc_write_configuration(&ADCA,&adc_conf);
	
	adc_set_compare_value(&ADCA,(310*4.0));	//prevent brownout
	
	/* led supply voltage */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN11,ADCCH_NEG_NONE,1);
	adcch_write_configuration(&ADCA,ADC_CH0, &adcch_conf);
	ADCA_CH0_INTCTRL=ADC_CH_INTMODE_BELOW_gc|ADC_CH_INTLVL_HI_gc;
	
	/* led current */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN6,ADCCH_NEG_PIN7,16);
	adcch_write_configuration(&ADCA,ADC_CH1,&adcch_conf);
	
	/* environment brightness */
	adcch_set_input(&adcch_conf,ADCCH_POS_PIN1,ADCCH_NEG_PAD_GND,0);	//Gain 1/2
	adcch_write_configuration(&ADCA,ADC_CH2,&adcch_conf);
	
	/* Internal temp */
	adcch_set_input(&adcch_conf,ADCCH_POS_TEMPSENSE,ADCCH_NEG_NONE,1);
	adcch_enable_interrupt(&adcch_conf);
	adcch_write_configuration(&ADCA,ADC_CH3,&adcch_conf);
}


//asf_usart_init_spi modified sck_pin pin configuration
#  define USART_UCPHA_bm 0x02
#  define USART_DORD_bm 0x04

void usart_init_spi_pull_up(USART_t *usart, const usart_spi_options_t *opt)
{
	ioport_pin_t sck_pin;
	//bool invert_sck;

	sysclk_enable_peripheral_clock(usart);

	usart_rx_disable(usart);

	/* configure Clock polarity using INVEN bit of the correct SCK I/O port **/
	//invert_sck = (opt->spimode == 2) || (opt->spimode == 3);
	//UNUSED(invert_sck);

	#ifdef USARTC0
	if ((uint16_t)usart == (uint16_t)&USARTC0) {
		#  ifdef PORT_USART0_bm
		if (PORTC.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTC, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTC, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTC, 1);
		#  endif
	}
	#endif
	#ifdef USARTC1
	if ((uint16_t)usart == (uint16_t)&USARTC1) {
		sck_pin = IOPORT_CREATE_PIN(PORTC, 5);
	}
	#endif
	#ifdef USARTD0
	if ((uint16_t)usart == (uint16_t)&USARTD0) {
		#  ifdef PORT_USART0_bm
		if (PORTD.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTD, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTD, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTD, 1);
		#  endif
	}
	#endif
	#ifdef USARTD1
	if ((uint16_t)usart == (uint16_t)&USARTD1) {
		sck_pin = IOPORT_CREATE_PIN(PORTD, 5);
	}
	#endif
	#ifdef USARTE0
	if ((uint16_t)usart == (uint16_t)&USARTE0) {
		#  ifdef PORT_USART0_bm
		if(PORTE.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTE, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTE, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTE, 1);
		#  endif
	}
	#endif
	#ifdef USARTE1
	if ((uint16_t)usart == (uint16_t)&USARTE1) {
		sck_pin = IOPORT_CREATE_PIN(PORTE, 5);
	}
	#endif
	#ifdef USARTF0
	if ((uint16_t)usart == (uint16_t)&USARTF0) {
		#  ifdef PORT_USART0_bm
		if(PORTF.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTF, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTF, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTF, 1);
		# endif
	}
	#endif
	#ifdef USARTF1
	if ((uint16_t)usart == (uint16_t)&USARTF1) {
		sck_pin = IOPORT_CREATE_PIN(PORTF, 5);
	}
	#endif

	/* Configure the USART output pin */
	ioport_set_pin_dir(sck_pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(sck_pin, IOPORT_MODE_WIREDANDPULL|IOPORT_MODE_SLEW_RATE_LIMIT|IOPORT_MODE_INVERT_PIN);
	
	//ioport_set_pin_mode(sck_pin,
	//IOPORT_MODE_TOTEM | (invert_sck? IOPORT_MODE_INVERT_PIN : 0));
	ioport_set_pin_level(sck_pin, IOPORT_PIN_LEVEL_HIGH);

	usart_set_mode(usart, USART_CMODE_MSPI_gc);

	if (opt->spimode == 1 || opt->spimode == 3) {
		usart->CTRLC |= USART_UCPHA_bm;
		} else {
		usart->CTRLC &= ~USART_UCPHA_bm;
	}
	if (opt->data_order) {
		(usart)->CTRLC |= USART_DORD_bm;
		} else {
		(usart)->CTRLC &= ~USART_DORD_bm;
	}

	usart_spi_set_baudrate(usart, opt->baudrate, sysclk_get_per_hz());
	usart_tx_enable(usart);
	//usart_rx_enable(usart);
}
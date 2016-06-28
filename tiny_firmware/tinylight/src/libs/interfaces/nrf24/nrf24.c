/*
 * nrf24.cpp
 *
 * nrf24l01+ click driver library for tiva C evaluation board
 *
 *  Pins:
 *  SCK  (OUT)
 *  MOSI (OUT)
 *  MISO (IN)
 *  CE   (OUT)
 *  CSN  (OUT)
 *  INT  (IN)
 *
 *  Created on: Jan 13, 2016
 *      Author: folfy
 */

#include <stdio.h>
#include <asf.h>
#include "libs/modules/settings_sled.h"
#include "nrf24.h"

#if	RF_avail==1

// Global variables
const uint8_t DATARATE      = NRF_RF_SETUP_DR_250KBPS;
const uint8_t CONFIGURATION = NRF_CONF_EN_CRC | NRF_CONF_MASK_RX_DR | NRF_CONF_MASK_TX_DS | NRF_CONF_MASK_MAX_RT;
uint8_t payload_size = 0;
uint8_t nrf_mode = 0; // OFF
volatile uint8_t nrf_status;

// Prototypes
void nrf24_ssiISR(void);
void nrf24ISR(void);

ISR (NRF_SPI_INT_vect) {
	nrf24_ssiISR();
}

/*
 * Initialize gpio, usart and NRF module
 */
void nrf24_init(uint8_t channel, uint8_t payload_len, uint32_t tx_address, uint32_t rx_address)
{
	/* Unfortunately the IRQ-Pin is on SPIs SS-Pin, which must be held low!
	ioport_set_pin_sense_mode(NRF_IRQ, IOPORT_SENSE_FALLING);
	NRF_INTMSK = NRF_IRQ_bm;
	NRF_INTCTRL = NRF_INTLVL;
	*/
	
	ioport_set_pin_dir (NRF_IRQ,  IOPORT_DIR_INPUT);
	ioport_set_pin_mode(NRF_IRQ,  IOPORT_MODE_PULLUP);
	ioport_set_pin_dir (NRF_MISO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(NRF_MISO, IOPORT_MODE_PULLDOWN);	
	ioport_set_pin_dir (NRF_CE,	  IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(NRF_CE,   IOPORT_MODE_TOTEM | IOPORT_MODE_SLEW_RATE_LIMIT);
	ioport_set_pin_dir (NRF_CSN,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(NRF_CSN,  IOPORT_MODE_TOTEM | IOPORT_MODE_SLEW_RATE_LIMIT);
	ioport_set_pin_dir (NRF_MOSI, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(NRF_MOSI, IOPORT_MODE_TOTEM | IOPORT_MODE_SLEW_RATE_LIMIT);
	ioport_set_pin_dir (NRF_SCK,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(NRF_SCK,  IOPORT_MODE_TOTEM | IOPORT_MODE_SLEW_RATE_LIMIT);

	/* Initialize SPI master */
	sysclk_enable_peripheral_clock(&NRF_SPI);
	NRF_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV16_gc;
	NRF_SPI.INTCTRL = SPI_INTLVL_MED_gc;
	
	ioport_set_pin_level(NRF_CE,  IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(NRF_CSN, IOPORT_PIN_LEVEL_HIGH);

	sled_set(0,10,10,true);

	// check presence of module - wait if missing
	while (nrf24_status()==0x00);
	
	sled_update();

	// power up and enable CRC, wait 1.5ms till device is ready
	nrf24_mode_standby();
	delay_us(1500);

	// set channel and basic RF (data rate / power) configuration
	nrf24_set_ch(channel);
	nrf24_reg_write(NRF_REG_RF_SETUP, DATARATE | NRF_RF_SETUP_PWR_30DBU | NRF_RF_SETUP_LNA_HIGH);

	// set address length to 4 bytes and write the rx / tx addresses
	nrf24_reg_write(NRF_REG_ADDR_WIDTH, NRF_ADDR_WIDTH_4);
	nrf24_set_tx_addr(tx_address);
	nrf24_set_rx_addr(rx_address);

	// enable RX address pipes, enable auto ACK, disable dyn. payload length and set static length
	nrf24_reg_write(NRF_REG_EN_RX_ADDR, NRF_EN_RX_ADDR_P0 | NRF_EN_RX_ADDR_P1);
	nrf24_reg_write(NRF_REG_EN_AUTOACK, NRF_EN_AUTOACK_P0 | NRF_EN_AUTOACK_P1);
	nrf24_reg_write(NRF_REG_DYN_PAYLOAD, 1<<1);
	nrf24_reg_write(NRF_REG_FEATURES, NRF_FEATURES_EN_DYN_PAY);
	payload_size = payload_len;
	nrf24_reg_write(NRF_REG_RX_LENGTH0, 0x00);         // auto ACK pipe
	nrf24_reg_write(NRF_REG_RX_LENGTH1, payload_size); // data pipe

	// configure retransmission delay acc. to datarate and payload size; automatic retransmission - 15 retries (max.)
	nrf24_set_retrans(DATARATE, payload_size, 15);
}

/*
 * mode - configuration
 */

void nrf24_conf(uint8_t conf_param);

void nrf24_mode_rx(void)
{
	if(!nrf24_is_rx())
	{
		nrf_mode = NRF_CONF_PWR_UP | NRF_CONF_PRIM_RX;
		nrf24_conf(nrf_mode);
		ioport_set_pin_level(NRF_CE,  IOPORT_PIN_LEVEL_HIGH);
	}
}

void nrf24_mode_tx(void)
{
	if(!nrf24_is_tx())
	{
		nrf_mode = NRF_CONF_PWR_UP;
		nrf24_conf(nrf_mode);
		ioport_set_pin_level(NRF_CE,  IOPORT_PIN_LEVEL_HIGH);
	}
}

void nrf24_mode_standby(void)
{
	nrf_mode = 0;
	nrf24_conf(NRF_CONF_PWR_UP);
}

Bool nrf24_is_rx(void)
{
	return (nrf_mode == (NRF_CONF_PWR_UP | NRF_CONF_PRIM_RX));
}

Bool nrf24_is_tx(void)
{
	return (nrf_mode == (NRF_CONF_PWR_UP));
}

void nrf24_conf(uint8_t conf_param)
{
	ioport_set_pin_level(NRF_CE,  IOPORT_PIN_LEVEL_LOW);
	nrf24_reg_write(NRF_REG_CONF, CONFIGURATION | conf_param);
	// clear flags after mode change
	nrf24_reg_write(NRF_REG_STATUS, NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR | NRF_STATUS_TX_DS);
}

/*
 * status and configuration functions
 */

uint8_t nrf24_status(void)
{
	nrf24_read(NRF_CMD_NOP, 0, 0);
	return nrf_status;
}

void nrf24_set_ch(uint8_t channel)
{
	// ensure operation within ISM-band
	channel%=100;
	nrf24_reg_write(NRF_REG_RF_CHANNEL, channel);
}

void nrf24_set_retrans(uint8_t datarate, uint8_t max_packet_size, uint8_t retrans_count)
{
	// acc. to datasheet
	uint8_t retrans_time;
	if (datarate == NRF_RF_SETUP_DR_2MBPS) {
		if (payload_size <= 15)
			retrans_time=0; // 250µs
		else
			retrans_time=1; // 500µs
	} else if (datarate == NRF_RF_SETUP_DR_1MBPS) {
		if (payload_size <= 5)
			retrans_time=0; // 250µs
		else
			retrans_time=1; // 500µs
	} else { // NRF_RF_SETUP_DR_250KBPS
		retrans_time=(payload_size+7)/8 + 1;
	}
	nrf24_reg_write(NRF_REG_RETRANS, retrans_time<<NRF_RETRANS_DELAY | retrans_count<<NRF_RETRANS_RETRIES);
}

void nrf24_set_tx_addr(uint32_t address)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_write32(NRF_CMD_REG_WRITE|NRF_REG_RX_ADDR0, 1, &address);
    nrf24_write32(NRF_CMD_REG_WRITE|NRF_REG_TX_ADDR,  1, &address);
}

void nrf24_set_rx_addr(uint32_t address)
{
    nrf24_write32(NRF_CMD_REG_WRITE|NRF_REG_RX_ADDR1, 1, &address);
}

inline void nrf24_reg_write(uint8_t reg, uint8_t data)
{
	nrf24_write(NRF_CMD_REG_WRITE | reg, 1, &data);
}

inline uint8_t nrf24_reg_read(uint8_t reg)
{
	uint8_t data;
	nrf24_read(NRF_CMD_REG_READ | reg, 1, &data);
	return data;
}

/*
 * FIFO data access
 */

void nrf24_data_send(uint8_t tx_buffer[], uint8_t size)
{	
	nrf24_mode_tx();
	nrf24_data_put(tx_buffer, size);
}

void nrf24_data_put(uint8_t tx_buffer[], uint8_t size)
{
	// wait if tx buffer is full
	while (!nrf24_space_avail());

	payload_size=size;
	nrf24_write(NRF_CMD_TX_WRITE, payload_size, tx_buffer);
}

uint8_t nrf24_data_get(uint8_t rx_buffer[])
{
	// wait for rx data ready flag
	while (!nrf24_data_avail());
	
	nrf24_read(NRF_CMD_RX_WIDTH, 1, &payload_size);
	nrf24_read(NRF_CMD_RX_READ, payload_size, rx_buffer);
	nrf24_reg_write(NRF_REG_STATUS, NRF_STATUS_RX_DR);
	
	return payload_size;
}

bool nrf24_data_avail(void)
{
	return !(nrf24_reg_read(NRF_REG_FIFO_STATUS) & NRF_FIFO_STATUS_RX_EMPTY);
}

bool nrf24_space_avail(void)
{
	return !(nrf24_status() & NRF_STATUS_TX_FULL);
}

void nrf24_flush_tx(void)
{
	nrf24_write(NRF_CMD_TX_FLUSH, 0, 0);
}

/*
 * low level access functions
 */

volatile bool    nrf_busy=false;
volatile bool    nrf_read=false;

volatile uint8_t nrf_length;
volatile uint8_t nrf_txbuff[32];
volatile uint8_t nrf_rxbuff[32];

inline void nrf24_write32(uint8_t command, uint8_t length, uint32_t data[])
{
	// required byte order is LSByte first - no conversion required since cortex ARM is little-endian
	nrf24_write(command, length*4, (uint8_t*) data);
}

void nrf24_write(uint8_t command, uint8_t length, uint8_t data[])
{
	// what till SSI is ready
	while(nrf_busy);

	nrf_busy=true;
	nrf_length=length;
	nrf_read=false;

	ioport_set_pin_level(NRF_CSN,  IOPORT_PIN_LEVEL_LOW);
	NRF_SPI.DATA=command;

	for (uint8_t i=0; i<length; i++) {
		nrf_txbuff[i]=data[i];
	}
}

void nrf24_read(uint8_t command, uint8_t length, uint8_t data[])
{
	// what till SSI is ready
	while(nrf_busy);

	nrf_busy=true;
	nrf_length=length;
	nrf_read=true;

	ioport_set_pin_level(NRF_CSN,  IOPORT_PIN_LEVEL_LOW);
	NRF_SPI.DATA=command;

	while(nrf_busy);

	for (uint8_t i=0; i<length; i++) {
		data[i]=nrf_rxbuff[i];
	}
}


void nrf24_ssiISR(void)
{
	// rx_pos 0 = status-byte, pos 1 = buffer byte 0
	static int8_t buf_pos=-1;

	// first byte is status register from NRF
	if (buf_pos == -1) {
		nrf_status = NRF_SPI.DATA;
	} else if (buf_pos < nrf_length) {
		nrf_rxbuff[buf_pos] = NRF_SPI.DATA;
	} else {
		//TODO: ERROR
	}
	buf_pos++;

	bool tx_completed = (buf_pos == nrf_length);
	// reset interface at end of message
	if(tx_completed) {
		buf_pos=-1;
		nrf_busy=false;
		ioport_set_pin_level(NRF_CSN,  IOPORT_PIN_LEVEL_HIGH);
	} else if (nrf_read) {
		NRF_SPI.DATA = 0x00;
	} else {
		NRF_SPI.DATA = nrf_txbuff[buf_pos];	
	}
}

void nrf24ISR(void)
{
	// use for further async communication enhancments - interrupts:
	//  - NRF_STATUS_RX_DR
	//  - NRF_STATUS_TX_DS
	//  - NRF_STATUS_MAX_RT
}

#endif

/*
 * nrf24.h
 *
 * nrf24l01+ click driver library for tiva C evaluation board
 *
 *
 *  Created on: Jan 13, 2016
 *      Author: folfy
 */

#ifndef NRF24_H
#define NRF24_H

// command, register and value definitions
#include "nrf24_def.h"

void     nrf24_init(uint8_t channel, uint8_t payload_len, uint32_t tx_address, uint32_t rx_address);

// mode configuration
void     nrf24_mode_rx(void);
void     nrf24_mode_tx(void);
void     nrf24_mode_standby(void);
Bool     nrf24_is_rx(void);
Bool     nrf24_is_tx(void);

// FIFO package buffer functions
void     nrf24_data_send(uint8_t tx_buffer[], uint8_t size);
void     nrf24_data_put(uint8_t rx_buffer[], uint8_t size);
uint8_t  nrf24_data_get(uint8_t rx_buffer[]);
bool     nrf24_data_avail(void);
bool     nrf24_space_avail(void);
void     nrf24_flush_tx(void);

// status and configuration functions
uint8_t  nrf24_status(void);
void     nrf24_set_ch(uint8_t channel);
void     nrf24_set_retrans(uint8_t datarate, uint8_t max_packet_size, uint8_t retrans_count);
void     nrf24_set_tx_addr(uint32_t address);
void     nrf24_set_rx_addr(uint32_t address);
void     nrf24_reg_write(uint8_t reg, uint8_t data);
uint8_t  nrf24_reg_read(uint8_t reg);

// low-level access API functions
void     nrf24_write32(uint8_t command, uint8_t length, uint32_t data[]);
void     nrf24_write(uint8_t command, uint8_t length, uint8_t data[]);
void     nrf24_read(uint8_t command, uint8_t length, uint8_t data[]);

#endif /* NRF24_H */

/*
 * nrf24_def.h
 *
 * NRF24L01 definition file
 *
 * Contains all commands and register addresses referenced in the datasheet
 *
 *  Created on: Jan 19, 2016
 *      Author: folfy
 */

#ifndef NRF24_DEF_H
#define NRF24_DEF_H

/* general purpose commands
 *
 * payload order: LSByte first
 */
#define NRF_CMD_REG_READ          0b00000000  // read register (bit 4:0 - add register address)
#define NRF_CMD_REG_WRITE         0b00100000  // write register (bit 4:0 - add register address)
#define NRF_CMD_RX_READ           0b01100001  // read RX-payload from FIFO
#define NRF_CMD_TX_WRITE          0b10100000  // write TX-payload to FIFO
#define NRF_CMD_TX_FLUSH          0b11100001  // Flush TX-FIFO (used in TX mode)
#define NRF_CMD_RX_FLUSH          0b11100010  // Flush RX-FIFO (used in RX mode)
#define NRF_CMD_TX_REUSE          0b11100011  // PTX only - Reuse last TX-payload (till TX_WRITE or TX_FLUSH)
#define NRF_CMD_NOP               0b11111111  // no operation (read STATUS register)


/* special commands
 *
 * To enable access to these three special functions, the activate command, followed by
 * the activate magic packet has to be sent, while the device is in standby or power down mode.
 */
#define NRF_CMD_ACTIVATE          0x01010000  // toggle special commands active (standby / power down only)
#define NRF_ACTIVATE_MAGIC        0x73

#define NRF_CMD_RX_WIDTH          0b01100000                   // read width of top RX payload (no. of bytes)
#define NRF_CMD_RX_WRITE_ACK_BASE 0b10101000                   // write ACK-payload pipe X base command address
#define NRF_CMD_RX_WRITE_ACK_P0   NRF_CMD_RX_WRITE_ACK_BASE+0  // write ACK-payload pipe 0
#define NRF_CMD_RX_WRITE_ACK_P1   NRF_CMD_RX_WRITE_ACK_BASE+1  // write ACK-payload pipe 1
#define NRF_CMD_RX_WRITE_ACK_P2   NRF_CMD_RX_WRITE_ACK_BASE+2  // write ACK-payload pipe 2
#define NRF_CMD_RX_WRITE_ACK_P3   NRF_CMD_RX_WRITE_ACK_BASE+3  // write ACK-payload pipe 3
#define NRF_CMD_RX_WRITE_ACK_P4   NRF_CMD_RX_WRITE_ACK_BASE+4  // write ACK-payload pipe 4
#define NRF_CMD_RX_WRITE_ACK_P5   NRF_CMD_RX_WRITE_ACK_BASE+5  // write ACK-payload pipe 5
#define NRF_CMD_TX_WRITE_NOACK    0b10110000                   // disables AUTOACK on this packet

/*
 * register definitions
 */

#define NRF_REG_CONF              0x00        // basic configuration
#define NRF_CONF_MASK_RX_DR       1<<6        // mask RX_DR interrupt
#define NRF_CONF_MASK_TX_DS       1<<5        // mask TX_DS interrupt
#define NRF_CONF_MASK_MAX_RT      1<<4        // mask MAX_RT interrupt
#define NRF_CONF_EN_CRC           1<<3        // enable CRC
#define NRF_CONF_CRC2             1<<2        // configure 2 CRC-bytes instead of 1
#define NRF_CONF_PWR_UP           1<<1        // power up IC (startup = 1.5ms)
#define NRF_CONF_PRIM_RX          1<<0        // RX/TX control - RX Mode

#define NRF_REG_EN_AUTOACK        0x01        // enable auto ACK for data pipes
#define NRF_EN_AUTOACK_P0         1<<0        // enable auto ACK for data pipe 0
#define NRF_EN_AUTOACK_P1         1<<1        // enable auto ACK for data pipe 1
#define NRF_EN_AUTOACK_P2         1<<2        // enable auto ACK for data pipe 2
#define NRF_EN_AUTOACK_P3         1<<3        // enable auto ACK for data pipe 3
#define NRF_EN_AUTOACK_P4         1<<4        // enable auto ACK for data pipe 4
#define NRF_EN_AUTOACK_P5         1<<5        // enable auto ACK for data pipe 5
#define NRF_EN_AUTOACK_ALL        0b00111111  // enable auto ACK for all data pipes

#define NRF_REG_EN_RX_ADDR        0x02        // enable data pipe RX address
#define NRF_EN_RX_ADDR_P0         1<<0        // enable RX data pipe 0
#define NRF_EN_RX_ADDR_P1         1<<1        // enable RX data pipe 1
#define NRF_EN_RX_ADDR_P2         1<<2        // enable RX data pipe 2
#define NRF_EN_RX_ADDR_P3         1<<3        // enable RX data pipe 3
#define NRF_EN_RX_ADDR_P4         1<<4        // enable RX data pipe 4
#define NRF_EN_RX_ADDR_P5         1<<5        // enable RX data pipe 5

#define NRF_REG_ADDR_WIDTH        0x03        // setup address width
#define NRF_ADDR_WIDTH_3          0b01        // set address width to 3 bytes
#define NRF_ADDR_WIDTH_4          0b10        // set address width to 4 bytes
#define NRF_ADDR_WIDTH_5          0b11        // set address width to 5 bytes

#define NRF_REG_RETRANS           0x04        // setup retransmission delay and retries...
#define NRF_RETRANS_RETRIES       0           // offset for max. retry count (0-15)
#define NRF_RETRANS_DELAY         4           // offset for retransmission delay (0-15=250-4000µs)
/*
 * set zero to disable automatic retransmission
 * low nibble  (bit 3:0) = max. retry count (0-15)
 * high nibble (bit 7:4) = retransmission delay with t=(n+1)*250µs (0-15=250-4000µs)
 */

#define NRF_REG_RF_CHANNEL        0x05        // select RF channel frequency (0-127=2400-2517MHz)

#define NRF_REG_RF_SETUP          0x06        // RF stage setup
#define NRF_RF_SETUP_CONT_WAVE    1<<7        // enable continuous carrier transmitting (testing)
#define NRF_RF_SETUP_PLL_LOCK     1<<4        // force PLL lock signal (testing)
#define NRF_RF_SETUP_DR_250KBPS   1<<5        // set RF data rate to 250kbps
#define NRF_RF_SETUP_DR_1MBPS     0<<3        // set RF data rate to 1Mbps
#define NRF_RF_SETUP_DR_2MBPS     1<<3        // set RF data rate to 2Mbps
#define NRF_RF_SETUP_PWR_30DBU    0b11<<1     // set RF output power to 0dBm   / 30dBu
#define NRF_RF_SETUP_PWR_24DBU    0b10<<1     // set RF output power to -6dBm  / 24dBu
#define NRF_RF_SETUP_PWR_18DBU    0b01<<1     // set RF output power to -12dBm / 18dBu
#define NRF_RF_SETUP_PWR_12DBU    0b00<<1     // set RF output power to -18dBm / 12dBu
#define NRF_RF_SETUP_LNA_HIGH     1           // increase LNA gain +~1.5dBm (high current mode +~0.7mA)

#define NRF_REG_STATUS            0x07        // RF status register
#define NRF_STATUS_RX_DR          1<<6        // RX data ready - write to clear flag
#define NRF_STATUS_TX_DS          1<<5        // packet transmitted or ACK received in auto-ACK mode - write to clear flag
#define NRF_STATUS_MAX_RT         1<<4        // maximum number of TX retransmissions reached - write to clear flag
#define NRF_STATUS_RXP_NO         0b111<<1    // pipe number for the available payload from RX_FIFO (111 = empty)
#define NRF_STATUS_TX_FULL        1<<0        // TX FIFO full flag

#define NRF_REG_OBSERVE_TX        0x08        // TX observation register...
/*
 * high nibble (bit 7:4) = packet loss count (overflow-protected, reset by writing REG_RF_CHANNEL)
 * low nibble  (bit 3:0) = retransmission-try counter (reset when a new packet starts)
 */

#define NRF_REG_RPD               0x09        // Received Power Detector (bit 0, delayed and latched!)

#define NRF_REG_RX_ADDR0          0x0A        // RX address pipe 0 (3-5 bytes)
#define NRF_REG_RX_ADDR1          0x0B        // RX address pipe 1 (3-5 bytes)
#define NRF_REG_RX_ADDR2          0x0C        // RX address pipe 2 (only LSB, MSBytes equal pipe 1)
#define NRF_REG_RX_ADDR3          0x0D        // RX address pipe 3 (only LSB, MSBytes equal pipe 1)
#define NRF_REG_RX_ADDR4          0x0E        // RX address pipe 4 (only LSB, MSBytes equal pipe 1)
#define NRF_REG_RX_ADDR5          0x0F        // RX address pipe 5 (only LSB, MSBytes equal pipe 1)

#define NRF_REG_TX_ADDR           0x10        // PTX only - TX address for auto-ACK with Enhanced ShockBurst

#define NRF_REG_RX_LENGTH0        0x11        // RX payload pipe 0 length (0-31 = 1-32byte)
#define NRF_REG_RX_LENGTH1        0x12        // RX payload pipe 1 length (0-31 = 1-32byte)
#define NRF_REG_RX_LENGTH2        0x13        // RX payload pipe 2 length (0-31 = 1-32byte)
#define NRF_REG_RX_LENGTH3        0x14        // RX payload pipe 3 length (0-31 = 1-32byte)
#define NRF_REG_RX_LENGTH4        0x15        // RX payload pipe 4 length (0-31 = 1-32byte)
#define NRF_REG_RX_LENGTH5        0x16        // RX payload pipe 5 length (0-31 = 1-32byte)

#define NRF_REG_FIFO_STATUS       0x17        // FIFO status register
#define NRF_FIFO_STATUS_TX_REUSE  1<<6        // PTX only - tx reuse last payload active flag (set with NRF_CMD_TX_REUSE)
#define NRF_FIFO_STATUS_TX_FULL   1<<5        // TX FIFO full flag
#define NRF_FIFO_STATUS_TX_EMPTY  1<<4        // TX FIFO empty flag
#define NRF_FIFO_STATUS_RX_FULL   1<<1        // RX FIFO full flag
#define NRF_FIFO_STATUS_RX_EMPTY  1<<0        // RX FIFO empty flag

#define NRF_REG_DYN_PAYLOAD       0x1C        // enable dynamic payload length for data pipes
#define NRF_DYN_PAYLOAD_P0        1<<0        // enable dynamic payload mode for pipe 0
#define NRF_DYN_PAYLOAD_P1        1<<1        // enable dynamic payload mode for pipe 1
#define NRF_DYN_PAYLOAD_P2        1<<2        // enable dynamic payload mode for pipe 2
#define NRF_DYN_PAYLOAD_P3        1<<3        // enable dynamic payload mode for pipe 3
#define NRF_DYN_PAYLOAD_P4        1<<4        // enable dynamic payload mode for pipe 4
#define NRF_DYN_PAYLOAD_P5        1<<5        // enable dynamic payload mode for pipe 5
#define NRF_DYN_PAYLOAD_ALL       0b00111111  // enable dynamic payload mode for all pipes

#define NRF_REG_FEATURES          0x1D        // feature register
#define NRF_FEATURES_EN_DYN_PAY   1<<2        // enable dynamic payload length
#define NRF_FEATURES_EN_ACK       1<<1        // enable payload with ack
#define NRF_FEATURES_EN_DYN_ACK   1<<0        // enables the NRF_CMD_TX_WRITE_NOACK command

#endif /* NRF24_DEF_H */

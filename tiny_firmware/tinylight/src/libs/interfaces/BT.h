/*
 * BT.h
 *
 * Created: 29.04.2014 19:04:32
 *  Author: Folfy
 */ 


#ifndef BT_H_
#define BT_H_

void BT_init(void);
uint_fast8_t BT_getc(void);
void BT_read_buf(uint_fast8_t buf[], uint_fast8_t size);
uint_fast8_t BT_get_nb_received_data(void);

void BT_putc(int value);
void BT_write_buf(uint_fast8_t buf[], iram_size_t size);

#endif /* BT_H_ */
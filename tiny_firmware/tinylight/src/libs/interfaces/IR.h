/*
 * IR.h
 *
 * Created: 29.04.2014 16:38:23
 *  Author: Martin
 */ 


#ifndef IR_H_
#define IR_H_

void IR_init(void);
uint_fast8_t get_ir_bit(uint_fast16_t time);
void TCC0_OVF_int(void);
void change_color(uint_fast8_t id, uint_fast8_t rgb_buffer[],bool save);

#endif /* IR_H_ */

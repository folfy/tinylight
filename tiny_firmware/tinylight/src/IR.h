/*
 * IR.h
 *
 * Created: 10.04.2014 12:48:06
 *  Author: Martin
 */ 

#ifdef IR_avail
#ifndef IR_H_
#define IR_H_

uint_fast8_t get_ir_byte(uint_fast16_t time);


#define TCC0_cycle 32000000/64
#define nec_start_max 0.015*TCC0_cycle
#define nec_start_min 0.012*TCC0_cycle
#define nec_repeat_max 0.012*TCC0_cycle
#define nec_one_max 0.003*TCC0_cycle
#define nec_zero_max 0.002*TCC0_cycle

#endif /* IR_H_ */
#endif
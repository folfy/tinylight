/*
 * main.h
 *
 * Created: 03.02.2014 18:19:08
 *  Author: Martin
 */ 

#ifndef MAIN_H_
#define MAIN_H_

void power_down(void);

void button_update(Bool key_state);
Bool mode_set_prev(void);
void mode_update(uint_fast8_t mode);
Bool get_state(uint_fast8_t state_mask);
void count_update(uint_fast8_t count);

void status_led_update(void);
void status_led_off(void);

void read_settings(void);
void save_settings(void);



#endif /* MAIN_H_ */

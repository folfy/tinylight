/*
 * set_sled.h
 *
 * Created: 21.04.2014 20:03:44
 *  Author: Folfy
 */ 


#ifndef SET_SLED_H_
#define SET_SLED_H_

extern settings set;

void read_settings(void);
void save_settings(void);

void mode_reset(void);
void mode_set_prev(void);
Bool mode_update(uint_fast8_t mode);
void count_update(uint_fast8_t count);

void rtc_button(uint32_t time);

void status_led_update(void);
void status_led_off(void);
void rtc_sled(void);


#endif /* SET_SLED_H_ */
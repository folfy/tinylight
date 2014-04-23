/*
 * set_sled.h
 *
 * Created: 21.04.2014 20:03:44
 *  Author: Folfy
 */ 


#ifndef SET_SLED_H_
#define SET_SLED_H_

/* EEPROM */
void read_settings(void);
void save_settings(void);

/* Settings access */
extern settings set;	//handle as read_only
Bool write_set(enum set_address_t address, uint8_t val);
Bool read_set (uint8_t address, uint8_t *val);

void mode_reset(void);
void mode_set_prev(void);
Bool mode_update(uint_fast8_t mode);

void write_gamma(uint_fast8_t gamma);
void write_count(uint_fast8_t count);

/* Button */
void rtc_button(uint32_t time);

/* SLED */
void status_led_update(void);
void status_led_off(void);
void rtc_sled(void);


#endif /* SET_SLED_H_ */
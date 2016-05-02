/*
 * settings_sled.h
 *
 * Created: 21.04.2014 20:03:44
 *  Author: Folfy
 */ 


#ifndef SETTINGS_SLED_H_
#define SETTINGS_SLED_H_

/* EEPROM */
void read_settings(void);
void save_settings(void);

/* Settings access */
extern settings set;										//handle as read_only
Bool write_set(enum set_address_t address, uint8_t val);
Bool read_set (uint8_t address, uint8_t *val);

void mode_set_prev(void);									//sets previous mode if valid
void mode_update(uint_fast8_t mode);

void write_default_mode(uint_fast8_t mode);
void write_timeout_mode(uint_fast8_t mode);
void write_oversample(uint_fast8_t oversamples);
void write_gamma(uint_fast8_t gamma);
void write_sled_bright(uint_fast8_t brightness);
void write_sled_dim(uint_fast8_t brightness);
void write_count(uint_fast8_t count);
void write_fps_lim(uint_fast8_t fps);

/* Button */
void rtc_button(uint32_t time);

/* SLED */
void sled_init(void);
void sled_update(void);
void status_led_off(void);
void rtc_sled(void);


#endif /* SETTINGS_SLED_H_ */
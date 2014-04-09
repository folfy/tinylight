/*
 * main.h
 *
 * Created: 03.02.2014 18:19:08
 *  Author: Martin
 */ 

#ifndef MAIN_H_
#define MAIN_H_

#include <usb_protocol_cdc.h>

#define F_CPU 32000000
#define Version 0x01
#define RTC_cycle 32768/16

typedef struct {
	uint_fast8_t volatile mode;
	uint_fast8_t default_mode;
	uint_fast8_t timeout_mode;
	uint_fast8_t timeout_time;
	uint_fast8_t alpha;
	uint_fast8_t default_alpha;
	uint_fast8_t gamma;
	uint_fast8_t smooth_time;
	uint_fast8_t alpha_min;
	uint_fast8_t lux_max;
	uint_fast8_t stat_LED;
	uint_fast8_t stb_LED;
	uint_fast8_t count;
	uint_fast8_t OCP;
	uint_fast8_t OCP_time;
	uint_fast8_t SCP;
	uint_fast8_t UVP;
} settings;

typedef struct {
	uint_fast16_t current;	//Storm der LED-Stripes in mV
	uint_fast16_t voltage;	//Versorgungsspannung der LED-Stripes in mV
	uint_fast16_t light;		//Umgebungshelligkeit in lux
	uint_fast16_t temp;	//Temperatur in 1/10 °C
} adc_sample;

void callback_init(void);
void power_down(void);

void button_update(Bool key_state);
void mode_update(uint_fast8_t mode);

void status_led_update(void);
void status_led_off(void);
void status_led_blink(void);

void read_settings(void);
void save_settings(void);

void status_bar(uint_fast16_t Wert, uint_fast16_t Wert_max, uint_fast8_t anzahl_Leds);

void frame_update(Bool buffer_update);
void gamma_map(void);
uint_fast16_t multi(uint_fast16_t a, uint_fast16_t b);
void gamma_calc(void);

void Mood_Lamp(uint_fast8_t anzahl_Leds);
void Rainbow(uint_fast8_t anzahl_Leds);
void Colorswirl(uint_fast8_t anzahl_Leds);

void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t buffer[]);

void SetupDMA(Bool multi);
void dma_init(void);
void SPI_start(void);

void main_cdc_rx_notify(uint8_t port);
Bool read_USB(void);
uint_fast8_t get_USB_char(void);

uint_fast8_t get_ir_byte(uint_fast16_t time);

#endif /* MAIN_H_ */
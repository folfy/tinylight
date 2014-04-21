/*
 * led.h
 *
 * Created: 20.04.2014 23:55:30
 *  Author: Folfy
 */ 


#ifndef LED_H_
#define LED_H_

//////////////////////////////////////////////////////////////////////////
/* LED Software */

extern uint_fast16_t volatile frame_count;

void handle_led_refresh(settings* set);
void frame_update(void);
void gamma_calc(settings* set);

void Mood_Lamp(uint_fast8_t anzahl_Leds);
void Rainbow(uint_fast8_t anzahl_Leds);
void Colorswirl(uint_fast8_t anzahl_Leds);

//////////////////////////////////////////////////////////////////////////
/* LED Driver */

void SPI_start(void);

void TCD1_OVF_int(void);
void DMA_Led_int(dma_callback_t state);

//////////////////////////////////////////////////////////////////////////
/* DMA */

void dma_init(uint8_t count);

void SetupDMA(Bool multi);

#endif /* LED_H_ */
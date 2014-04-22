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

extern volatile uint_fast16_t FPS;

void handle_led_refresh(void);
void frame_update(void);
void gamma_calc(void);

void Mood_Lamp(void);
void Rainbow(void);
void Colorswirl(void);

//////////////////////////////////////////////////////////////////////////
/* LED Driver */

void SPI_TIMER_OVF_int(void);
void SPI_DMA_int(dma_callback_t state);
void rtc_fps(void);

//////////////////////////////////////////////////////////////////////////
/* DMA */

void dma_init(void);
void SetupDMA(void);

#endif /* LED_H_ */
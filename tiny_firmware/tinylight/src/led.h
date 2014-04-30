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

extern uint8_t back_buffer[];
extern volatile uint_fast16_t FPS;

void frame_update(void);
void gamma_calc(void);

void handle_auto_modes(void);
void Mood_Lamp(void);
void Rainbow(void);
void Colorswirl(void);

//////////////////////////////////////////////////////////////////////////
/* LED Driver */

void rtc_fps(void);

//////////////////////////////////////////////////////////////////////////
/* DMA */

void dma_init(void);
void dma_update_count(void);
void SetupDMA(void);

#endif /* LED_H_ */
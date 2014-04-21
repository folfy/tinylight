/*
 * led.c
 *
 * Created: 20.04.2014 23:55:42
 *  Author: Folfy
 */ 

#include <stdio.h>
#include <asf.h>
#include "linsin.h"
#include "mul16x16.h"
#include "tiny_protocol.h"
#include "data.h"
#include "led.h"

volatile uint_fast16_t frame_count = 0;

static void gamma_map(settings* set);

static void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t rgb_buffer[]);


//////////////////////////////////////////////////////////////////////////
/* LED Software */

uint_fast8_t back_buffer[buffer_size*3];
uint_fast8_t front_buffer[buffer_size*3];

static uint_fast8_t gamma_lut[256];
static uint_fast8_t gamma_ms[256];
static uint_fast8_t ms_mask;

static volatile Bool gamma_update = false;

void handle_led_refresh(settings* set)
{
	if(gamma_update&&!dma_channel_is_busy(DMA_CHANNEL_LED))
	{
		gamma_update=false;
		SPI_start(); // avoid delay if mapping takes longer than 500µs
		gamma_map(set);
	}
}

void frame_update(void)
{
	gamma_update=true;
}

static void gamma_map(settings* set)
{
	if(set->gamma)
	{
		static uint_fast8_t run=0;
		for(uint_fast16_t n=0;n<set->count*3;n++)
		{
			uint_fast8_t val=back_buffer[n];
			front_buffer[n]=gamma_lut[val];
			if(gamma_ms[val]>run)				//oversample pwm for increased resolution
			front_buffer[n]++;
		}
		run=(run+1)&ms_mask;
	}
	else
	{
		for(uint_fast16_t n=0;n<set->count*3;n++)
		front_buffer[n]=back_buffer[n];
	}
}

void gamma_calc(settings* set)
{
	//split gamma in full exponentiations and 10th roots
	uint_fast8_t gamma_dec=set->gamma;
	uint_fast8_t gamma_pot=0;
	while(gamma_dec>=10)
	{
		gamma_pot++;
		gamma_dec-=10;
	}

	uint_fast16_t val_max=255<<set->oversample;

	switch(set->oversample)
	{
		case 0x01:	ms_mask=0x01;
		break;
		case 0x02:	ms_mask=0x03;
		break;
		case 0x03:	ms_mask=0x07;
		break;
		default:	ms_mask=0x00;
		break;
	}
	uint_fast16_t val_mask=ms_mask;
	
	for(uint8_t x=0;x++<255;)
	{
		uint_fast16_t y=65280;
		uint_fast16_t root=(nvm_flash_read_byte( (flash_addr_t) &root_10[x]+1) <<8)+nvm_flash_read_byte( (flash_addr_t) &root_10[x]);
		
		for(uint_fast8_t cnt=gamma_pot;cnt;cnt--)
		y=MulU16X16toH16Round(y,x*257);	//marginal error by shifting (equals div 256) instead of dividing by 255 is left
		for(uint_fast8_t cnt=gamma_dec;cnt;cnt--)
		y=MulU16X16toH16Round(y,root);

		y=MulU16X16toH16Round(y,val_max);
		gamma_lut[x]=y>>set->oversample;
		gamma_ms[x]=y&val_mask;
	}
}

static uint_fast16_t hue1 = 0;

void Mood_Lamp(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		hsv_to_rgb(hue1,back_buffer);
		hue1 = (hue1 + 2) % 1536;
		frame_update();
	}
}

void Rainbow(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		uint_fast16_t hue2 = hue1;
		for (uint_fast8_t k=0; k<anzahl_Leds; k++)
		{
			hsv_to_rgb(hue2, &back_buffer[k*3]);
			hue2  += 40;
		}
		hue1 = (hue1 + 4) % 1536;
		frame_update();
	}
}

void Colorswirl(uint_fast8_t anzahl_Leds)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_cycle;
		/* x=linsin(alpha); x e ]-1...1]= -32767...32768; alpha e [0...259.99]=65535 (1deg=256) */
		/* sine1, sine2 - 1deg=128 - prevent overflow at 260deg */
		const uint_fast16_t deg360 = 46080, rad_3 = 17.2*128, rad_03 = 1.72*128;
		static uint_fast16_t sine1 = 0;
		
		uint_fast16_t sine2 = sine1;
		uint_fast16_t hue2 = hue1;

		for (uint_fast8_t k=0; k<anzahl_Leds; k++)
		{
			uint_fast8_t rgb_buffer[3];
			uint_fast8_t bright;
			hsv_to_rgb(hue2, rgb_buffer);
			if(sine2 >= deg360)
			sine2-= deg360;
			bright = linsin_360(sine2)+127;
			
			back_buffer[k*3]	=	bright * rgb_buffer[0] >> 8;		//R
			back_buffer[k*3+1]	=	bright * rgb_buffer[1] >> 8;		//G
			back_buffer[k*3+2]	=	bright * rgb_buffer[2] >> 8;		//B
			
			hue2  += 40;
			sine2 += rad_3;	// 0.3 rad
		}
		hue1   = (hue1 + 4) % 1536;
		sine1 -= rad_03;		// 0.03 rad
		if(sine1>=deg360)
		sine1-=(UINT_FAST16_MAX-deg360);
		frame_update();
	}
}

static void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t rgb_buffer[])
{
	uint_fast8_t lo = hue & 255;
	switch((hue >> 8) % 6) {
		case 0:
		rgb_buffer[0] = 255;
		rgb_buffer[1] = lo;
		rgb_buffer[2] = 0;
		return;
		case 1:
		rgb_buffer[0] = 255 - lo;
		rgb_buffer[1] = 255;
		rgb_buffer[2] = 0;
		return;
		case 2:
		rgb_buffer[0] = 0;
		rgb_buffer[1] = 255;
		rgb_buffer[2] = lo;
		return;
		case 3:
		rgb_buffer[0] = 0;
		rgb_buffer[1] = 255 - lo;
		rgb_buffer[2] = 255;
		return;
		case 4:
		rgb_buffer[0] = lo;
		rgb_buffer[1] = 0;
		rgb_buffer[2] = 255;
		return;
		default:
		rgb_buffer[0] = 255;
		rgb_buffer[1] = 0;
		rgb_buffer[2] = 255 - lo;
		return;
	}
}

//////////////////////////////////////////////////////////////////////////
/* LED Driver */

volatile Bool update_frame = false;

/* Start DMA Transfer */
void SPI_start(void)
{
	if (dma_channel_is_busy(DMA_CHANNEL_LED) || TCD1.CTRLA)
	update_frame = true;
	else
	dma_channel_enable(DMA_CHANNEL_LED);
}

//Latch delay DMA (LED)
void TCD1_OVF_int(void)
{
	TCD1.CTRLA = 0; //Stop Timer
	if (update_frame)
	{
		dma_channel_enable(DMA_CHANNEL_LED);
		update_frame = false;
	}
	frame_count++;
}

void DMA_Led_int(dma_callback_t state)
{
	if(!get_state(state_multi))
		SetupDMA(true);
	tc_restart(&TCD1);
	tc_set_resolution(&TCD1,500000);
	gamma_update=true;
}

//////////////////////////////////////////////////////////////////////////
/* DMA */

struct dma_channel_config dmach_conf_single;
struct dma_channel_config dmach_conf_multi;

/* Init DMA setup struct */
void dma_init(uint8_t count)
{
	//single
	memset(&dmach_conf_single, 0, sizeof(dmach_conf_single));

	dma_channel_set_burst_length		(&dmach_conf_single, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_single, 3);
	dma_channel_set_repeats				(&dmach_conf_single, count);

	dma_channel_set_src_reload_mode		(&dmach_conf_single, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_single, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_single, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_single, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_single, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_single, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_single, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot			(&dmach_conf_single);
	
	dma_channel_set_interrupt_level		(&dmach_conf_single, DMA_INT_LVL_MED);
	
	//multi
	memset(&dmach_conf_multi, 0, sizeof(dmach_conf_multi));
	
	dma_channel_set_burst_length		(&dmach_conf_multi, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_multi, count*3);

	dma_channel_set_src_reload_mode		(&dmach_conf_multi, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_multi, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_multi, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_multi, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_multi, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_multi, (uint16_t)(uintptr_t)&USARTC0_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_multi, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_single_shot			(&dmach_conf_multi);
	
	dma_channel_set_interrupt_level		(&dmach_conf_multi, DMA_INT_LVL_MED);
	
	dma_enable();
};

/* Config DMA in single / multi LED mode */
void SetupDMA(Bool multi)
{
	while (dma_channel_is_busy(DMA_CHANNEL_LED))
		;
	if (multi)
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_multi);
	else
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
}
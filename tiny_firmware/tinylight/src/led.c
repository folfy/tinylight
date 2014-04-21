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
#include "led.h"

extern settings set;

// framerate statistics
volatile uint_fast16_t FPS = 0;
volatile uint_fast16_t frame_count = 0;

//////////////////////////////////////////////////////////////////////////
/* LED Software */

static const uint16_t root_10[256] PROGMEM =
{
	/*	0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F		*/
	0		,37654	,40357	,42027	,43253	,44229	,45043	,45743	,46358	,46907	,47404	,47858	,48276	,48664	,49026	,49366	,	//	0
	49685	,49987	,50274	,50546	,50806	,51055	,51293	,51521	,51741	,51953	,52157	,52354	,52545	,52730	,52909	,53082	,	//	1
	53251	,53415	,53575	,53731	,53882	,54030	,54174	,54315	,54453	,54588	,54719	,54848	,54974	,55098	,55219	,55338	,	//	2
	55455	,55569	,55682	,55792	,55901	,56007	,56112	,56215	,56316	,56416	,56514	,56611	,56706	,56800	,56892	,56984	,	//	3
	57073	,57162	,57249	,57335	,57420	,57504	,57587	,57669	,57750	,57829	,57908	,57986	,58063	,58139	,58214	,58288	,	//	4
	58361	,58434	,58506	,58576	,58647	,58716	,58785	,58853	,58920	,58987	,59053	,59118	,59183	,59247	,59310	,59373	,	//	5
	59435	,59497	,59558	,59618	,59678	,59738	,59796	,59855	,59913	,59970	,60027	,60083	,60139	,60195	,60250	,60304	,	//	6
	60358	,60412	,60465	,60518	,60571	,60623	,60674	,60725	,60776	,60827	,60877	,60927	,60976	,61025	,61073	,61122	,	//	7
	61170	,61217	,61265	,61312	,61358	,61405	,61451	,61496	,61542	,61587	,61632	,61676	,61720	,61764	,61808	,61851	,	//	8
	61894	,61937	,61980	,62022	,62064	,62106	,62148	,62189	,62230	,62271	,62311	,62352	,62392	,62432	,62471	,62511	,	//	9
	62550	,62589	,62628	,62666	,62705	,62743	,62781	,62818	,62856	,62893	,62930	,62967	,63004	,63041	,63077	,63113	,	//	A
	63149	,63185	,63220	,63256	,63291	,63326	,63361	,63396	,63430	,63465	,63499	,63533	,63567	,63601	,63634	,63668	,	//	B
	63701	,63734	,63767	,63800	,63832	,63865	,63897	,63929	,63962	,63993	,64025	,64057	,64088	,64120	,64151	,64182	,	//	C
	64213	,64244	,64274	,64305	,64335	,64366	,64396	,64426	,64456	,64485	,64515	,64545	,64574	,64603	,64633	,64662	,	//	D
	64691	,64719	,64748	,64777	,64805	,64834	,64862	,64890	,64918	,64946	,64974	,65001	,65029	,65057	,65084	,65111	,	//	E
	65138	,65165	,65192	,65219	,65246	,65273	,65299	,65326	,65352	,65379	,65405	,65431	,65457	,65483	,65509	,65535		//	F
};

static uint_fast8_t gamma_lut[256];
static uint_fast8_t gamma_ms[256];
static uint_fast8_t ms_mask;

uint_fast8_t back_buffer [buffer_size*3];
uint_fast8_t front_buffer[buffer_size*3];

static volatile Bool gamma_update = false;

static void gamma_map(void);
static void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t rgb_buffer[]);
static void SPI_start(void);

void handle_led_refresh(void)
{
	if(gamma_update&&!dma_channel_is_busy(DMA_CHANNEL_LED))
	{
		gamma_update=false;
		SPI_start(); // avoid delay if mapping takes longer than 500�s
		gamma_map();
	}
}

void frame_update(void)
{
	gamma_update=true;
}

static void gamma_map(void)
{
	if(set.gamma)
	{
		static uint_fast8_t run=0;
		for(uint_fast16_t n=0;n<set.count*3;n++)
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
		for(uint_fast16_t n=0;n<set.count*3;n++)
		front_buffer[n]=back_buffer[n];
	}
}

void gamma_calc()
{

	//split gamma in full exponentiations and 10th roots
	uint_fast8_t gamma_dec=set.gamma;
	uint_fast8_t gamma_pot=0;
	while(gamma_dec>=10)
	{
		gamma_pot++;
		gamma_dec-=10;
	}

	uint_fast16_t val_max=255<<set.oversample;

	switch(set.oversample)
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
		gamma_lut[x]=y>>set.oversample;
		gamma_ms[x]=y&val_mask;
	}
}

static uint_fast16_t hue1 = 0;

void Mood_Lamp(void)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_freq;
		hsv_to_rgb(hue1,back_buffer);
		hue1 = (hue1 + 2) % 1536;
		frame_update();
	}
}

void Rainbow(void)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_freq;
		uint_fast16_t hue2 = hue1;
		for (uint_fast8_t k=0; k<set.count; k++)
		{
			hsv_to_rgb(hue2, &back_buffer[k*3]);
			hue2  += 40;
		}
		hue1 = (hue1 + 4) % 1536;
		frame_update();
	}
}

void Colorswirl(void)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+0.010*RTC_freq;
		/* x=linsin(alpha); x e ]-1...1]= -32767...32768; alpha e [0...259.99]=65535 (1deg=256) */
		/* sine1, sine2 - 1deg=128 - prevent overflow at 260deg */
		const uint_fast16_t deg360 = 46080, rad_3 = 17.2*128, rad_03 = 1.72*128;
		static uint_fast16_t sine1 = 0;
		
		uint_fast16_t sine2 = sine1;
		uint_fast16_t hue2 = hue1;

		for (uint_fast8_t k=0; k<set.count; k++)
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
static void SPI_start(void)
{
	if (dma_channel_is_busy(DMA_CHANNEL_LED) || SPI_TIMER.CTRLA)
		update_frame = true;
	else
		dma_channel_enable(DMA_CHANNEL_LED);
}

//Latch delay DMA (LED)
void SPI_TIMER_OVF_int(void)
{
	SPI_TIMER.CTRLA = 0; //Stop Timer
	if (update_frame)
	{
		dma_channel_enable(DMA_CHANNEL_LED);
		update_frame = false;
	}
	frame_count++;
}

void SPI_DMA_int(dma_callback_t state)
{
	if(!(set.mode&state_multi))
		SetupDMA(true);
	tc_restart(&SPI_TIMER);
	tc_set_resolution(&SPI_TIMER,500000);
	gamma_update=true;
}

void rtc_fps(void)
{
	const uint_fast8_t prescaler = 1/RTC_time+0.5;	//Round
	static uint_fast8_t cycle=prescaler;
	if(!--cycle)
	{
		FPS = frame_count;
		frame_count=0;
		cycle=prescaler;
	}
}

//////////////////////////////////////////////////////////////////////////
/* DMA */

struct dma_channel_config dmach_conf_single;
struct dma_channel_config dmach_conf_multi;

/* Init DMA setup struct */
void dma_init(void)
{
	//single
	memset(&dmach_conf_single, 0, sizeof(dmach_conf_single));

	dma_channel_set_burst_length		(&dmach_conf_single, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_single, 3);
	dma_channel_set_repeats				(&dmach_conf_single, set.count);

	dma_channel_set_src_reload_mode		(&dmach_conf_single, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_single, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_single, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_single, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_single, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_single, (uint16_t)(uintptr_t)&LED_UART_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_single, LED_UART_DMA_TRIG_DRE);
	dma_channel_set_single_shot			(&dmach_conf_single);
	
	dma_channel_set_interrupt_level		(&dmach_conf_single, DMA_INT_LVL_MED);
	
	//multi
	memset(&dmach_conf_multi, 0, sizeof(dmach_conf_multi));
	
	dma_channel_set_burst_length		(&dmach_conf_multi, DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count		(&dmach_conf_multi, set.count*3);

	dma_channel_set_src_reload_mode		(&dmach_conf_multi, DMA_CH_SRCRELOAD_BLOCK_gc);
	dma_channel_set_src_dir_mode		(&dmach_conf_multi, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_source_address		(&dmach_conf_multi, (uint16_t)(uintptr_t)front_buffer);

	dma_channel_set_dest_reload_mode	(&dmach_conf_multi, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_dest_dir_mode		(&dmach_conf_multi, DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_destination_address	(&dmach_conf_multi, (uint16_t)(uintptr_t)&LED_UART_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_multi, LED_UART_DMA_TRIG_DRE);
	dma_channel_set_single_shot			(&dmach_conf_multi);
	
	dma_channel_set_interrupt_level		(&dmach_conf_multi, DMA_INT_LVL_MED);
	
	tc_set_overflow_interrupt_callback(&SPI_TIMER,SPI_TIMER_OVF_int);
	dma_set_callback(DMA_CHANNEL_LED,(dma_callback_t) SPI_DMA_int);	//TODO: Init DMA split
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
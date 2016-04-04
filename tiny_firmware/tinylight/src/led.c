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
#include "set_sled.h"
#include "led.h"

uint_fast8_t back_buffer [BUFFER_SIZE*3];
uint_fast8_t front_buffer[BUFFER_SIZE*3];

// framerate statistics
volatile uint_fast16_t FPS = 0;
volatile uint_fast16_t frame_count = 0;

static volatile Bool update_gamma = false;
static volatile Bool update_frame = false;

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

static void gamma_map(void);
static void hsv_to_rgb(uint_fast16_t hue, uint_fast8_t rgb_buffer[]);

void frame_update(void)
{
	if(dma_channel_is_busy(DMA_CHANNEL_LED))
		update_gamma=true;
	else
	{
		if(SPI_TIMER.CTRLA)
		{
			update_frame=true;
			gamma_map();
		}
		else
		{
			gamma_map();
			dma_channel_enable(DMA_CHANNEL_LED);
		}
	}
}

uint_fast8_t run=0;

static void gamma_map(void)
{
	if(set.gamma)
	{
		for(uint_fast16_t n=0;n<set.count*3;n++)
		{
			uint_fast8_t val=back_buffer[n];
			val=(val*set.alpha)>>8;				//TODO: implement auto_alpha
			front_buffer[n]=gamma_lut[val];
			if(gamma_ms[val]>run)				//oversample pwm for increased resolution
				front_buffer[n]++;
		}
	}
	else
	{
		for(uint_fast16_t n=0;n<set.count*3;n++)
			front_buffer[n]=back_buffer[n];
	}
}

void gamma_calc(void)
{
	//split gamma in full exponentiations and 10th roots
	uint_fast8_t gamma_dec=set.gamma>>1;
	uint_fast8_t gamma_pot=0;
	while(gamma_dec>=10)
	{
		gamma_pot++;
		gamma_dec-=10;
	}

	uint_fast16_t val_max=255<<set.oversample;

	switch(set.oversample)
	{
		case OVERSAMPLE_X2:	ms_mask=0x01;	break;
		case OVERSAMPLE_X4:	ms_mask=0x03;	break;
		case OVERSAMPLE_X8:	ms_mask=0x07;	break;
		default:			ms_mask=0x00;	break;
	}
	uint_fast16_t val_mask=ms_mask;
	
	for(uint8_t x=0;x++<255;)
	{
		uint_fast16_t y=65280;
		uint_fast16_t root=(nvm_flash_read_byte( (flash_addr_t) &root_10[x]+1) <<8)+nvm_flash_read_byte( (flash_addr_t) &root_10[x]);
		
		for(uint_fast8_t cnt=gamma_pot;cnt;cnt--)
		y=MulU16X16toH16Round(y,x*257);	// reduce shifting errors (equals div 256) -> x e [0,254]  (reduced range due to brightness adjustment)
		for(uint_fast8_t cnt=gamma_dec;cnt;cnt--)
		y=MulU16X16toH16Round(y,root);

		y=MulU16X16toH16Round(y,val_max);
		gamma_lut[x]=y>>set.oversample;
		gamma_ms[x]=y&val_mask;
	}
}

uint_fast16_t frame_time;

void handle_auto_modes(void)
{
	static uint_fast32_t time1=0;
	if(rtc_get_time()>=time1)
	{
		time1=rtc_get_time()+frame_time;
		switch (set.mode)
		{
			case MODE_MOODLAMP:		Mood_Lamp();	break;
			case MODE_RAINBOW:		Rainbow();		break;
			case MODE_COLORSWIRL:	Colorswirl();	break;
			default: break;
		}
	}
}

void fps_lim_update(void)
{
	frame_time=RTC_FREQ/set.fps_lim;
}

static uint_fast16_t hue1 = 0;

void Mood_Lamp(void)
{
	hsv_to_rgb(hue1,back_buffer);
	hue1 = (hue1 + 2) % 1536;
	frame_update();
}

void Rainbow(void)
{
	uint_fast16_t hue2 = hue1;
	for (uint_fast8_t k=0; k<set.count; k++)
	{
		hsv_to_rgb(hue2, &back_buffer[k*3]);
		hue2  += 40;
	}
	hue1 = (hue1 + 4) % 1536;
	frame_update();
}

void Colorswirl(void)
{
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

static void SPI_TIMER_OVF_int(void);
static void SPI_DMA_int(dma_callback_t state);

//Latch delay DMA (LED)
static void SPI_TIMER_OVF_int(void)
{
	tc_write_clock_source(&SPI_TIMER,TC_CLKSEL_OFF_gc);
	if (update_frame)
	{
		dma_channel_enable(DMA_CHANNEL_LED);
		update_frame = false;
	}
	frame_count++;
}

static void SPI_DMA_int(dma_callback_t state)
{
	if(!(set.mode&STATE_MULTI))
		SetupDMA();
	tc_restart(&SPI_TIMER);
	tc_write_clock_source(&SPI_TIMER,TC_CLKSEL_DIV64_gc);
	if(update_gamma||(set.oversample&&set.gamma&&(set.mode&STATE_ON)))
	{
		update_frame=true;
		gamma_map();
		run=(run+1)&ms_mask;
	}
}

void rtc_fps(void)
{
	const uint_fast8_t prescaler = 1/RTC_TIME+0.5;	//Round
	static uint_fast8_t cycle=prescaler;
	if(!--cycle)
	{
		FPS = frame_count;
		frame_count=0;
		cycle=prescaler;
	}
}

static void usart_init_spi_pull_up(USART_t *usart, const usart_spi_options_t *opt);
static void dma_init(void);

/* Init LED strip USART as SPI Master */
void led_init(void)
{
	static usart_spi_options_t USART_SPI_OPTIONS = {
		.spimode	= USART_CMODE_MSPI_gc,
		.baudrate	= 1000000,					//1Mhz Clock
		.data_order	= 0							//SPI Mode 0
	};
	usart_init_spi_pull_up(&LED_USART,&USART_SPI_OPTIONS);
	ioport_set_pin_mode(LED_TX, IOPORT_MODE_WIREDANDPULL|IOPORT_MODE_SLEW_RATE_LIMIT|IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_dir(LED_TX,IOPORT_DIR_OUTPUT);
	
	/* Init SPI latch delay timer */
	tc_enable(&SPI_TIMER);
	tc_set_overflow_interrupt_level(&SPI_TIMER,TC_INT_LVL_MED);
	tc_write_period(&SPI_TIMER,1000/2);
	
	dma_init();
}



//asf_usart_init_spi modified sck_pin pin configuration
#  define USART_UCPHA_bm 0x02
#  define USART_DORD_bm 0x04

static void usart_init_spi_pull_up(USART_t *usart, const usart_spi_options_t *opt)
{
	ioport_pin_t sck_pin;
	//bool invert_sck;

	sysclk_enable_peripheral_clock(usart);

	usart_rx_disable(usart);

	/* configure Clock polarity using INVEN bit of the correct SCK I/O port **/
	//invert_sck = (opt->spimode == 2) || (opt->spimode == 3);
	//UNUSED(invert_sck);

	#ifdef USARTC0
	if ((uint16_t)usart == (uint16_t)&USARTC0) {
		#  ifdef PORT_USART0_bm
		if (PORTC.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTC, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTC, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTC, 1);
		#  endif
	}
	#endif
	#ifdef USARTC1
	if ((uint16_t)usart == (uint16_t)&USARTC1) {
		sck_pin = IOPORT_CREATE_PIN(PORTC, 5);
	}
	#endif
	#ifdef USARTD0
	if ((uint16_t)usart == (uint16_t)&USARTD0) {
		#  ifdef PORT_USART0_bm
		if (PORTD.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTD, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTD, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTD, 1);
		#  endif
	}
	#endif
	#ifdef USARTD1
	if ((uint16_t)usart == (uint16_t)&USARTD1) {
		sck_pin = IOPORT_CREATE_PIN(PORTD, 5);
	}
	#endif
	#ifdef USARTE0
	if ((uint16_t)usart == (uint16_t)&USARTE0) {
		#  ifdef PORT_USART0_bm
		if(PORTE.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTE, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTE, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTE, 1);
		#  endif
	}
	#endif
	#ifdef USARTE1
	if ((uint16_t)usart == (uint16_t)&USARTE1) {
		sck_pin = IOPORT_CREATE_PIN(PORTE, 5);
	}
	#endif
	#ifdef USARTF0
	if ((uint16_t)usart == (uint16_t)&USARTF0) {
		#  ifdef PORT_USART0_bm
		if(PORTF.REMAP & PORT_USART0_bm) {
			sck_pin = IOPORT_CREATE_PIN(PORTF, 5);
			} else {
			sck_pin = IOPORT_CREATE_PIN(PORTF, 1);
		}
		#  else
		sck_pin = IOPORT_CREATE_PIN(PORTF, 1);
		# endif
	}
	#endif
	#ifdef USARTF1
	if ((uint16_t)usart == (uint16_t)&USARTF1) {
		sck_pin = IOPORT_CREATE_PIN(PORTF, 5);
	}
	#endif

	/* Configure the USART output pin */
	ioport_set_pin_dir(sck_pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(sck_pin, IOPORT_MODE_WIREDANDPULL|IOPORT_MODE_SLEW_RATE_LIMIT|IOPORT_MODE_INVERT_PIN);
	
	//ioport_set_pin_mode(sck_pin,
	//IOPORT_MODE_TOTEM | (invert_sck? IOPORT_MODE_INVERT_PIN : 0));
	ioport_set_pin_level(sck_pin, IOPORT_PIN_LEVEL_HIGH);

	usart_set_mode(usart, USART_CMODE_MSPI_gc);

	if (opt->spimode == 1 || opt->spimode == 3) {
		usart->CTRLC |= USART_UCPHA_bm;
		} else {
		usart->CTRLC &= ~USART_UCPHA_bm;
	}
	if (opt->data_order) {
		(usart)->CTRLC |= USART_DORD_bm;
		} else {
		(usart)->CTRLC &= ~USART_DORD_bm;
	}

	usart_spi_set_baudrate(usart, opt->baudrate, sysclk_get_per_hz());
	usart_tx_enable(usart);
	//usart_rx_enable(usart);
}

//////////////////////////////////////////////////////////////////////////
/* DMA */

struct dma_channel_config dmach_conf_single;
struct dma_channel_config dmach_conf_multi;

/* Init DMA setup struct */
static void dma_init(void)
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
	dma_channel_set_destination_address	(&dmach_conf_single, (uint16_t)(uintptr_t)&LED_USART_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_single, LED_USART_DMA_TRIG_DRE);
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
	dma_channel_set_destination_address	(&dmach_conf_multi, (uint16_t)(uintptr_t)&LED_USART_DATA);

	dma_channel_set_trigger_source		(&dmach_conf_multi, LED_USART_DMA_TRIG_DRE);
	dma_channel_set_single_shot			(&dmach_conf_multi);
	
	dma_channel_set_interrupt_level		(&dmach_conf_multi, DMA_INT_LVL_MED);
	
	tc_set_overflow_interrupt_callback(&SPI_TIMER,SPI_TIMER_OVF_int);
	dma_set_callback(DMA_CHANNEL_LED,(dma_callback_t) SPI_DMA_int);
	dma_enable();
};

void dma_update_count(void)
{
	dma_channel_set_repeats				(&dmach_conf_single, set.count);
	dma_channel_set_transfer_count		(&dmach_conf_multi,  set.count*3);
	SetupDMA();
}

/* Config DMA in single / multi LED mode */
void SetupDMA(void)
{
	while (dma_channel_is_busy(DMA_CHANNEL_LED));	//DMA_Blocking
	if (set.mode&STATE_MULTI)
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_multi);
	else
		dma_channel_write_config(DMA_CHANNEL_LED, &dmach_conf_single);
}
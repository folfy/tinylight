/*
 * tiny_protocol.h
 *
 * Created: 03.02.2014 18:19:08
 *  Author: Folfy
 */ 

#ifndef TINY_PROTOCOL_H_
#define TINY_PROTOCOL_H_

#define protocol_rev 0x03

#ifdef _WIN32
#define term 1
#elif __AVR__
#define term 0
#else
#warning "Unknown architecture"
#endif


//UNDONE: Enum modes, etc.
extern const uint8_t preamble[3+term];
extern const uint8_t response[3+term];
extern const uint8_t pre_ada[3+term];
extern const uint8_t ack_ada[5+term];

#define ack					0x00	//Ack new state 
#define nack_preamble		0xC0	//state_preamble:	preamble mismatch
#define nack_command		0xC1	//state_command:	unknown command
#define nack_raw_mode		0xC2	//state_raw:		not in usb mode
#define nack_read_add		0xC3	//state_set_read:	address unknown
#define nack_write_crc		0xC4	//state_set_write:	address or data not matching
#define nack_write_add		0xC5	//state_set_write:	unknown address
#define nack_ada_crc		0xD0	//state_ada_header:	length crc mismatch
#define nack_ada_length		0xD1	//state_ada_header:	length too high
#define nack_timeout		0xE0	//any state but state_preamble:	interrupted transmission detected, timeout bit 3:0(LSB) represent last usb_state, see block below
#define nack				0xFF

#define usb_state_idle			0
#define usb_state_cmd			1
#define usb_state_raw_single	2
#define usb_state_raw_multi		3
#define usb_state_set_read		4
#define usb_state_set_write		5
#define usb_state_ada_header	6
#define usb_state_ada_raw		7

#define cmd_test			0x00
#define cmd_raw_data		0x11
#define cmd_measure			0x22
#define cmd_set_read		0x33
#define cmd_set_write		0x44
#define cmd_set_save		0x55

#define set_mode			0
#define set_default_mode	1
#define set_timeout_mode	2
#define set_timeout_time	3
#define set_oversample		4
#define set_alpha			5
#define set_default_alpha	6
#define set_gamma			7
#define set_smooth_time		8
#define set_alpha_min		9
#define set_lux_max			10
#define set_stat_Led		11
#define set_stb_Led			12
#define set_count			13
#define set_OCP				14
#define set_OCP_time		15
#define set_SCP				16
#define set_UVP				17

#define state_on			0b10000000
#define state_error			0b01000000
#define state_usb			0b00100000
#define state_multi			0b00010000
#define state_prev			state_error	//	reuse error state bit for default mode = previous mode

enum mode_t {
	mode_off					=	(	0x00															),
	mode_usb_single				=	(	0x00	| state_on					| state_usb					),
	mode_usb_multi				=	(	0x01	| state_on					| state_usb	| state_multi	),
	mode_usb_ada				=	(	0x08	| state_on					| state_usb	| state_multi	),
	mode_mood_lamp				=	(	0x00	| state_on												),
	mode_rainbow				=	(	0x01	| state_on								| state_multi	),
	mode_colorswirl				=	(	0x02	| state_on								| state_multi	),
	mode_error_BOP				=	(	0x00				| state_error								),
	mode_error_UVP				=	(	0x01				| state_error								),
	mode_error_SCP				=	(	0x04				| state_error								),
	mode_error_internal			=	(	0x0F				| state_error								)
};

enum mode_def_t {
	mode_def_usb_single			=	(	0x00	| state_on					| state_usb					),
	mode_def_usb_multi			=	(	0x01	| state_on					| state_usb	| state_multi	),
	mode_def_usb_ada			=	(	0x08	| state_on					| state_usb	| state_multi	),
	mode_def_mood_lamp			=	(	0x00	| state_on												),
	mode_def_rainbow			=	(	0x01	| state_on								| state_multi	),
	mode_def_colorswirl			=	(	0x02	| state_on								| state_multi	),
	mode_def_prev_off			=	(	0x00				| state_prev								),
	mode_def_prev_usb_single	=	(	0x00	| state_on	| state_prev	| state_usb					),
	mode_def_prev_usb_multi		=	(	0x01	| state_on	| state_prev	| state_usb	| state_multi	),
	mode_def_prev_usb_ada		=	(	0x08	| state_on	| state_prev	| state_usb	| state_multi	),
	mode_def_prev_mood_lamp		=	(	0x00	| state_on	| state_prev								),
	mode_def_prev_rainbow		=	(	0x01	| state_on	| state_prev				| state_multi	),
	mode_def_prev_colorswirl	=	(	0x02	| state_on	| state_prev				| state_multi	)
	};

enum oversample_t {
oversample_off	=	0x00,
oversample_x2	=	0x01,
oversample_x4	=	0x02,
oversample_x8	=	0x03
};

#define timeout_vbus		0x00

#define alpha_auto			0x00
#define	scp_off				0x00
#define uvp_off				0x00

typedef struct {
	enum mode_t volatile	mode;
	enum mode_def_t			default_mode;
	enum mode_t				timeout_mode;
	uint_fast8_t			timeout_time;
	enum oversample_t		oversample;
	uint_fast8_t			alpha;
	uint_fast8_t			default_alpha;
	uint_fast8_t			gamma;
	uint_fast8_t			smooth_time;
	uint_fast8_t			alpha_min;
	uint_fast8_t			lux_max;
	uint_fast8_t			stat_LED;
	uint_fast8_t			stb_LED;
	uint_fast8_t			count;
	uint_fast8_t			SCP;
	uint_fast8_t			UVP;
} settings;

typedef struct {
	uint_fast16_t voltage;	//Versorgungsspannung der LED-Stripes in mV
	uint_fast16_t current;	//Storm der LED-Stripes in mV
	uint_fast16_t light;	//Umgebungshelligkeit in lux
	uint_fast16_t temp;		//Temperatur in 1/10 °C
} adc_sample;

#endif /* TINY_PROTOCOL_H_ */

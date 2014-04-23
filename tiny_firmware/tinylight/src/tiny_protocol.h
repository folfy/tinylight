/*
 * tiny_protocol.h
 *
 * Created: 03.02.2014 18:19:08
 *  Author: Folfy
 */ 

#ifndef TINY_PROTOCOL_H_
#define TINY_PROTOCOL_H_

#define PROTOCOL_REV 0x03

#ifdef _WIN32
#define term 1
#elif __AVR__
#define term 0
#else
#warning "Unknown architecture"
#endif

extern const uint8_t preamble[3+term];
extern const uint8_t response[3+term];
extern const uint8_t pre_ada[3+term];
extern const uint8_t ack_ada[5+term];

enum usb_response_t {
	ACK						=	0,		//Ack new state 
	NACK_PREAMPLE			=	100,	//state_idle:		preamble mismatch
	NACK_COMMAND			=	101,	//state_command:	unknown command
	NACK_RAW_MODE			=	102,	//state_raw:		not in correct usb mode
	NACK_READ_ADD			=	103,	//state_set_read:	address unknown
	NACK_WRITE_CRC			=	104,	//state_set_write:	address or data not matching
	NACK_WRITE_ADD			=	105,	//state_set_write:	unknown address
	NACK_ADA_CRC			=	110,	//state_ada_header:	length crc mismatch
	NACK_ADA_LENGTH			=	111,	//state_ada_header:	length too high
	NACK_TIMEOUT			=	200,	//Internal value, if received -> fatal error
	NACK_TIMEOUT_CMD		=	201,	//command not received, transmission timed out
	NACK_TIMEOUT_RAW_SINGLE	=	202,	//raw data not received, transmission timed out
	NACK_TIMEOUT_RAW_MULTI	=	203,	//multi raw cmd received, transmission timed out
	NACK_TIMEOUT_SET_READ	=	204,	//read address not received, transmission timed out
	NACK_TIMEOUT_SET_WRITE	=	205,	//write address/value not received, transmission timed out
	NACK_TIMEOUT_ADA_HEADER =	206,	//ada header incomplete, transmission timed out
	NACK_TIMEOUT_ADA_RAW	=	207,	//ada raw data not received, transmission timed out
	NACK					=	255		//unknown error
	};

enum usb_cmd_t {
	CMD_TEST				=	0x00,
	CMD_RAW_DATA			=	0x11,
	CMD_MEASURE				=	0x22,
	CMD_SET_READ			=	0x33,
	CMD_SET_WRITE			=	0x44,
	CMD_SET_SAVE			=	0x55
	};

enum set_address_t {
	set_mode				=	0,
	set_default_mode		=	1,
	set_timeout_mode		=	2,
	set_timeout_time		=	3,
	set_oversample			=	4,
	set_alpha				=	5,
	set_default_alpha		=	6,
	set_gamma				=	7,
	set_smooth_time			=	8,
	set_alpha_min			=	9,
	set_lux_max				=	10,
	set_sled_bright			=	11,
	set_sled_dim			=	12,
	set_count				=	13,
	set_SCP					=	14,
	set_UVP					=	15
	};
	
#define STATE_ON			0b10000000
#define STATE_ERROR			0b01000000
#define STATE_USB			0b00100000
#define STATE_MULTI			0b00010000
#define STATE_PREV			STATE_ERROR	//	reuse error state bit for default mode = previous mode

enum mode_t {
	MODE_OFF					=	(	0x00															),
	MODE_USB_SINGLE				=	(	0x00	| STATE_ON					| STATE_USB					),
	MODE_USB_MULTI				=	(	0x01	| STATE_ON					| STATE_USB	| STATE_MULTI	),
	MODE_USB_ADA				=	(	0x08	| STATE_ON					| STATE_USB	| STATE_MULTI	),
	MODE_MOODLAMP				=	(	0x00	| STATE_ON												),
	MODE_RAINBOW				=	(	0x01	| STATE_ON								| STATE_MULTI	),
	MODE_COLORSWIRL				=	(	0x02	| STATE_ON								| STATE_MULTI	),
	MODE_ERROR_BOP				=	(	0x00				| STATE_ERROR								),
	MODE_ERROR_UVP				=	(	0x01				| STATE_ERROR								),
	MODE_ERROR_SCP				=	(	0x04				| STATE_ERROR								),
	MODE_ERROR_INTERNAL			=	(	0x0F				| STATE_ERROR								)
};

enum mode_def_t {
	MODE_DEF_MOODLAMP			=	(	0x00	| STATE_ON												),
	MODE_DEF_RAINBOW			=	(	0x01	| STATE_ON								| STATE_MULTI	),
	MODE_DEF_COLORSWIRL			=	(	0x02	| STATE_ON								| STATE_MULTI	),
	MODE_DEF_PREV_OFF			=	(	0x00				| STATE_PREV								),
	MODE_DEF_PREV_MOODLAMP		=	(	0x00	| STATE_ON	| STATE_PREV								),
	MODE_DEF_PREV_RAINBOW		=	(	0x01	| STATE_ON	| STATE_PREV				| STATE_MULTI	),
	MODE_DEF_PREV_COLORSWIRL	=	(	0x02	| STATE_ON	| STATE_PREV				| STATE_MULTI	)
	};

//timeout_time
#define TIMEOUT_VBUS		0x00
#define TIMEOUT_OFF			0xFF

enum oversample_t {
OVERSAMPLE_OFF	=	0x00,
OVERSAMPLE_X2	=	0x01,
OVERSAMPLE_X4	=	0x02,
OVERSAMPLE_X8	=	0x03
};

#define ALPHA_AUTO			0x00
#define	SCP_OFF				0x00
#define UVP_OFF				0x00

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

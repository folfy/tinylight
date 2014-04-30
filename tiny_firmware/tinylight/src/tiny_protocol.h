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
extern const uint8_t pre_BT[2+term];

//NACK and ACK suppressed in ada_mode
enum usb_response_t {
	ACK						=	0,		//Ack new state 
	NACK_PREAMPLE			=	100,	//state_idle:		preamble mismatch
	NACK_COMMAND			=	101,	//state_command:	unknown command
	NACK_RAW_MODE			=	102,	//state_raw:		not in correct usb mode
	NACK_READ_ADD			=	103,	//state_set_read:	address unknown
	NACK_WRITE_ADD			=	104,	//state_set_write:	unknown address
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
	CMD_TEST				=	0,
	CMD_RAW_DATA			=	11,
	CMD_MEASURE				=	22,
	CMD_SET_READ			=	33,
	CMD_SET_WRITE			=	44,
	CMD_SET_SAVE			=	55
	};

enum set_address_t {
	SET_MODE				=	0,
	SET_DEFAULT_MODE		=	1,
	SET_TIMEOUT_MODE		=	2,
	SET_TIMEOUT_TIME		=	3,
	SET_OVERSAMPLE			=	4,
	SET_ALPHA				=	5,
	SET_DEFAULT_ALPHA		=	6,
	SET_GAMMA				=	7,
	SET_SMOOTH_TIME			=	8,
	SET_ALPHA_MIN			=	9,
	SET_LUX_MAX				=	10,
	SET_SLED_BRIGHT			=	11,
	SET_SLED_DIM			=	12,
	SET_COUNT				=	13,
	SET_READ_ALL			=	0xFF
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
	MODE_SINGLE					=	(	0x03	| STATE_ON												)
};

enum mode_def_t {
	MODE_DEF_MOODLAMP			=	(	0x00	| STATE_ON												),
	MODE_DEF_RAINBOW			=	(	0x01	| STATE_ON								| STATE_MULTI	),
	MODE_DEF_COLORSWIRL			=	(	0x02	| STATE_ON								| STATE_MULTI	),
	MODE_DEF_SINGLE				=	(	0x03	| STATE_ON												),
	MODE_DEF_PREV_OFF			=	(	0x00				| STATE_PREV								),
	MODE_DEF_PREV_MOODLAMP		=	(	0x00	| STATE_ON	| STATE_PREV								),
	MODE_DEF_PREV_RAINBOW		=	(	0x01	| STATE_ON	| STATE_PREV				| STATE_MULTI	),
	MODE_DEF_PREV_COLORSWIRL	=	(	0x02	| STATE_ON	| STATE_PREV				| STATE_MULTI	),
	MODE_DEF_PREV_SINGLE		=	(	0x03	| STATE_ON	| STATE_PREV								)
	};

enum oversample_t {
	OVERSAMPLE_OFF	=	0x00,
	OVERSAMPLE_X2	=	0x01,
	OVERSAMPLE_X4	=	0x02,
	OVERSAMPLE_X8	=	0x03
	};

#define TIMEOUT_VBUS		0x00
#define ALPHA_AUTO			0x00
#define GAMMA_OFF			0x00

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
	uint_fast8_t			sled_bright;
	uint_fast8_t			sled_dim;
	uint_fast8_t			count;
} settings;

typedef struct {
	uint_fast16_t voltage;	//Versorgungsspannung der LED-Stripes in mV
	int_fast16_t  current;	//Storm der LED-Stripes in mV
	uint_fast16_t light;	//Umgebungshelligkeit in lux
	uint_fast16_t temp;		//Temperatur in 1/10 °C
} adc_sample;

#endif /* TINY_PROTOCOL_H_ */

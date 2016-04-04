/*
 * tiny_protocol.h
 *
 * Protocol description for tinyLight
 *
 * Works with Lightpack (Adalight emulation - legacy), and tinyLight Software.
 * Commands always start with the preamble, followed by the command and eventual parameters.
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

enum usb_cmd_t {
	CMD_TEST				=	0,		//Returns: Ack, "LED", Protocol_rev, SW_rev, HW_rev, Mode
	CMD_RAW_DATA			=	11,		//Returns: Ack if ready for data; Parameter: binary led values (after ack)
	CMD_MEASURE				=	22,		//Returns: Ack, U [mV], I [mA], Brightness [Lux], Temp [1/10C], output framerate [1/s], U_min, I_max; (all values are uint16, little endian - LSB first)
	CMD_SET_READ			=	33,		//Parameter: <set_address_t>; Returns: Ack, address, value
	CMD_SET_WRITE			=	44,		//Parameter: <set_address_t>, <new value>; Returns: Ack, address, new value
	CMD_SET_SAVE			=	55		//Returns: Ack; Stores settings in EEPROM
};

enum set_address_t {
	SET_MODE				=	0,		//<mode_t>		- not validated!
	SET_DEFAULT_MODE		=	1,		//<mode_def_t>	- not validated!
	SET_TIMEOUT_MODE		=	2,		//<mode_def_t>	- not validated!
	SET_TIMEOUT_TIME		=	3,		//USB timeout: 0x00 - VBus only, .1 ... 25.5 sec (.1s)
	SET_OVERSAMPLE			=	4,		//<oversample mode>
	SET_ALPHA				=	5,		//Brightness: 0x00 - Auto, 0 ... 100% (.39%)
	SET_DEFAULT_ALPHA		=	6,		//Brightness: 0x00 - Auto, 0 ... 100% (.39%)
	SET_GAMMA				=	7,		//Gamma correction: 0x00 - Off (incl. Alpha + Oversample), .05 ... 12.75
	SET_SMOOTH_TIME			=	8,
	SET_ALPHA_MIN			=	9,
	SET_LUX_MAX				=	10,
	SET_SLED_BRIGHT			=	11,		//Status LED brightness (On):      0 ... 100% (.39%)
	SET_SLED_DIM			=	12,		//Status LED brightness (Standby): 0 ... 100% (.39%)
	SET_COUNT				=	13,		//Number of LEDs (max: BUFFER_SIZE)
	SET_FPS_LIM				=	14,		//Limit framerate in auto mode (reduces flicker when oversampling)
	SET_DEBUG				=   254,    //Reserved for debugging purposes
	SET_READ_ALL			=	255		//Return all settings from 0 to sizeof(settings)
};

/* mode description
 *
 *	bit 7 (MSB):	On (Strip power / sleep mode)
 *	bit 6:			reserved (default mode: set previous mode)
 *	bit 5:			USB
 *	bit 4:			Multiple values (individual LED control)
 *	bit 3-0 (LSB):	designator (unique identifier)
 *
 */
	
#define STATE_ON			0b10000000
#define STATE_RES			0b01000000
#define STATE_USB			0b00100000
#define STATE_MULTI			0b00010000
#define STATE_PREV			STATE_RES	// reuse reserved state bit for default mode = previous mode

enum mode_t {
	MODE_SLEEP					=	(	0x00															),	//UID: 0x00	(  0)
	MODE_OFF                    =   (   0x01                                                            ),  //UID: 0x01 (  1)
	MODE_USB_SINGLE				=	(	0x00	| STATE_ON					| STATE_USB					),	//UID: 0xA0 (160)
	MODE_USB_MULTI				=	(	0x01	| STATE_ON					| STATE_USB	| STATE_MULTI	),	//UID: 0xB1 (177)
	MODE_USB_ADA				=	(	0x08	| STATE_ON					| STATE_USB	| STATE_MULTI	),	//UID: 0xB8 (184)
	MODE_MOODLAMP				=	(	0x00	| STATE_ON												),	//UID: 0x80	(128)
	MODE_RAINBOW				=	(	0x01	| STATE_ON								| STATE_MULTI	),	//UID: 0x91	(145)
	MODE_COLORSWIRL				=	(	0x02	| STATE_ON								| STATE_MULTI	),	//UID: 0x92 (146)
	MODE_SINGLE					=	(	0x03	| STATE_ON												)	//UID: 0x83	(131)
};

enum mode_def_t {
	MODE_DEF_MOODLAMP			=	(	0x00	| STATE_ON												),	//UID: 0x00	(  0)
	MODE_DEF_RAINBOW			=	(	0x01	| STATE_ON								| STATE_MULTI	),	//UID: 0xA0 (160)
	MODE_DEF_COLORSWIRL			=	(	0x02	| STATE_ON								| STATE_MULTI	),	//UID: 0xB1 (177)
	MODE_DEF_SINGLE				=	(	0x03	| STATE_ON												),	//UID: 0xB8 (184)
	MODE_DEF_PREV_OFF			=	(	0x00				| STATE_PREV								),	//UID: 0x40 ( 64)
	MODE_DEF_PREV_MOODLAMP		=	(	0x00	| STATE_ON	| STATE_PREV								),	//UID: 0xC0 (192)
	MODE_DEF_PREV_RAINBOW		=	(	0x01	| STATE_ON	| STATE_PREV				| STATE_MULTI	),	//UID: 0xD1 (209)
	MODE_DEF_PREV_COLORSWIRL	=	(	0x02	| STATE_ON	| STATE_PREV				| STATE_MULTI	),	//UID: 0xD2 (210)
	MODE_DEF_PREV_SINGLE		=	(	0x03	| STATE_ON	| STATE_PREV								)	//UID: 0xC3 (195)
};

enum oversample_t {
	OVERSAMPLE_OFF	=	0x00,	//Off
	OVERSAMPLE_X2	=	0x01,	//2xMS
	OVERSAMPLE_X4	=	0x02,	//4xMS
	OVERSAMPLE_X8	=	0x03	//8xMS
};

#define TIMEOUT_VBUS		0x00
#define ALPHA_AUTO			0x00
#define GAMMA_OFF			0x00

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
	NACK_TIMEOUT			=	200,	//no data timeout (if set), switched to timeout mode
	NACK_TIMEOUT_CMD		=	201,	//command not received, transmission timed out
	NACK_TIMEOUT_RAW_SINGLE	=	202,	//raw data not received, transmission timed out
	NACK_TIMEOUT_RAW_MULTI	=	203,	//multi raw cmd received, transmission timed out
	NACK_TIMEOUT_SET_READ	=	204,	//read address not received, transmission timed out
	NACK_TIMEOUT_SET_WRITE	=	205,	//write address/value not received, transmission timed out
	NACK_TIMEOUT_ADA_HEADER =	206,	//ada header incomplete, transmission timed out
	NACK_TIMEOUT_ADA_RAW	=	207,	//ada raw data not received, transmission timed out
	NACK					=	255		//unknown error
};

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
	uint_fast8_t			fps_lim;
} settings;

typedef struct {
	uint_fast16_t	voltage;	//Supply voltage (LED - 5V) [mV]
	int_fast16_t	current;	//LED current draw [mA]
	int_fast16_t	light;		//Environmental brightness [Lux]
	uint_fast16_t	temp;		//Temperature [1/10 °C]
} adc_sample;

#endif /* TINY_PROTOCOL_H_ */


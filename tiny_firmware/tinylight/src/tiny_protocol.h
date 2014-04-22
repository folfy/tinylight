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

#define set_mode			0x00
#define set_default_mode	0x01
#define set_timeout_mode	0x02
#define set_timeout_time	0x03
#define set_oversample		0x04
#define set_alpha			0x10
#define set_default_alpha	0x11
#define set_gamma			0x12
#define set_smooth_time		0x13
#define set_alpha_min		0x14
#define set_lux_max			0x15
#define set_stat_Led		0x20
#define set_stb_Led			0x21
#define set_count			0x30
#define set_OCP				0x31
#define set_OCP_time		0x32
#define set_SCP				0x33
#define set_UVP				0x34

#define state_on			0b10000000
#define state_error			0b01000000
#define state_usb			0b00100000
#define state_multi			0b00010000

#define mode_prev			state_error	//	reuse error state bit for default mode = previous mode

#define mode_off			(	0x00															)
#define mode_usb_single		(	0x00	| state_on					| state_usb					)
#define mode_usb_multi		(	0x01	| state_on					| state_usb	| state_multi	)
#define mode_usb_ada		(	0x08	| state_on					| state_usb	| state_multi	)
#define mode_mood_lamp		(	0x00	| state_on												)
#define mode_rainbow		(	0x01	| state_on								| state_multi	)
#define mode_colorswirl		(	0x02	| state_on								| state_multi	)
#define mode_error_BOP		(	0x00				| state_error								)
#define mode_error_UVP		(	0x01				| state_error								)
#define mode_error_OVP		(	0x02				| state_error								)
#define mode_error_OCP		(	0x03				| state_error								)
#define mode_error_SCP		(	0x04				| state_error								)
#define mode_error_internal	(	0x0F				| state_error								)

#define oversample_off		0x00
#define oversample_x2		0x01
#define oversample_x4		0x02
#define oversample_x8		0x03

#define timeout_vbus		0x00

#define alpha_auto			0x00

#define ocp_off				0x00
#define	scp_off				0x00
#define uvp_off				0x00

typedef struct {
	uint_fast8_t volatile mode;
	uint_fast8_t default_mode;
	uint_fast8_t timeout_mode;
	uint_fast8_t timeout_time;
	uint_fast8_t oversample;
	uint_fast8_t alpha;
	uint_fast8_t default_alpha;
	uint_fast8_t gamma;
	uint_fast8_t smooth_time;
	uint_fast8_t alpha_min;
	uint_fast8_t lux_max;
	uint_fast8_t stat_LED;
	uint_fast8_t stb_LED;
	uint_fast8_t count;
	uint_fast8_t OCP;
	uint_fast8_t OCP_time;
	uint_fast8_t SCP;
	uint_fast8_t UVP;
} settings;

typedef struct {
	uint_fast16_t voltage;	//Versorgungsspannung der LED-Stripes in mV
	uint_fast16_t current;	//Storm der LED-Stripes in mV
	uint_fast16_t light;	//Umgebungshelligkeit in lux
	uint_fast16_t temp;		//Temperatur in 1/10 °C
} adc_sample;

#endif /* TINY_PROTOCOL_H_ */

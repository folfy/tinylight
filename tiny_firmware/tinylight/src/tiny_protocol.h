/*
 * tiny_protocol.h
 *
 * Created: 03.02.2014 18:19:08
 *  Author: Folfy
 */ 

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define protocol_rev 0x02

#ifdef _WIN32
#define term 1
#elif __AVR__
#define term 0
#else
#warning "Unknown architecture"
#endif

const char preamble[4+term]	= "tLED";
const char response[3+term]	= "LED";
const char pre_ada[3+term]	= "Ada";

#define ack					0x77
#define nack				0xff

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

#define set_EE_offset		1    // page wise

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
#define mode_error_UVP		(	0x00				| state_error								)
#define mode_error_OVP		(	0x01				| state_error								)
#define mode_error_OCP		(	0x02				| state_error								)
#define mode_error_SCP		(	0x03				| state_error								)
#define mode_error_internal	(	0x0F				| state_error								)

#define timeout_vbus		0x00
#define	timeout_off			0xFF
#define alpha_auto			0x00
#define ocp_off				0x00
#define	scp_off				0x00
#define uvp_off				0x00

#endif

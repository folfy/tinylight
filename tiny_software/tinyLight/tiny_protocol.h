
#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#ifdef _WIN32
const char preamble[5] = "tLED";
const char response[4] = "LED";
#elif __AVR__
const char preamble[4] = "tLED";
const char response[3] = "LED";
#else
#error "ERROR - FAIL - ERROR - FATAL - ERROR - FAIL - ..."
#endif

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

#define set_EE_offset		0x20

#define mode_off			0x00
#define mode_single_led		0x10
#define mode_multi_led		0x11
#define mode_colorswirl		0x20
#define mode_mood_lamp		0x21
#define mode_light_bar		0x22
#define mode_UVP			0xF0
#define mode_OVP			0xF1
#define mode_OCP			0xF2
#define mode_SCP			0xF3

#define mode_prev_offset	0x80

#define state_OFF			0x00
#define state_USB			0x10
#define state_ON			0x20
#define state_ERROR			0xF0

#define state_mask			0xF0


#endif

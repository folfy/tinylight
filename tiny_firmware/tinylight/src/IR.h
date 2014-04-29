/*
 * IR.h
 *
 * Created: 29.04.2014 16:38:23
 *  Author: Martin
 */ 


#ifndef IR_H_
#define IR_H_

enum ir_key_t
{
	IR_off_key					= 2,
	IR_play_pause_key			= 130,
	IR_brightness_plus_key		= 58,
	IR_brightness_minus_key	= 186,

	IR_red_color_key			= 26,
	IR_dark_orange_color_key	= 42,
	IR_orange_color_key		= 10,
	IR_dark_yellow_color_key	= 56,
	IR_yellow_color_key		= 24,

	IR_green_color_key			= 154,
	IR_light_green_color_key	= 170,
	IR_bluegreen_color_key		= 138,
	IR_cyan_color_key			= 184,
	IR_petrol_color_key		= 152,

	IR_blue_color_key			= 162,
	IR_cobalt_blue_color_key	= 146,
	IR_blueviolet_color_key	= 178,
	IR_violet_color_key		= 120,
	IR_pink_color_key			= 88,

	IR_white_color_key			= 34,
	IR_rosa_color_key			= 18,
	IR_rosa_2_color_key		= 50,
	IR_skyblue_color_key		= 248,
	IR_skyblue_2_color_key		= 216,

	IR_red_plus_key			= 40,
	IR_red_minus_key			= 8,
	IR_green_plus_key			= 168,
	IR_green_minus_key			= 136,
	IR_blue_plus_key			= 104,
	IR_blue_minus_key			= 72,

	IR_DIY1_key				= 48,
	IR_DIY2_key				= 176,
	IR_DIY3_key				= 112,
	IR_DIY4_key				= 16,
	IR_DIY5_key				= 144,
	IR_DIY6_key				= 80,

	IR_jump3_key				= 32,
	IR_jump7_key				= 160,
	IR_fade3_key				= 96,
	IR_fade7_key				= 224,

	IR_flash_key				= 208,
	IR_auto_key					= 240,
	IR_speed_plus_key			= 232,
	IR_speed_minus_key			= 200
};

void IR_init(void);
uint_fast8_t get_ir_bit(uint_fast16_t time);
void TCC0_OVF_int(void);
void handle_remote_key(uint_fast8_t addr, enum ir_key_t cmd, bool repeat);
void change_color(uint_fast8_t id, uint_fast8_t rgb_buffer[],bool save);

#define TCC0_cycle 32000000/64
#define nec_start_max 0.015*TCC0_cycle
#define nec_start_min 0.012*TCC0_cycle
#define nec_repeat_max 0.012*TCC0_cycle
#define nec_one_max 0.003*TCC0_cycle
#define nec_zero_max 0.002*TCC0_cycle

#define IR_addr 0
#define IR_colors_EE_offset 64



#endif /* IR_H_ */
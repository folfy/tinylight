/*
 * mul16x16.c
 *
 * Created: 15.04.2014 22:38:39
 *  Author: Folfy
 */ 

#include <stdio.h>
#include "mul16x16.h"

#define MultiU16X16toH16(intRes, intIn1, intIn2) \


uint_fast32_t MulU16X16to32(uint_fast16_t a, uint_fast16_t b)
{
	uint_fast32_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"movw %A0, r0 \n\t"
	"mul %B1, %B2 \n\t"
	"movw %C0, r0 \n\t"
	"mul %B2, %A1 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"mul %B1, %A2 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26"
	);
	return result;
};

uint_fast16_t MulU16X16toH16(uint_fast16_t a, uint_fast16_t b)
{
	uint_fast16_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"mov r27, r1 \n\t"
	"mul %B1, %B2 \n\t"
	"movw %A0, r0 \n\t"
	"mul %B2, %A1 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"mul %B1, %A2 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26" , "r27"
	);
	return result;
};

uint_fast16_t MulU16X16toH16Round(uint_fast16_t a, uint_fast16_t b)
{
	uint_fast16_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"mov r27, r1 \n\t"
	"mul %B1, %B2 \n\t"
	"movw %A0, r0 \n\t"
	"mul %B2, %A1 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"mul %B1, %A2 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"lsl r27 \n\t"
	"adc %A0, r26 \n\t"
	"adc %B0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26" , "r27"
	);
	return result;
};

int_fast32_t MulS16X16to32(int_fast16_t a, int_fast16_t b)
{
	int_fast32_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"movw %A0, r0 \n\t"
	"muls %B1, %B2 \n\t"
	"movw %C0, r0 \n\t"
	"mulsu %B2, %A1 \n\t"
	"sbc %D0, r26 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"mulsu %B1, %A2 \n\t"
	"sbc %D0, r26 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26"
	);
	return result;
};

int_fast16_t MulS16X16toH16(int_fast16_t a, int_fast16_t b)
{
	int_fast16_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"mov r27, r1 \n\t"
	"muls %B1, %B2 \n\t"
	"movw %A0, r0 \n\t"
	"mulsu %B2, %A1 \n\t"
	"sbc %B0, r26 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"mulsu %B1, %A2 \n\t"
	"sbc %B0, r26 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26", "r27"
	);
	return result;
};

int_fast32_t MulSU16X16to32(int_fast16_t a, uint_fast16_t b)
{
	int_fast32_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"movw %A0, r0 \n\t"
	"mulsu %B1, %B2 \n\t"
	"movw %C0, r0 \n\t"
	"mul %B2, %A1 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"mulsu %B1, %A2 \n\t"
	"sbc %D0, r26 \n\t"
	"add %B0, r0 \n\t"
	"adc %C0, r1 \n\t"
	"adc %D0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26"
	);
	return result;
};

int_fast16_t MulSU16X16toH16(int_fast16_t a, uint_fast16_t b)
{
	int_fast16_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"mov r27, r1 \n\t"
	"mulsu %B1, %B2 \n\t"
	"movw %A0, r0 \n\t"
	"mul %B2, %A1 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"mulsu %B1, %A2 \n\t"
	"sbc %B0, r26 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26", "r27"
	);
	return result;
};

int_fast16_t MulSU16X16toH16Round(int_fast16_t a, uint_fast16_t b)
{
	int_fast16_t result;
	asm volatile (
	"clr r26 \n\t"
	"mul %A1, %A2 \n\t"
	"mov r27, r1 \n\t"
	"mulsu %B1, %B2 \n\t"
	"movw %A0, r0 \n\t"
	"mul %A1, %B2 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"mulsu %B1, %A2 \n\t"
	"sbc %B0, r26 \n\t"
	"add r27, r0 \n\t"
	"adc %A0, r1 \n\t"
	"adc %B0, r26 \n\t"
	"lsl r27 \n\t"
	"adc %A0, r26 \n\t"
	"adc %B0, r26 \n\t"
	"clr r1 \n\t"
	:
	"=&r" (result)
	:
	"a" (a),
	"a" (b)
	:
	"r26", "r27"
	);
	return result;
};
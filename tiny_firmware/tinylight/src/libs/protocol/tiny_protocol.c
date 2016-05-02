/*
 * tiny_protocol.c
 *
 * Created: 21.04.2014 01:43:29
 *  Author: Folfy
 */ 

#ifdef _WIN32
#define term 1
#elif __AVR__
#define term 0
#include <stdio.h>
#else
#warning "Unknown architecture"
#endif

const uint8_t preamble[3+term]	= "LeD";
const uint8_t response[3+term]	= "LED";
const uint8_t pre_ada[3+term]	= "Ada";
const uint8_t ack_ada[5+term]	= "Ada\n";
const uint8_t pre_BT[2+term]	= "BT";
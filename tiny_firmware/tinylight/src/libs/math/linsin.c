/*
 * linsin.c
 *
 * Created: 23.03.2014 08:02:01
 *  Author: Folfy
 */ 



#include <avr/pgmspace.h>

#include "linsin.h"

// Nachguck-Tabelle für eine Sinus-Interpolante.
// Die Stützstellen sind volle °-Werte von 0°...90°.
// Die Werte sind so gewählt, daß die Polygonzug-Interpolante
// den Maximalfehler minimiert; daher sind die Werte in der
// Tabelle nicht die Funktionswerte des Sinus an den
// entsprechenden Stellen.
static const int16_t linsin_tab[91] PROGMEM =
{
	0x0000, 0x023c, 0x0478, 0x06b3, 0x08ee, 0x0b28, 0x0d61, 0x0f9a, 0x11d1, 0x1406,
	0x163a, 0x186d, 0x1a9d, 0x1ccb, 0x1ef8, 0x2121, 0x2348, 0x256d, 0x278e, 0x29ad,
	0x2bc8, 0x2ddf, 0x2ff4, 0x3204, 0x3410, 0x3619, 0x381d, 0x3a1d, 0x3c18, 0x3e0f,
	0x4001, 0x41ed, 0x43d5, 0x45b7, 0x4794, 0x496c, 0x4b3d, 0x4d09, 0x4ecf, 0x508e,
	0x5248, 0x53fb, 0x55a7, 0x574d, 0x58eb, 0x5a83, 0x5c14, 0x5d9e, 0x5f20, 0x609b,
	0x620f, 0x637a, 0x64df, 0x663b, 0x678f, 0x68db, 0x6a1f, 0x6b5b, 0x6c8e, 0x6db9,
	0x6edb, 0x6ff5, 0x7106, 0x720e, 0x730d, 0x7403, 0x74f0, 0x75d4, 0x76af, 0x7781,
	0x7849, 0x7908, 0x79bd, 0x7a69, 0x7b0c, 0x7ba5, 0x7c34, 0x7cb9, 0x7d35, 0x7da7,
	0x7e0f, 0x7e6e, 0x7ec2, 0x7f0d, 0x7f4e, 0x7f85, 0x7fb1, 0x7fd4, 0x7fed, 0x7ffc,
	0x7fff
};

// Ein Word per post-Increment aus dem Flash lesen.
// Dies ist besser als  pgm_read_word (p++)
#define pgm_read_word_inc(addr)             \
(__extension__({                            \
	uint16_t __result;                      \
	__asm__                                 \
	(                                       \
	"lpm %A0, Z+"   "\n\t"              \
	"lpm %B0, Z+"                       \
	: "=r" (__result), "+z" (addr)      \
	);                                      \
	__result;                               \
}))

// Multiply-Accumulate
// return c + a*b
// c = 1.15 signed Q-Format
// a = 1.15 signed Q-Format
// b = 0.8 unsigned Q-Format
static inline int16_t fmac16_8 (int16_t c, int16_t a, uint8_t b)
{
	asm ("; fmac16_8: %B[c]:%A[c] += %B[a]:%A[a] * %[b]"  "\n\t"
	"mulsu  %B[a], %[b]"       "\n\t"      // ( (signed)ah * (unsigned)b ) << 1
	"add    %A[c], R0"         "\n\t"
	"adc    %B[c], R1"         "\n\t"
	"mul    %[b], %A[a]"       "\n\t"      // ( (unsigned) b * al ) << 1
	"add    %A[c], R1"         "\n\t"
	"clr    __zero_reg__"      "\n\t"
	"adc    %B[c], __zero_reg__"
	: [c] "+r" (c)
	: [a] "a" (a), [b] "a" (b)
	);
	
	return c;
}

// Cosinus (int8 output)
// phi in [0°,360°) wobei 1° = 128
int8_t linsin_360 (uint16_t phi)
{
	uint8_t neg = 0;
	uint16_t _180_grad = (uint16_t) 180*128;
	// Wir wollen diese Konstante in einem Register
	asm ("" : "+r" (_180_grad));
	
	if (phi >= _180_grad)
	{
		// sin(x) = -sin(x+pi)
		// phi in [0°,180°)
		phi -= _180_grad;
		neg = 1;
	}
	
	if (phi > 90*128)
	// sin(x) = sin(pi-x)
	// phi in [0°,90°]
	phi = _180_grad - phi;
	
	phi <<=1;
	
	// Stützpunkte aus Tabelle lesen.
	// Für phi = 90° ist das formal nicht 100% korrekt,
	// da über das Tabellenende hinausgelesen wird.
	// In diesem Falle ist aber phi.lo=0, so daß unten
	// dann data1-data0 mit 0 multipliziert wird.
	// Ausserdem gibt's auf AVR keine Segmentation Faults ;-)
	const int16_t * p = & linsin_tab[phi >> 8];
	uint16_t data0  = pgm_read_word_inc (p);
	uint16_t data1  = pgm_read_word (p);
	
	// sin = data0 + (data1-data0) * phi_lo
	int8_t si = fmac16_8 (data0, data1-data0, phi) >> 8;
	return neg ? -si : si;
}

// Sinus
// phi in [0°,256°) wobei 1° = 256
int16_t linsin (uint16_t phi)
{
	uint8_t neg = 0;
	uint16_t _180_grad = (uint16_t) 180*256;
	// Wir wollen diese Konstante in einem Register
	asm ("" : "+r" (_180_grad));
	
	if (phi >= _180_grad)
	{
		// sin(x) = -sin(x+pi)
		// phi in [0°,180°)
		phi -= _180_grad;
		neg = 1;
	}
	
	if (phi > 90*256)
	// sin(x) = sin(pi-x)
	// phi in [0°,90°]
	phi = _180_grad - phi;
	
	// Stützpunkte aus Tabelle lesen.
	// Für phi = 90° ist das formal nicht 100% korrekt,
	// da über das Tabellenende hinausgelesen wird.
	// In diesem Falle ist aber phi.lo=0, so daß unten
	// dann data1-data0 mit 0 multipliziert wird.
	// Ausserdem gibt's auf AVR keine Segmentation Faults ;-)
	const int16_t * p = & linsin_tab[phi >> 8];
	uint16_t data0  = pgm_read_word_inc (p);
	uint16_t data1  = pgm_read_word (p);
	
	// sin = data0 + (data1-data0) * phi_lo
	int16_t si = fmac16_8 (data0, data1-data0, phi);
	
	return neg ? -si : si;
}

// Cosinus
// phi in [0°,256°) wobei 1° = 256
int16_t lincos (uint16_t phi)
{
	uint16_t _90_grad = (uint16_t) 90*256;
	// Wir wollen diese Konstante in einem Register
	asm ("" : "+r" (_90_grad));
	
	if (phi <= _90_grad)
	// cos(x) = sin(pi/2 - x)
	return linsin (_90_grad - phi);
	else
	// cos(x) = -sin (x - pi/2)
	return -linsin (phi - _90_grad);
}
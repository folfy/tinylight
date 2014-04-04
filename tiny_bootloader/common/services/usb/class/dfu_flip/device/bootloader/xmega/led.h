#ifndef led
#define led

void led_on()
{
    TCD0_CTRLA = 0;				//Stop Timer
    PORTD.DIRSET = PIN0_bm;
    TCD0_CTRLB = 0x13;	//Link PD0 to Timer; Mode: Single Slope PWM
    TCD0_PER = 0xffff;		//Set TOP value to 16Bit
    TCD0_CCA = 0x7fff;          //Set blue color 50% on
    TCD0_CTRLA = 0x6;	//Start Timer Clk/256
}

//        LDI   R18, 0x00
//        STS   TCD0_CTRLA, R18
//        STS   TCD0_CTRLB, R18
//        LDI   R18, PIN0_bm
//        STS   PORTD_DIRCLR, R18
 
#endif

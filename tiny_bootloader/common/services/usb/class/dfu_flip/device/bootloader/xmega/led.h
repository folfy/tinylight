#ifndef led
#define led

void led_on()
{
    TCD0_CTRLA = 0;	        //Stop Timer
    PORTD.DIRSET = PIN0_bm | PIN2_bm;
    TCD0_CTRLB = 0x53;	        //Link PD0 to Timer; Mode: Single Slope PWM
    TCD0_PER = 0xfff;	        //Set TOP value to 12Bit
    TCD0_CCA = 0x5b7;           //Set blue color
    TCD0_CCC = 0x555;           //Set red color
    TCD0_CTRLA = 1;	        //Start Timer Clk/1
}

//        LDI   R18, 0x00
//        STS   TCD0_CTRLA, R18
//        STS   TCD0_CTRLB, R18
//        LDI   R18, PIN0_bm
//        STS   PORTD_DIRCLR, R18
 
#endif

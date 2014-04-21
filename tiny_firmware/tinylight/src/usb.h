/*
 * usb.h
 *
 * Created: 21.04.2014 12:00:01
 *  Author: Folfy
 */ 


#ifndef USB_H_
#define USB_H_

extern uint8_t back_buffer[];

void usb_init(void);
void rtc_usb(void);
void handle_usb(void);

#endif /* USB_H_ */
/*
 * usb.h
 *
 * Created: 21.04.2014 12:00:01
 *  Author: Folfy
 */ 


#ifndef USB_H_
#define USB_H_

void Vbus_init(void);

void usb_init(void);
void rtc_usb(uint32_t time);
void handle_usb(void);

#endif /* USB_H_ */
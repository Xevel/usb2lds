/*
    usb2lds.h

    Copyright 2014 - Nicolas "Xevel" Saugnier

    Based on:
    - LUFA library, by Dean Camera
    - USB2AX, By Nicolas Saugnier and Richard Ibbotson  
*/
/*
  Original notice:

  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#ifndef _USB2LDS_H_
#define _USB2LDS_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <stdio.h>

		#include "Descriptors.h"

		#include <LUFA/Drivers/Peripheral/Serial.h>
		#include <LUFA/Drivers/Misc/RingBuffer.h>
		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Platform/Platform.h>

		void SetupHardware(void);
        void usart_init();
        void apply_motor_pwm(uint8_t val);
        void tx_led_on();
        void tx_led_off();
        void tx_led_toggle();

		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
        void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
        void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);

#endif


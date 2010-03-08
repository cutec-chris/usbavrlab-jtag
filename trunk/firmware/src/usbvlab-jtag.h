/*
    usbvlab-jtag, by Cahya Wirawan <Cahya.Wirawan at gmail.com> 
    Based on estick-jtag.
    Released under the MIT Licence.
*/

#ifndef _USBVLAB_JTAG_H_
#define _USBVLAB_JTAG_H_

	/* Includes: */
	#include <avr/io.h>
	#include <avr/wdt.h>
	#include <avr/power.h>
	#include <util/delay_basic.h>

	/* Macros: */

	/* Type Defines: */
	
	#define USBVLAB_USB_BUFFER_SIZE 800
	#define USBVLAB_USB_BUFFER_OFFSET 2
	#define USBVLAB_IN_BUFFER_SIZE	 (USBVLAB_USB_BUFFER_SIZE)
	#define USBVLAB_OUT_BUFFER_SIZE  (USBVLAB_USB_BUFFER_SIZE)

	/* Global Variables: */

	/* Function Prototypes: */

  void ProcessData();
  void USBMessageSend(uint8_t *, uint16_t);

#endif //USBVLAB_JTAG

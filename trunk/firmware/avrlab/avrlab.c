/************************************************************************************************
 * Project: USB AVR-Lab
 * Author: Christian Ulrich
 * Contact: christian at ullihome dot de
 *
 * Creation Date: 2007-09-24
 * Copyright: (c) 2007 by Christian Ulrich
 * License: GPLv2 for private use
 *	        commercial use prohibited 
 *
 * Changes:
 ***********************************************************************************************/

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "avrlab.h"
#include "usbdrv.h"
#include "usbconfig.h"
#include "led.h"
#include "timer.h"
#include "main.h"

#ifndef USBASP_COMPATIBLE
led_t leds[] =  {{4,LED_OFF,LED_OFF},
                 {3,LED_OFF,LED_OFF},
                 {5,LED_OFF,LED_OFF}}; 
#else
led_t leds[] =  {{0,LED_OFF,LED_OFF},
                 {1,LED_OFF,LED_OFF},
                 {3,LED_OFF,LED_OFF}}; 
#endif
const uint8_t led_count = sizeof(leds)/sizeof(led_t);

void SetLed(unsigned char led, unsigned char mode,unsigned char frequency)
{
  leds[led].status = mode;
  leds[led].frequency = frequency;
}

void AvrLabSendData(char *data,unsigned int length)
{
}

void AvrLabInit(void)
{
#ifndef SIMULATION
  uint8_t i;
//Reconnect USB
  usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
  i = 0;
  while(--i)
     _delay_ms(2);
  usbDeviceConnect();
  usbInit();
  wdt_enable(WDTO_60MS);
#endif
  sei();
}

void AvrLabPoll(void)
{
#ifndef SIMULATION 
  extern uchar usbNewDeviceAddr;
  wdt_reset();
  usbPoll();
  LED_poll();
  if(usbNewDeviceAddr)
    LED_PORT &= ~(1<<leds[LED_BLUE].pin);
#endif
}



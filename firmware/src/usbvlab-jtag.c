/************************************************************************************************
 * Project: usbvlab-jtag
 * Author:  Cahya Wirawan
 * Contact: Cahya.Wirawan at gmail.com
 *
 * Creation Date: 2010-03-03
 * Copyright: (c) 2010 by Cahya Wirawan
 * License: GPLv2
 *
 ***********************************************************************************************/

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usbvlab-jtag.h"
#include "avrlab.h"
#include "usbdrv.h"
#include "usbconfig.h"
#include "jtag_defs.h"
#include "jtag_functions.h"

uint8_t  usbBuffer[USBVLAB_USB_BUFFER_SIZE+USBVLAB_USB_BUFFER_OFFSET];
uint8_t  *dataFromHost = usbBuffer+USBVLAB_USB_BUFFER_OFFSET;
uint8_t  *dataToHost = usbBuffer;
uint16_t dataFromHostSize=0;
uint16_t dataToHostSize=0;
uint8_t  replyBuffer[10];
uint16_t jtagDelay=0;

uint16_t data_out_length;
uint16_t data_in_length;
usbMsgLen_t currentPosition, bytesRemaining;

volatile uint8_t resetJtagTransfers=0;

usbMsgLen_t usbFunctionSetup(uint8_t data[8])
{
  usbMsgLen_t len = 0;
  usbRequest_t *rq = (void *)data;
  if(data[1] == FUNC_GET_TYPE) {
	  replyBuffer[0] = 6;
	  len = 1;
    usbMsgPtr = replyBuffer;
	}
  else if(data[1] == FUNC_START_BOOTLOADER) {
    cli();
	  wdt_enable(WDTO_15MS);
    while(1);
	  len = 0;
    usbMsgPtr = replyBuffer;
	}
  else if(data[1] == FUNC_WRITE_DATA) {
    currentPosition = 0;
    bytesRemaining = rq->wLength.word;
    if(bytesRemaining > USBVLAB_USB_BUFFER_SIZE)
      bytesRemaining = USBVLAB_USB_BUFFER_SIZE;
    len = USB_NO_MSG;
  }
  else if(data[1] == FUNC_READ_DATA) {
    usbMsgPtr = dataToHost;
    len = dataToHostSize;
  }

  return len;
}

uint8_t usbFunctionWrite( uint8_t  *data, uint8_t  len )
{
  usbMsgLen_t  i;
  if(len > bytesRemaining)
    len = bytesRemaining;
  bytesRemaining -= len;

  if(dataFromHostSize == 0) {
  	dataFromHostSize = *(uint16_t*)&data[0];
  	for(i = sizeof(uint16_t); i < len; i++) {
  	  dataFromHost[currentPosition++] = data[i];
    }
  }
  else {
  	for(i = 0; i < len; i++) {
  	  dataFromHost[currentPosition++] = data[i];
    }
  }  

  if(bytesRemaining == 0) {
    dataToHostSize = jtag_process_data(dataFromHost, dataFromHostSize, dataToHost, jtagDelay);
    dataFromHostSize=0;
    return 1;
  }
  else
    return 0;
}

int main(void)
{
	uint16_t i;

  AvrLabInit();  
  jtag_init();

	for (i = 0; i < USBVLAB_USB_BUFFER_SIZE+USBVLAB_USB_BUFFER_OFFSET; i++) {
		usbBuffer[i] = 0;
	}
  dataFromHostSize=0;
  dataToHostSize=0;
  resetJtagTransfers=0;

  while (1) {
    AvrLabPoll();
	}
}

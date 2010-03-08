#include <avr/io.h>
#include <util/delay_basic.h>

#include "jtag_defs.h"
#include "jtag_functions.h"

//! initialize JTAG interface
void jtag_init(void)
{
  JTAG_DIR_1 |= JTAG_OUTPUT_MASK_1;
  JTAG_DIR_2 |= JTAG_OUTPUT_MASK_2;
  JTAG_OUT_2 |= (1<<JTAG_PIN_TRST)|(1<<JTAG_PIN_SRST);
}

//! send taps through JTAG interface and recieve responce from TDO pin only
//! \parameter out_buffer - buffer of taps for output, data is packed TDI and TMS values a stored together 
//! \parameter out_length - total number of pairs to send (maximum length is 4*255 samples)
//! \parameter in_buffer  - buffer which will hold recieved data data will be packed 
//! \return    number of bytes used in the in_buffer 
uint8_t jtag_tap_output_max_speed(const uint8_t *out_buffer, uint16_t out_length, uint8_t *in_buffer)
{
  uint8_t tms_tdi;
  uint8_t out_data;
  uint8_t in_data = 0;
  uint16_t out_buffer_index = 0;
  uint16_t in_buffer_index = 0;
  uint16_t out_length_index = 0;

#ifdef      DEBUG
  printf("Sending %d bits \r\n", dataFromHostSize);
#endif

  while(1)
  {
    out_data = out_buffer[out_buffer_index++];

    //First TMS/TDI/TDO
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    asm("nop");
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Second TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    asm("nop");
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Third TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    asm("nop");
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;

    //Fourth TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    asm("nop");
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;    
    if(!(out_length_index%8))
    {
      in_buffer[in_buffer_index] = in_data;
      in_buffer_index++;
      in_data = 0;
    }    
    if(out_length_index>=out_length)
      break;
  }

  if(out_length_index%8)
    in_buffer[in_buffer_index] = in_data>>(8-(out_length_index%8));
  
  return (out_length+7)/8;
}

//! send taps through JTAG interface and recieve responce from TDO pin only
//! \parameter out_buffer - buffer of taps for output, data is packed TDI and TMS values a stored together 
//! \parameter out_length - total number of pairs to send (maximum length is 4*255 samples)
//! \parameter in_buffer  - buffer which will hold recieved data data will be packed 
//! \return    number of bytes used in the in_buffer 
uint8_t jtag_tap_output_with_delay(const uint8_t *out_buffer, uint16_t out_length, uint8_t *in_buffer, uint16_t jtagDelay)
{
  uint8_t tms_tdi;
  uint8_t out_data;
  uint8_t in_data = 0;
  uint16_t out_buffer_index = 0;
  uint16_t in_buffer_index = 0;
  uint16_t out_length_index = 0;

#ifdef      DEBUG
  printf("Sending %d bits \r\n", dataFromHostSize);
#endif

  while(1)
  {
    out_data = out_buffer[out_buffer_index++];

    //First TMS/TDI/TDO
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Second TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Third TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;

    //Fourth TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;    
    if(!(out_length_index%8))
    {
      in_buffer[in_buffer_index] = in_data;
      in_buffer_index++;
      in_data = 0;
    }    
    if(out_length_index>=out_length)
      break;
  }

  if(out_length_index%8)
    in_buffer[in_buffer_index] = in_data>>(8-(out_length_index%8));
  
  return (out_length+7)/8;
}


//! send taps through JTAG interface and recieve responce from TDO and EMU pins 
//! \parameter out_buffer - buffer of taps for output, data is packed TDI and TMS values a stored together 
//! \parameter out_length - total number of pairs to send (maximum length is 4*255 samples)
//! \parameter in_buffer  - buffer which will hold recieved data data will be packed 
//! \return    number of bytes used in the in_buffer (equal to the input (length+3)/4
uint8_t jtag_tap_output_emu(const uint8_t *out_buffer,uint16_t out_length,uint8_t *in_buffer, uint16_t jtagDelay)
{
  uint8_t tms_tdi;
  uint8_t out_data;
  uint8_t in_data = 0;
  uint16_t out_buffer_index = 0;
  uint16_t in_buffer_index = 0;
  uint16_t out_length_index = 0;

  while(1)
  {
    out_data = out_buffer[out_buffer_index++];

    //First TMS/TDI/TDO
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Second TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;
    
    //Third TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;
    if(out_length_index>=out_length)
      break;

    //Fourth TMS/TDI/TDO
    out_data = out_data>>2;
    tms_tdi = out_data & JTAG_SIGNAL_INPUT_MASK;
    JTAG_OUT_1 = ( JTAG_OUT_1 & ( ~JTAG_SIGNAL_OUTPUT_MASK ) ) | (tms_tdi<<JTAG_SIGNAL_INPUT_OUTPUT_SHIFT);
    JTAG_OUT_1|=JTAG_CLK_HI;//CLK hi
    _delay_loop_2(jtagDelay);
    JTAG_OUT_1&=JTAG_CLK_LO;//CLK lo
    _delay_loop_2(jtagDelay);
    
    in_data = in_data>>1;
    in_data |= ((JTAG_IN_1<<(7-JTAG_PIN_TDO))&0x80);
    out_length_index++;    
    if(!(out_length_index%8))
    {
      in_buffer[in_buffer_index] = in_data;
      in_buffer_index++;
      in_data = 0;
    }    
    if(out_length_index>=out_length)
      break;
  }

  if(out_length_index%4)
    in_buffer[in_buffer_index] = in_data>>(8-2*(out_length_index%4));
  
  return (out_length+3)/4;
}

//! return current status of TDO & EMU pins
//! \return packed result TDO - bit 0 , EMU bit 1
uint8_t jtag_read_input(void)
{
    return ((JTAG_IN_1>>JTAG_PIN_TDO)&1)|(((JTAG_IN_2>>JTAG_PIN_EMU)&1)<<1);
} 

//! set pin TRST 
void jtag_set_trst(uint8_t trst)
{
  JTAG_OUT_2 = (JTAG_OUT_2&(~(1<<JTAG_PIN_TRST)))|(trst<<JTAG_PIN_TRST);
} 

//! set pin SRST 
void jtag_set_srst(uint8_t srst)
{
  JTAG_OUT_2 = (JTAG_OUT_2&(~(1<<JTAG_PIN_SRST))) |(srst<<JTAG_PIN_SRST);
} 

//! set both srst and trst simultaneously
void jtag_set_trst_srst(uint8_t trst,uint8_t srst)
{
  JTAG_OUT_2 = (JTAG_OUT_2&(~ ((1<<JTAG_PIN_SRST)|(1<<JTAG_PIN_TRST)) ))| 
           (srst<<JTAG_PIN_SRST)|(trst<<JTAG_PIN_TRST);
}

uint16_t jtag_process_data(uint8_t *dataFromHost, uint16_t dataFromHostSize, uint8_t *dataToHost, uint16_t jtagDelay)
{
  uint16_t dataToHostSize = 0;
  if(dataFromHostSize>0) {        
    //first byte is always the command
    dataFromHostSize--;
    
    switch(dataFromHost[0] & JTAG_CMD_MASK) 
    {      
    case JTAG_CMD_TAP_OUTPUT:
      
      dataFromHostSize*=4;

      if( dataFromHost[0] & JTAG_DATA_MASK )
        dataFromHostSize-= (4- ((dataFromHost[0] & JTAG_DATA_MASK)>>4));
      if(jtagDelay)
        dataToHostSize= jtag_tap_output_with_delay( &dataFromHost[1] , dataFromHostSize, dataToHost, jtagDelay);
      else
        dataToHostSize= jtag_tap_output_max_speed( &dataFromHost[1] , dataFromHostSize, dataToHost);
      break;
      
    case JTAG_CMD_TAP_OUTPUT_EMU:
      dataFromHostSize*=4;
      if(dataFromHost[0]&JTAG_DATA_MASK)
        dataFromHostSize-=(4- ((dataFromHost[0]&JTAG_DATA_MASK)>>4));
      
      dataToHostSize=jtag_tap_output_emu(&dataFromHost[1], dataFromHostSize, dataToHost, jtagDelay);
      
      break;
      
    case JTAG_CMD_READ_INPUT:
      dataToHost[0]=jtag_read_input();
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_SRST:
      jtag_set_srst(dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_TRST:
      jtag_set_trst(dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_DELAY:
      jtagDelay=dataFromHost[1]*256;
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;

    case JTAG_CMD_SET_SRST_TRST:
      jtag_set_trst_srst(dataFromHost[1]&2?1:0,dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
    
    default: //REPORT ERROR?
      break;
    }
  }

  return dataToHostSize;
}

//License: see License.txt

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include "gpio.h"

 #define CURSOR_HOME   		  0x02
 #define CMD_CLEAR_DISPLAY    0x01
 #define ENTRY_MODE_SET       0x06
 #define DISPLAY_ON           0x0C 
 #define FUNCTION_8bit_MODE   0x30 
 #define FUNCTION_4bit_MODE   0x29
 #if (Lcd_Mode == 4)
  #define FUNCTION_NORM_MODE  0x28 
 #else
  #define FUNCTION_NORM_MODE  0x38 
 #endif
 #define BIAS_SET             0x14 
 #define POWER		          0x5E 
 #define FOLLOWER_CTRL        0x6A 
 #define CONTRAST_SET         0x78 


#if (Lcd_Mode == 4)
 #define Lcd_Attatch			Lcd_D4_DDR = DataDirectionOutput; Lcd_D5_DDR = DataDirectionOutput; Lcd_D6_DDR = DataDirectionOutput; Lcd_D7_DDR = DataDirectionOutput;
 #define Lcd_Detatch			Lcd_D4_DDR = DataDirectionInput; Lcd_D5_DDR = DataDirectionInput; Lcd_D6_DDR = DataDirectionInput; Lcd_D7_DDR = DataDirectionInput;
 #define Lcd_ClearPort			Lcd_D4 = 0; Lcd_D5 = 0; Lcd_D6 = 0; Lcd_D7 = 0;
#else
 #define Lcd_Attatch			Lcd_D0_DDR = DataDirectionOutput; Lcd_D1_DDR = DataDirectionOutput; Lcd_D2_DDR = DataDirectionOutput; Lcd_D3_DDR = DataDirectionOutput; Lcd_D4_DDR = DataDirectionOutput; Lcd_D5_DDR = DataDirectionOutput; Lcd_D6_DDR = DataDirectionOutput; Lcd_D7_DDR = DataDirectionOutput;
 #define Lcd_Detatch			Lcd_D0_DDR = DataDirectionInput; Lcd_D1_DDR = DataDirectionInput; Lcd_D2_DDR = DataDirectionInput; Lcd_D3_DDR = DataDirectionInput; Lcd_D4_DDR = DataDirectionInput; Lcd_D5_DDR = DataDirectionInput; Lcd_D6_DDR = DataDirectionInput; Lcd_D7_DDR = DataDirectionInput;
 #define Lcd_ClearPort			Lcd_D0 = 0; Lcd_D1 = 0; Lcd_D2 = 0; Lcd_D3 = 0; Lcd_D4 = 0; Lcd_D5 = 0; Lcd_D6 = 0; Lcd_D7 = 0;
#endif

#define Lcd_ToggleE				Lcd_E = 1;asm volatile ("nop");Lcd_E = 0;
#define Lcd_WriteDelay 42

void Lcd_write(unsigned char cmd ,unsigned char data) 
{
  if (cmd == 1)
    Lcd_RS = 0;
  else
    Lcd_RS = 1;
  Lcd_E = 0;
#if (Lcd_Mode == 8)
  Lcd_ClearPort;
  if(data & 0x80) Lcd_D7 = 1;
  if(data & 0x40) Lcd_D6 = 1;
  if(data & 0x20) Lcd_D5 = 1;
  if(data & 0x10) Lcd_D4 = 1;
  if(data & 0x08) Lcd_D3 = 1;
  if(data & 0x04) Lcd_D2 = 1;
  if(data & 0x02) Lcd_D1 = 1;
  if(data & 0x01) Lcd_D0 = 1;
  Lcd_ToggleE;
#else
  unsigned char i;
  for (i=0;i<2;i++)
    {
	  if (i==1)
	    data <<= 4;
      Lcd_ClearPort;
      if(data & 0x80) Lcd_D7 = 1;
      if(data & 0x40) Lcd_D6 = 1;
      if(data & 0x20) Lcd_D5 = 1;
      if(data & 0x10) Lcd_D4 = 1;
      Lcd_ToggleE;
	}
#endif
 #ifndef Lcd_HasRWPin
  _delay_us(Lcd_WriteDelay);
 #else
  Lcd_busy();
 #endif
}

void Lcd_clr(void)
{
  Lcd_write(1,CMD_CLEAR_DISPLAY);
  _delay_ms(2);
}

void Lcd_init(void)
{
  unsigned char i;
  //delay after reset
  _delay_ms(15) ; 
  // RS , RW , E configured as  Output
  Lcd_E_DDR = 1;
  Lcd_RS_DDR = 1;
 #ifdef LCD_HAS_RW_PIN
  LCD_RW_DDR |=  (1<<LCD_RW) ;
  LCD_RW_LOW ;
 #endif
  Lcd_RS = 0;
  Lcd_Attatch;
  Lcd_ClearPort;
  Lcd_D5 = 1;
  Lcd_D4 = 1;
  for (i=0;i<3;i++)
    {
      Lcd_ToggleE;
      _delay_ms(5) ;
	}
 #if (Lcd_Mode == 4)
  Lcd_write(1, FUNCTION_4bit_MODE );
 #endif
 #if (Lcd_Chipset == CHIPSET_ST7036)
  Lcd_write(1, BIAS_SET );	
  Lcd_write(1, POWER );	
  Lcd_write(1, FOLLOWER_CTRL );	
  Lcd_write(1, CONTRAST_SET );	
 #endif
  Lcd_write(1, FUNCTION_NORM_MODE);	
  Lcd_write(1, ENTRY_MODE_SET);
  Lcd_write(1, DISPLAY_ON);	
  Lcd_write(1, CMD_CLEAR_DISPLAY);
  _delay_ms(2);
}

void Lcd_gotoxy(unsigned char x , unsigned char y)
{
  if (y < 2)
    Lcd_write(1,Lcd_RamStartAddr+(y*Lcd_RowLength)+x);
  else
    Lcd_write(1,Lcd_RamStartAddr+((y-2)*Lcd_RowLength)+x+0x10);
}

void Lcd_print(char *string)
{
  while(*string) 
    Lcd_write(0,*string++);
}

void Lcd_printP(char *string)
{
  while(pgm_read_byte(string)) 
    Lcd_write(0,pgm_read_byte(string++));
}

#ifdef LCD_HAS_RW_PIN
char Lcd_busy(void)
{	
  LCD_CLEAR_PORT ;
  LCD_DATA_IN; 
  LCD_RS_LOW;
  LCD_RW_HIGH;
  LCD_E_HIGH;
  while( ( LCD_DATAPIN & (1<<LCD_D7) ));
  LCD_E_LOW;	
  LCD_RW_LOW;
  LCD_DATA_OUT;
  return 1; 
}
#endif

void Lcd_custom_char(char chr,char *data)
{
  Lcd_write(1,0x40+(chr*8));
  unsigned char i;
  for (i=0;i<8;i++)
    {
	  Lcd_write(0,*data++);
	}
  Lcd_clr();
}

void Lcd_custom_charP(char chr,char *data)
{
  Lcd_write(1,0x40+(chr*8));
  unsigned char i;
  for (i=0;i<8;i++)
    {
	  Lcd_write(0,pgm_read_byte(data++));
	}
  Lcd_clr();
}

const TLcd Lcd = {Lcd_init,Lcd_print,Lcd_printP,Lcd_gotoxy,Lcd_clr,Lcd_custom_char,Lcd_custom_charP};


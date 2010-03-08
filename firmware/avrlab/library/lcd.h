//License: see License.txt

#ifndef __EBLibLcd_H_
 #define __EBLibLcd_H_

/** \file */
/** \~english \defgroup Lcd Lcd - Lcd control routines 
    \~german  \defgroup Lcd Lcd - Routinen um LC-Displays anzusteuern
    \code #include "lcd.h" \endcode 
    \~english This is an Interface to LC-Displays<br>Example:
    \~german Ein Interface um LC-Displays anzusteuern<br>Beispiel:
    \code 
      #include "lcd.h"  
      
      int main(void)
      {
        Lcd.Init();
        Lcd.Print("Hello World !");  
        
        Lcd.GotoXY(0,1);
        Lcd.Print("Here I am !");
      } 
    \endcode
    
    */

#include <avr/io.h>

//*PROPERTY LCD.Chipset ENUM replace(CHIPSET_,)
#define CHIPSET_HD44780 1
//PROPERTY LCD.Chipset ENUM replace(CHIPSET_,)
#define CHIPSET_ST7036  2

//PROPERTY LCD.Chipset ENUM
#define Lcd_Chipset CHIPSET_HD44780

//PROPERTY LCD.Mode ENUM(4,8)
#define Lcd_Mode 8//Bit

//PROPERTY LCD.CGRamStartAddr INTEGER_HEX
#define Lcd_CGRamStartAddr	0x40 
//PROPERTY LCD.RamStartAddr INTEGER_HEX
#define Lcd_RamStartAddr	0x80
//PROPERTY LCD.RowLength INTEGER
#define Lcd_RowLength		16

//#define Lcd_HasRWPin
//PROPERTY LCD.D0 PORTPIN
#define Lcd_D0				PortB.Output.Bit4
//PROPERTY LCD.D0 PORTPIN replace(Output,DataDirection)
#define Lcd_D0_DDR			PortA.DataDirection.Bit4
//PROPERTY LCD.D1 PORTPIN
#define Lcd_D1				PortA.Output.Bit5
//PROPERTY LCD.D1 PORTPIN replace(Output,DataDirection)
#define Lcd_D1_DDR			PortA.DataDirection.Bit5
//PROPERTY LCD.D2 PORTPIN
#define Lcd_D2				PortA.Output.Bit6
//PROPERTY LCD.D2 PORTPIN replace(Output,DataDirection)
#define Lcd_D2_DDR			PortA.DataDirection.Bit6
//PROPERTY LCD.D3 PORTPIN
#define Lcd_D3				PortA.Output.Bit7
//PROPERTY LCD.D3 PORTPIN replace(Output,DataDirection)
#define Lcd_D3_DDR			PortA.DataDirection.Bit7

//PROPERTY LCD.D4 PORTPIN
#define Lcd_D4				PortA.Output.Bit0
//PROPERTY LCD.D4 PORTPIN replace(Output,DataDirection)
#define Lcd_D4_DDR			PortA.DataDirection.Bit0
#define Lcd_D5				PortA.Output.Bit1
#define Lcd_D5_DDR			PortA.DataDirection.Bit1
#define Lcd_D6				PortA.Output.Bit2
#define Lcd_D6_DDR			PortA.DataDirection.Bit2
#define Lcd_D7				PortA.Output.Bit3
#define Lcd_D7_DDR			PortA.DataDirection.Bit3

#define Lcd_RW				PortB.Input.Bit0
#define Lcd_E				PortB.Output.Bit7
#define Lcd_E_DDR			PortB.DataDirection.Bit7
#define Lcd_RS				PortB.Output.Bit4
#define Lcd_RS_DDR			PortB.DataDirection.Bit4

/** 
   \class TLcd
   \brief Lcd
   \~english This structure represents the LC-Display 
   \~german über diese Struktur kann auf ein LC-Display zugegriffen werden 
*/
typedef struct
{
  void (*Init)(void);
  void (*Print)(char *string);
  void (*PrintP)(char *string);
  void (*GotoXY)(unsigned char x , unsigned char y);
  void (*Clear)(void);
  void (*CustomChar)(char Character,char *data);
  void (*CustomCharP)(char Character,char *data);
} TLcd;

extern const TLcd Lcd;

#endif


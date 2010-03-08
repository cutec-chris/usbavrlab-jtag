//License: see License.txt

#ifndef __EBLibGPIO_H_
  #define __EBLibGPIO_H_

  #include <avr/io.h>

//Suppurted Devices:
//All AVR Controller

/** \file */
/** \~english \defgroup GPIO GPIO - General Purpose I/O 
    \~german  \defgroup GPIO GPIO - I/O Pin spezifisches
    \code #include "gpio.h" \endcode
    \~german Dieses Modul ermöglicht es, einfach und Plattformübergreifend auf I/O Pins zuzugreifen.<br>
    Beispiel:
    \~english This Module makes it possible, to access I/O Pins easy and plattform independent.<br>
    Example: 
    \~
    \code
    #include "gpio.h"
    
    int main(void)
    {
      PortA.DataDirection.Bit1 = DataDirectionOutput;
      PortA.DataDirection.Bit0 = DataDirectionInput;
                   
      while (1)
        { 
          if (PortA.Input.Bit0)
            PortA.Output.Bit1 = 1;
        }        
    }    
    \endcode 
    */

#define DataDirectionOutput 1
#define DataDirectionInput 0

/** 
   \class TPort
   \brief Port
   \~english This structure represents an I/O Port 
   \~german Mithilfe dieser Struktur kann vereinfacht auf I/O Ports zugegriffen werden
*/
typedef union
{
  struct 
    {
      struct 
	    {
        unsigned char Bit0 : 1;
        unsigned char Bit1 : 1;
        unsigned char Bit2 : 1;
        unsigned char Bit3 : 1;
        unsigned char Bit4 : 1;
        unsigned char Bit5 : 1;
        unsigned char Bit6 : 1;
        unsigned char Bit7 : 1;
        } Input;
      struct {
        unsigned char Bit0 : 1;
        unsigned char Bit1 : 1;
        unsigned char Bit2 : 1;
        unsigned char Bit3 : 1;
        unsigned char Bit4 : 1;
        unsigned char Bit5 : 1;
        unsigned char Bit6 : 1;
        unsigned char Bit7 : 1;
      } DataDirection;
      struct {
        unsigned char Bit0 : 1;
        unsigned char Bit1 : 1;
        unsigned char Bit2 : 1;
        unsigned char Bit3 : 1;
        unsigned char Bit4 : 1;
        unsigned char Bit5 : 1;
        unsigned char Bit6 : 1;
        unsigned char Bit7 : 1;
      } Output;
    };
  struct 
    {
	  unsigned char Input;
	  unsigned char DataDirection;
	  unsigned char Output;
    };
} TPort;

#define PortA (*(volatile TPort *)&PINA)
#define PortB (*(volatile TPort *)&PINB)
#define PortC (*(volatile TPort *)&PINC)
#define PortD (*(volatile TPort *)&PIND)
#define PortE (*(volatile TPort *)&PINE)
#define PortF (*(volatile TPort *)&PINF)
#define PortG (*(volatile TPort *)&PING)
#define PortH (*(volatile TPort *)&PINH)

#endif

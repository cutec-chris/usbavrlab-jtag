//License: see License.txt

#include <avr/io.h>
#ifndef __EBLibADC_H_
  #define __EBLibADC_H_

//Supported Devices
// Atmega8
// Atmega48
// Atmega88
// Atmega168
// Atmega328

// Atmega16(A)
// Atmega32(A)

// Atmega64

/** \file */
/** \~english \defgroup ADC ADC - Analog to Digital Converter 
    \~german  \defgroup ADC ADC - Analog zu Digital Wandler
    \code #include "adc.h" \endcode
    \~english With an ADC you can digitize Analog Signals<br>Example:
    \~german Mit einem ADC kann man analoge Signale digitalisieren<br>Beispiel:
    \code 
      #include "adc.h"  
      
      int main(void)
      {          
        Adc.Enabled = 1;      
        Adc.Prescaler = 3;
        Adc.Reference = 3;
        Adc.StartConversation = 1;
        while (Adc.StartConversation);
        
        unsigned int MyValue = Adc.Value; 
      } 
    \endcode
    */

#if (_AVR_IOM8_H_)|(_AVR_IOM16_H_)|(_AVR_IOM32_H_)|(_AVR_IOM64_H_)

/** 
   \class TAdc
   \brief Adc
   \~english This structure represents the Adc registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die ADC register zugegriffen werden
*/
typedef union
{
  struct {
  /*! \~english The ADC Value after the conversation(s) are done. */
  /*! \~german Der gemessene ADC Wert nach einer oder mehreren Konversationen. */
  unsigned int Value;

  /*! \~english The Prescaler Bits */
  /*! \~german Die Vorteiler Bits */
  unsigned char Prescaler : 3;
  /*! \~english This Flag should be set to Enable the ADC conversation complete interrupt */
  unsigned char InterruptEnable : 1;
  /*! \~english This Flag will be 1 after an Interrupt occours */
  unsigned char InterruptFlag : 1;
#if (_AVR_IOM16_H_)|(_AVR_IOM32_H_)
  /*! \~english This Flag should be set to enable the Auto Trigger */
  unsigned char AutoTriggerEnabled : 1;
#else
  /*! \~english This Flag should be set to enable the Free Running Mode */
  unsigned char FreeRunning : 1;
#endif
  /*! \~english Write this Flag to 1 to start the conversation(s) */
  unsigned char StartConversation : 1;
  /*! \~english This Flag should be 1 to enable the ADC */
  unsigned char Enabled : 1;

  /*! \~english This property selects the ADC Channel */
  unsigned char Mux : 4;
  unsigned char unused0 : 1;
  /*! \~english Write this Flag to 1 to use an LefttoRight Alignment in Result */
  unsigned char LeftAdjustResult : 1;
  /*! \~english This property selects the reference to use */
  unsigned char Reference : 2;
#if (_AVR_IOM64_H_)
  unsigned char unused1[0x66];
  unsigned char TriggerSource : 3;
#endif
  };

  struct {
  unsigned char adcl;
  unsigned char adch;  
  unsigned char adcsr;
  unsigned char admux;
#if (_AVR_IOM64_H_)
  unsigned char unused0[134];
  unsigned char adsrb;
#endif
  } raw;
} TAdc;
#endif

#if (_AVR_IOMX8_H_)
typedef union
{
  struct {
  unsigned int Value;

  unsigned char Prescaler : 3;
  unsigned char InterruptEnable : 1;
  unsigned char InterruptFlag : 1;
  unsigned char AutoTriggerEnabled : 1;
  unsigned char StartConversation : 1;
  unsigned char Enabled : 1;

  unsigned char TriggerSource : 3;
  unsigned char unused1 : 3;
  unsigned char acme : 1;
  unsigned char unused2 : 1;

  unsigned char Mux : 4;
  unsigned char unused0 : 1;
  unsigned char LeftAdjustResult : 1;
  unsigned char Reference : 2;
  };

  struct {
  unsigned char adcl;
  unsigned char adch;  
  unsigned char adcsra;
  unsigned char adcsrb;
  unsigned char admux;
  } raw;

} TAdc;
#endif


#define Adc (*(volatile TAdc *)&ADC)

#endif

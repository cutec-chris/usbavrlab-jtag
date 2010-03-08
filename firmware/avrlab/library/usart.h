//License: see License.txt

#ifndef __EBLibUSART_H_
  #define __EBLibIUSART_H_

#include <avr/io.h>

//*****************************
// USART Registers
//*****************************
//Supported Devices:
// Atmega8515 - OK
// Atmega8
// Atmega64

/** \file */
/** \~english \defgroup UART UART - Usart interface 
    \~german  \defgroup UART UART - Serielle Schnittstellen
    \code #include "usart.h" \endcode
    \~english With UART you can communicate with PC´s or other devices that uses an USART<br>Example:
    \~german Mit einem UART können Sie mit PC´s oder anderen Geräten kommunizieren<br>Beispiel:
    \code 
      #include "usart.h"  
          
      int main(void)
      {
        Uart.(..)          
      } 
    \endcode
    */

/** 
   \class TUartControl
   \brief UartControl
   \~english This structure represents the Uart registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die Uart register zugegriffen werden
*/
typedef union
{
  struct
  {
    //USART Baud Rate Register - UBRRL
    /*! \~english This is a part of a 12-bit register which contains the USART baud rate.*/
    /*! \~german  Das ist ein Teil eines 12-Bit Registers, dass die USART Baudrate enthält.*/
    unsigned char BaudRateLow : 8;//UBRRL
    //USART Control and Status Register B - UCRSB
    /*! \~english TXB8 is the ninth data bit in the character to be transmitted when operating with serial frames with 9 data bits. Must be written before writing the low bits to UDR.*/
    /*! \~german  TXB8 ist das neunte Daten Bit in einem zu sendenem Zeichen, wenn der 9 Bit Datenmodus eingestellt ist. Muss vor dem Schreiben der niedrigen Bits ins UDR geschrieben werden.*/
    unsigned char TransmitDataBit8 : 1;//TXB8
    /*! \~english RXB8 is the ninth data bit of the received character when operating with serial frames with nine data bits. Must be read before reading the low bits from UDR.*/
    /*! \~german  RXB8 ist das neunte Daten Bit in einem zu lesendem Zeichen, wenn der 9 Bit Datenmodus eingestellt ist. Muss vor dem Lesen der niedrigen Bits ins UDR geschrieben werden*/
    unsigned char ReceiveDataBit8 : 1;//RXB8
    /*! \~english The UCSZ2 bits combined with the UCSZ1:0 bit in UCSRC sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
    /*! \~german  Das UCSZ2 Bit */
    unsigned char CharacterSize : 1;//UCSZ2
    /*! \~english The UCSZ2 bits combined with the UCSZ1:0 bit in UCSRC sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
    /*! \~german  Das UCSZ2 Bit, das mit dem UCSZ1:0-Bit in UCSRC verbunden ist, setzen die Anzahl von Datenbits (Zeichengröße) in einem Rahmen der Empfänger und Sender benötigt.*/
    unsigned char TransmitterEnabled : 1;//TXEN
    /*! \~english Writing this bit to one enables the USART Transmitter.*/
    /*! \~german  Wenn diese Bit 1 ist, wird der USART Sender gestartet.*/
    unsigned char ReceiverEnabled : 1;//RXEN
    /*! \~english Writing this bit to one enables the USART Receiver.*/
    /*! \~german  Wenn diese Bit 1 ist, wird der USART Empfänger gestartet.*/
    unsigned char DataRegisterEmptyInterruptEnable : 1;//UDRIE
    /*! \~english Writing this bit to one enables interrupt on the UDRE Flag. A Data Register Empty interrupt will be generated only if the UDRIE bit is written to one, the Global Interrupt Flag in SREG is written to one and the UDRE bit in UCSRA is set.*/
    /*! \~german  Sollte dieses Bit 1 sein wird der Interrupt des UDRE Flag aktiv. Ein Data Register Empty interupt wird nur erzeugt, wenn das UDRIE-Bit gesetzt wird der, das Global interrupt Flag wird im  SREG gesetzt, und das UDRE-Bit wird in UCSRA gesetzt.*/
    unsigned char TXCompleteInterruptEnable : 1;//TXCIE
    /*! \~english Writing this bit to one enables interrupt on the TXC Flag. A USART Transmit Complete interrupt will be generated only if the TXCIE bit is written to one, the Global Interrupt Flag in SREG is written to one and the TXC bit in UCSRA is set.*/
    /*! \~german  Sollte dieses Bit 1 sein, wir der Interrupt des TXC Flag aktiviert. Ist eine USART Übertragung abgeschlossen, wird nur der Interrupt generiert wenn das TXCIE Bit 1 ist. Das Global interrupt Flag wird im SREG auf 1 geschrieben und das TXC Bit wird in UCSRA gesetzt*/
    unsigned char RXCompleteInterruptEnable : 1;//RXCIE
    //USART Control and Status Register A - UCRSA
    /*! \~english This bit enables the Multi-processor Communication mode. When the MPCM bit is written to one, all the incoming frames received by the USART Receiv that do not contain address information will be ignored.*/
    /*! \~german  Dieses Bin aktiviert den Multi-Processor Communikation Mode. Wenn das MPCN Bit auf 1 gestezt ist, werden alle eingehenden Frames vom USART Empgänger, die keine Addressinformationen enthalten, ignoriert.*/
    unsigned char MultiProcessorCommunicationMode : 1;//MPCM
    /*! \~english This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.*/
    /*! \~german  Dieses Bit hat nur eine Auswirkung auf den asynchronen Betrieb. Dieses Bit ist 1 wenn man die synchrone Betriebsweise benutzt.*/
    unsigned char DoubleTransmissionSpeed : 1;//U2X
    /*! \~english This bit is set if the next character in the receive buffer had a Parity Error.*/
    /*! \~german  Dieses Bit ist gesetzt wenn das Folgende Zeichen im Empfängerspeicher einen Paritätsfehler aufweist. */
    unsigned char ParityError : 1;//PE
    /*! \~english This bit is set if a Data OverRun condition is detected. A Data OverRun occurs when the receive buffer is full (two characters), it is a new character waiting in the Receive Shift Register, and a new start bit is detected.*/
    /*! \~german  Dieses Bit ist gesetzt, wenn eine Data OverRun Bedingung gefunden wurde. Ein Data OverRun kommt vor, wenn der Empfängerspeicher voll ist(zwei Zeichen), dann wird das Zeichen in das Receive Shift Register geschrieben. */
    unsigned char DataOverRun : 1;//DOR
    /*! \~english This bit is set if the next character in the receive buffer had a Frame Error when received*/
    /*! \~german  Dieses Bit ist gesetzt, wenn das folgende Zeichen in dem Emfpängerspeicher einen Frame Fehler beim empfangen hat */
    unsigned char FrameError : 1;//FE
    /*! \~english The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data.*/
    /*! \~german  Das UDRE Flag zeigt an, wenn der Sendespeicher (UDR) bereit ist neue Daten zu erhalten*/
    unsigned char DataRegisterEmpty : 1;//UDRE
    /*! \~english This flag bit is set when the entire frame in the Transmit Shift Register has been shifted out and there are no new data currently present in the transmit buffer (UDR).*/
    /*! \~german  Dieses Flag ist gesetzt wenn das ganze Frame in das Transmit Shift Register ausgetausch worden ist und keine neuen daten momentan in dem Sendespeicher sind.*/
    unsigned char TransmitComplete : 1;//V
    /*! \~english This flag bit is set when there are unread data in the receive buffer and cleared when the receive buffer is empty. If the Receiver is disabled, the receive buffer will be flushed and consequently the RXC bit will become zero.*/
    /*! \~german  Dieses Flag ist gesetzt, wenn drei ungelesene Daten in dem Empfängerspeicher sind und wenn er leer ist. Wenn der Empfänger abgeschaltet ist, wird der Emfpängerspeicher geflushed und das RXC Bit wird immer null*/
    unsigned char ReceiveComplete : 1;//RXC:
    //USART I/O Data Register - UDR
    /*! \~english The USART Transmit Data Buffer Register and USART Receive Data Buffer Registers share the same I/O address.*/
    /*! \~german  Der USART sende Datenspeicher und der USART empfangs Datenspeicher teilen sich die gleiche I/O Addesse.*/
    union
	{
	  unsigned char Data;
	  unsigned char ReceiveDataBuffer;//RXB
	  unsigned char TransmitDataBuffer;//TXB
	};
    //USART Control and Status Register C - UCSRC **and** USART Baud Rate Registers - UBRRH
#if (_AVR_IOM64_H_)
    unsigned char unused0[99];
#else
    unsigned char unused0[19];
#endif    
    union
    {
      struct
	  {
	    /*! \~english This bit is used for Synchronous mode only. Write this bit to zero when Asynchronous mode is used. The UCPOL bit sets the relationship between data output change and data input sample, and the synchronous clock (XCK).*/
        /*! \~german  Dieses Bit wird nur für den sychronen Betrieb benutzt. Das bit muss 1 im asynchronen Betrieb sein. Das UCPOL Bit setzt die Beziehung zwischen Datenausgang, Dateneingang und dem Synchronisierungstak (XCK)*/
	    unsigned char ClockPolarity : 1;//UCPOL
		/*! \~english The UCSZ bits combined with the UCSZ2 bit in UCSRB sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
        /*! \~german  Das UCSZ0:1 Bit vereint mit dem UCSZ2 Bit im UCSRB setzten die Nummer von Datenbits (character size) in einem Berech den Empfänger und Sender benötigen.*/ 
        unsigned char CharacterSize : 2;//UCSZ
		/*! \~english This bit selects the number of stop bits to be inserted by the Transmitter. The Receiver ignores this setting.*/
        /*! \~german  Dieser Wert setzt die Anzahl von Stop Bits, die der Sender überträgt. Der Empfänger ignoriert diese EInstellung*/
        unsigned char StopBitSelect : 1;//USBS
		/*! \~english These bits enable and set type of parity generation and check. If enabled, the Transmitter will automatically generate and send the parity of the transmitted data bits within each frame. The Receiver will generate a parity value for the incoming data and compare it to the UPM0 setting. If a mismatch is detected, the PE Flag in UCSRA will be set.*/
        /*! \~german  DIeses Bit startet, generiert und prüft die Parität. Wenn sie eingeschaltet ist, generiert und übertärgt die Sendeeinheit automatisch die Parität. Der Empfänger generiert ebelfalls aus den eingehenden Daten einen Patitätswert und vergleicht diesen mit den UPM0 Einstellungen. Wenn diese nicht übereinstimmen, wird das PE Flag im UCSRA gesetzt. */
        unsigned char ParityMode : 2;//UPM
		/*! \~english This bit selects between asynchronous and synchronous mode of operation.*/
        /*! \~german  Mit diesem Wert wählt man zwischen asynchronen und synchronen Modus.*/
        unsigned char ModeSelect : 1;//UMSEL
		/*! \~english This bit selects between accessing the UCSRC or the UBRRH Register. It is read as one when reading UCSRC. The URSEL must be one when writing the UCSRC.*/
        /*! \~german  Das Bit organisiert den Zugriff auf das UCSRS oder uaf das UBRRH Register. Es wird als eins gelesen, wenn UCSRC ausgelesen werden soll. Es muss 1 sein, wenn UCSRC geschrieben wird.*/
        unsigned char RegisterSelect : 1;//URSEL
	  };
	  /*! \~english This is a register which contains the USART baud rate*/
      /*! \~german  Das ist ein Register, welches die Baud Rate enthält*/
	  unsigned char BaudRateHigh;//UBRRH
    };
  };
} TUartControl;

#if (_AVR_IOM8515_H_) || (_AVR_IOM8_H_)
 #define UsartControl  (*(volatile TUartControl *)&UBRRL)
 #define Usart0Control (*(volatile TUartControl *)&UBRRL)
#elif (_AVR_IOM64_H_)
 #define UsartControl  (*(volatile TUartControl *)&UBRR0L)
 #define Usart0Control (*(volatile TUartControl *)&UBRR0L)
#endif

typedef union
{
  struct
  {
    //USART Baud Rate Register - UBRRL
    /*! \~english This is a part of a 12-bit register which contains the USART baud rate.*/
    /*! \~german  Das ist ein Teil eines 12-Bit Registers, dass die USART Baudrate enthält.*/
    unsigned char BaudRateLow;//UBRRL
    unsigned char BaudRateHigh;//UBRRH
    //USART Control and Status Register B - UCRSB
    /*! \~english TXB8 is the ninth data bit in the character to be transmitted when operating with serial frames with 9 data bits. Must be written before writing the low bits to UDR.*/
    /*! \~german  TXB8 ist das neunte Daten Bit in einem zu sendenem Zeichen, wenn der 9 Bit Datenmodus eingestellt ist. Muss vor dem Schreiben der niedrigen Bits ins UDR geschrieben werden.*/
    unsigned char TransmitDataBit8 : 1;//TXB8
    /*! \~english RXB8 is the ninth data bit of the received character when operating with serial frames with nine data bits. Must be read before reading the low bits from UDR.*/
    /*! \~german  RXB8 ist das neunte Daten Bit in einem zu lesendem Zeichen, wenn der 9 Bit Datenmodus eingestellt ist. Muss vor dem Lesen der niedrigen Bits ins UDR geschrieben werden*/
    unsigned char ReceiveDataBit8 : 1;//RXB8
    /*! \~english The UCSZ2 bits combined with the UCSZ1:0 bit in UCSRC sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
    /*! \~german  Das UCSZ2 Bit */
    unsigned char CharacterSize2 : 1;//UCSZ2
    /*! \~english The UCSZ2 bits combined with the UCSZ1:0 bit in UCSRC sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
    /*! \~german  Das UCSZ2 Bit, das mit dem UCSZ1:0-Bit in UCSRC verbunden ist, setzen die Anzahl von Datenbits (Zeichengröße) in einem Rahmen der Empfänger und Sender benötigt.*/
    unsigned char TransmitterEnabled : 1;//TXEN
    /*! \~english Writing this bit to one enables the USART Transmitter.*/
    /*! \~german  Wenn diese Bit 1 ist, wird der USART Sender gestartet.*/
    unsigned char ReceiverEnabled : 1;//RXEN
    /*! \~english Writing this bit to one enables the USART Receiver.*/
    /*! \~german  Wenn diese Bit 1 ist, wird der USART Empfänger gestartet.*/
    unsigned char DataRegisterEmptyInterruptEnable : 1;//UDRIE
    /*! \~english Writing this bit to one enables interrupt on the UDRE Flag. A Data Register Empty interrupt will be generated only if the UDRIE bit is written to one, the Global Interrupt Flag in SREG is written to one and the UDRE bit in UCSRA is set.*/
    /*! \~german  Sollte dieses Bit 1 sein wird der Interrupt des UDRE Flag aktiv. Ein Data Register Empty interupt wird nur erzeugt, wenn das UDRIE-Bit gesetzt wird der, das Global interrupt Flag wird im  SREG gesetzt, und das UDRE-Bit wird in UCSRA gesetzt.*/
    unsigned char TXCompleteInterruptEnable : 1;//TXCIE
    /*! \~english Writing this bit to one enables interrupt on the TXC Flag. A USART Transmit Complete interrupt will be generated only if the TXCIE bit is written to one, the Global Interrupt Flag in SREG is written to one and the TXC bit in UCSRA is set.*/
    /*! \~german  Sollte dieses Bit 1 sein, wir der Interrupt des TXC Flag aktiviert. Ist eine USART Übertragung abgeschlossen, wird nur der Interrupt generiert wenn das TXCIE Bit 1 ist. Das Global interrupt Flag wird im SREG auf 1 geschrieben und das TXC Bit wird in UCSRA gesetzt*/
    unsigned char RXCompleteInterruptEnable : 1;//RXCIE
    //USART Control and Status Register A - UCRSA
    /*! \~english This bit enables the Multi-processor Communication mode. When the MPCM bit is written to one, all the incoming frames received by the USART Receiv that do not contain address information will be ignored.*/
    /*! \~german  Dieses Bin aktiviert den Multi-Processor Communikation Mode. Wenn das MPCN Bit auf 1 gestezt ist, werden alle eingehenden Frames vom USART Empgänger, die keine Addressinformationen enthalten, ignoriert.*/
    unsigned char MultiProcessorCommunicationMode : 1;//MPCM
    /*! \~english This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.*/
    /*! \~german  Dieses Bit hat nur eine Auswirkung auf den asynchronen Betrieb. Dieses Bit ist 1 wenn man die synchrone Betriebsweise benutzt.*/
    unsigned char DoubleTransmissionSpeed : 1;//U2X
    /*! \~english This bit is set if the next character in the receive buffer had a Parity Error.*/
    /*! \~german  Dieses Bit ist gesetzt wenn das Folgende Zeichen im Empfängerspeicher einen Paritätsfehler aufweist. */
    unsigned char ParityError : 1;//PE
    /*! \~english This bit is set if a Data OverRun condition is detected. A Data OverRun occurs when the receive buffer is full (two characters), it is a new character waiting in the Receive Shift Register, and a new start bit is detected.*/
    /*! \~german  Dieses Bit ist gesetzt, wenn eine Data OverRun Bedingung gefunden wurde. Ein Data OverRun kommt vor, wenn der Empfängerspeicher voll ist(zwei Zeichen), dann wird das Zeichen in das Receive Shift Register geschrieben. */
    unsigned char DataOverRun : 1;//DOR
    /*! \~english This bit is set if the next character in the receive buffer had a Frame Error when received*/
    /*! \~german  Dieses Bit ist gesetzt, wenn das folgende Zeichen in dem Emfpängerspeicher einen Frame Fehler beim empfangen hat */
    unsigned char FrameError : 1;//FE
    /*! \~english The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data.*/
    /*! \~german  Das UDRE Flag zeigt an, wenn der Sendespeicher (UDR) bereit ist neue Daten zu erhalten*/
    unsigned char DataRegisterEmpty : 1;//UDRE
    /*! \~english This flag bit is set when the entire frame in the Transmit Shift Register has been shifted out and there are no new data currently present in the transmit buffer (UDR).*/
    /*! \~german  Dieses Flag ist gesetzt wenn das ganze Frame in das Transmit Shift Register ausgetausch worden ist und keine neuen daten momentan in dem Sendespeicher sind.*/
    unsigned char TransmitComplete : 1;//V
    /*! \~english This flag bit is set when there are unread data in the receive buffer and cleared when the receive buffer is empty. If the Receiver is disabled, the receive buffer will be flushed and consequently the RXC bit will become zero.*/
    /*! \~german  Dieses Flag ist gesetzt, wenn drei ungelesene Daten in dem Empfängerspeicher sind und wenn er leer ist. Wenn der Empfänger abgeschaltet ist, wird der Emfpängerspeicher geflushed und das RXC Bit wird immer null*/
    unsigned char ReceiveComplete : 1;//RXC:
    //USART I/O Data Register - UDR
    /*! \~english The USART Transmit Data Buffer Register and USART Receive Data Buffer Registers share the same I/O address.*/
    /*! \~german  Der USART sende Datenspeicher und der USART empfangs Datenspeicher teilen sich die gleiche I/O Addesse.*/
    union
	{
	  unsigned char Data;
	  unsigned char ReceiveDataBuffer;//RXB
	  unsigned char TransmitDataBuffer;//TXB
	};
    //USART Control and Status Register C - UCSRC **and** USART Baud Rate Registers - UBRRH
    /*! \~english This bit is used for Synchronous mode only. Write this bit to zero when Asynchronous mode is used. The UCPOL bit sets the relationship between data output change and data input sample, and the synchronous clock (XCK).*/
    /*! \~german  Dieses Bit wird nur für den sychronen Betrieb benutzt. Das bit muss 1 im asynchronen Betrieb sein. Das UCPOL Bit setzt die Beziehung zwischen Datenausgang, Dateneingang und dem Synchronisierungstak (XCK)*/
    unsigned char ClockPolarity : 1;//UCPOL
    /*! \~english The UCSZ bits combined with the UCSZ2 bit in UCSRB sets the number of data bits (character size) in a frame the Receiver and Transmitter use.*/
    /*! \~german  Das UCSZ0:1 Bit vereint mit dem UCSZ2 Bit im UCSRB setzten die Nummer von Datenbits (character size) in einem Berech den Empfänger und Sender benötigen.*/ 
    unsigned char CharacterSize01 : 2;//UCSZ
    /*! \~english This bit selects the number of stop bits to be inserted by the Transmitter. The Receiver ignores this setting.*/
    /*! \~german  Dieser Wert setzt die Anzahl von Stop Bits, die der Sender überträgt. Der Empfänger ignoriert diese EInstellung*/
    unsigned char StopBitSelect : 1;//USBS
	/* \~english These bits enable and set type of parity generation and check. If enabled, the Transmitter will automatically generate and send the parity of the transmitted data bits within each frame. The Receiver will generate a parity value for the incoming data and compare it to the UPM0 setting. If a mismatch is detected, the PE Flag in UCSRA will be set.*/
    /*! \~german  DIeses Bit startet, generiert und prüft die Parität. Wenn sie eingeschaltet ist, generiert und übertärgt die Sendeeinheit automatisch die Parität. Der Empfänger generiert ebelfalls aus den eingehenden Daten einen Patitätswert und vergleicht diesen mit den UPM0 Einstellungen. Wenn diese nicht übereinstimmen, wird das PE Flag im UCSRA gesetzt. */
    unsigned char ParityMode : 2;//UPM
	/*! \~english This bit selects between asynchronous and synchronous mode of operation.*/
    /*! \~german  Mit diesem Wert wählt man zwischen asynchronen und synchronen Modus.*/
    unsigned char ModeSelect : 1;//UMSEL
	/*! \~english This bit selects between accessing the UCSRC or the UBRRH Register. It is read as one when reading UCSRC. The URSEL must be one when writing the UCSRC.*/
    /*! \~german  Das Bit organisiert den Zugriff auf das UCSRS oder uaf das UBRRH Register. Es wird als eins gelesen, wenn UCSRC ausgelesen werden soll. Es muss 1 sein, wenn UCSRC geschrieben wird.*/
    unsigned char RegisterSelect : 1;//URSEL
  };
} TUart1Control;

#if (_AVR_IOM64_H_)
 #define Usart1Control (*(volatile TUart1Control *)&UBRR1L)
#endif

typedef struct
{
  void (*Init)(unsigned int baudrate);
  void (*Print)(char *string);
  void (*PrintP)(char *string);
} TUsart;

extern const TUsart Usart;
#define Usart0 Usart

#if (_AVR_IOM64_H_)
extern const TUsart Usart1;
#endif

#endif

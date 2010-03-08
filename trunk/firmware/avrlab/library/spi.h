//License: see License.txt

#ifndef __EBLibSPI_H_
  #define __EBLibSPI_H_

//Supported Devices:
// Atmega8515 - OK

/** \~english \defgroup SPI Spi - Serial Perephal Interface 
    \~german  \defgroup SPI Spi - Serielles Perepherie Interface
    \code #include "spi.h" \endcode
    \~english SPI is an popular Interface to many external perpherals<br>Example:
    \~german SPI ist ein belibtes Interface zu externen Perepherie Komponenten<br>Beispiel:
    \code 
      #include "spi.h"  
      
      int main(void)
      {
        Spi.ClockRate = 1;
        Spi.MasterSlaveSelect = 1; //Master
        Spi.Enabled = 1;
        Spi.Data = 0xAA;
      } 
    \endcode
    */

/** 
   \class TSpi
   \brief Spi
   \~english This structure represents the Spi registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die Spi register zugegriffen werden
*/
typedef union
{
  struct
  {
    //SPI Control Register - SPCR
    /*! \~english These two bits control the SCK rate of the device configured as a Master. SPR1 and SPR0 have no effect on the Slave*/
    /*! \~german  Diese zwei Bits steuern die SCK Übertragungsrate auf dem als Master eingestelltem Gerät. SPR1 und SPR0 haben keinen Einfluss auf den Slave*/
    unsigned char ClockRate : 2;//SPR1, SPR0
    /*! \~english The settings of the Clock Phase bit (CPHA) determine if data is sampled on the leading (first) or trailing (last) edge of SCK*/
    /*! \~german  Die Einstellungen des Clock Phase-Bit (CPHA) bestimmen, ob Daten beim Leading(zuerst) Trailing(zuletzt) in der SCK ausfallen*/
    unsigned char ClockPhase : 1;//CPHA
    /*! \~english When this bit is written to one, SCK is high when idle else SCK is low when idle.*/
    /*! \~german  Wenn dieses Bit gesetzt ist, ist die SDK High im Leerlauf, ansonsten is die SCK Low im Leerlauf */
    unsigned char ClockPolarity : 1;//CPOL
    /*! \~english This bit selects Master SPI mode when written to one*/
    /*! \~german  Das Bit selektiert den Master SPI Mode wenn es gesetzt ist*/
    unsigned char MasterSlaveSelect : 1;//MSTR:
    /*! \~english When the DORD bit is written to one, the LSB of the data word is transmitted first else the MSB of the data word is transmitted first.*/
    /*! \~german  Wenn das DORD Bit gesetzt ist wird zuerst der LSB des Datenwortes übertragen, ansonnsten wird das MSB des Datenwortes zuerst übertragen*/
    unsigned char DataOrder : 1;//DORD:
    /*! \~english This bit must be set to enable any SPI operations.*/
    /*! \~german  Dieses Bit muss gesetzt sein um jegliche SPI Operationen zu ermöglichen.*/
    unsigned char Enabled : 1;//SPE:
    /*! \~english This bit causes the SPI interrupt to be executed if SPIF bit in the SPSR Register is set and the if the Global Interrupt Enable bit in SREG is set.*/
    /*! \~german  Dieses Bit veranlasst den SPI-Interrupt, durchgeführt zu werden, wenn das SPIF-Bit im SPSR-Register gesetzt wird und wenn das Global Interrupt Enable Bit in SREG gesetzt ist*/
    unsigned char InterruptEnable : 1;//SPIE:
    //SPI Status Register - SPSR
    /*! \~english When this bit is written logic one the SPI speed (SCK Frequency) will be doubled when the SPI is in Master mode.*/
    /*! \~german  Wenn dieses Bit gesetzt wird und Master ist, wird die SPI Geschwindigkeit (SCK Frequenz) verdoppelt*/
    unsigned char DoubleSpeed : 1;//SPI2X 
    /*! \~english not used part of register*/
    /*! \~german  nicht genutzter Teil des Registers*/
    unsigned char unused : 5;
    /*! \~english The WCOL bit is set if the SPI Data Register (SPDR) is written during a data transfer.*/
    /*! \~german  Das WOCOL Bit ist gesetzt, wenn das SPI Data Register (SPDR) mittels eines Datentransfers beschrieben wird.*/
    unsigned char WriteCollisionFlag : 1;//WCOL
    /*! \~english When a serial transfer is complete, the SPIF Flag is set*/
    /*! \~german  Wenn eine serielle Datenübertragung abgeschlossen ist, ist das SPIF Flag gesetzt*/
    unsigned char InterruptOccurred : 1;//SPIF
    //SPI Data Register - SPDR
    /*! \~english */
    /*! \~german */
    unsigned char Lsb : 1;//LSB
    unsigned char unused : 6;
    /*! \~english */
    /*! \~german */
    unsigned char Msb : 1;//MSB
  };
} TSpi;

#if (_AVR_IOM8515_H_)
 #define Spi  (*(volatile TSpi *)&SPCR)
#endif

#endif

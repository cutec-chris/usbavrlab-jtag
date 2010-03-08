//License: see License.txt

#ifndef __EBLibCPU_H_
  #define __EBLibCPU_H_
//Suppurted Devices:
// Atmega64
// Atmega48
// Atmega88
// Atmega168
// Atmega328
// Atmega8515

/** \file */
/** \~english \defgroup MCU MCU - Processor specific Declarations 
    \~german  \defgroup MCU MCU - Prozessor spezifisches
    \code #include "mcu.h" \endcode
    */

/** 
   \class TMCUStatus
   \brief MCUStatus
   \~english This structure represents the MCU Status Register 
   \~german Mithilfe dieser Struktur kann vereinfacht auf das MCU Status Register zugegriffen werden
*/
typedef union
{
  struct {
  /*! \~english This Flag will be 1 after an Power On reset*/
  /*! \~german Dieses Flag ist 1 nach einem Power On Reset*/
  unsigned char PowerOnReset : 1;
  /*! \~english This Flag will be 1 after an External reset*/
  /*! \~german Dieses Flag ist 1 nach einem External Reset */
  unsigned char ExternalReset : 1;
  /*! \~english This Flag will be 1 after an Brown Out reset*/
  /*! \~german Dieses Flag ist 1 nach einem Brown Out Reset*/
  unsigned char BrownOutReset : 1;
  /*! \~english This Flag will be 1 after an Watchdog Reset */
  unsigned char WatchdogReset : 1;
#if (_AVR_IOM64_H_)
  /*! \~english This Flag will be 1 after an JTAG reset*/
  /*! \~german Dieses Flag ist 1 nach einem JTAG Reset */
  unsigned char JtagReset : 1;
  unsigned char unused0 : 2;
  /*! \~english This Flag will be 1 during JTAG debug*/
  /*! \~german Dieses Flag ist 1 bei laufendem JTAG Debug */
  unsigned char JtagDebugging : 1;
#endif
#if (_AVR_IOM8515_H_)
  unsigned char unused0 : 1;
  /*! \~english The Sleep Mode Select bits select between the three available sleep modes*/
  /*! \~german Sleep Mode Auswahl Bit, Auswahl zwischern drei Sleep Modes*/
  unsigned char SleepModeSelectBit2 : 1;
#endif
  };
  /*! \~english This property will guarantee raw access to the register*/
  /*! \~german  Diese Funktion gibt den direckten Zugang zum Register*/
} TMCUStatus;

#if (_AVR_IOMX8_H_)|(_AVR_IOM64_H_)|(_AVR_IOM8515_H_)

#if (_AVR_IOM64_H_)
# define MCUStatus (*(volatile TMCUStatus *)&MCUCSR)
#elif (_AVR_IOMX8_H_)
# define MCUStatus (*(volatile TMCUStatus *)&MCUSR)
#endif

#endif


/** 
   \class TMCUControl
   \brief MCUControl
   \~english This structure represents the MCU Control Register 
   \~german Mithilfe dieser Struktur kann vereinfacht auf das MCU Steuer Register zugegriffen werden
*/
typedef union
{
  struct {

  unsigned char ISC : 4;
  /*! \~english The Sleep Mode Select bits select between the three available sleep modes */
  /*! \~german Sleep Mode Auswahl Bit, Auswahl zwischern drei Sleep Modes*/
    unsigned char SleepModeSelectBit1 : 1;
  /*! \~english Make the MCU enter the sleep mode */
  /*! \~german  Setzt die MCU in den Sleep Mode*/
  unsigned char SleepEnable : 1;
  unsigned char SRW10_ : 1;
  unsigned char SRE_ : 1;
  unsigned char zunused0;
  unsigned char ISC2_ : 1;
  unsigned char SRW_ : 3;
  unsigned char SRL_ : 3;
  /*! \~english The Sleep Mode Select bits select between the three available sleep modes */
  /*! \~german Sleep Mode Auswahl Bit, Auswahl zwischern drei Sleep Modes */
  unsigned char SleepModeSelectBit0 : 1;
  };

  /*! \~english This property will gurantee raw access to the register */
  /*! \~german Diese Funktion gibt den direckten Zugang zum Register */
  unsigned char mcucr;
  unsigned char unused0;
  /*! \~english see :: EMCUCR*/
  /*! \~german Siehe ::EMCUCR*/
  unsigned char emcucr;
} TMCUControl;

/** 
   \class TInterrupts
   \brief Tnterrupts
   \~english This structure represents the Interrupt Control Register 
   \~german Mithilfe dieser Struktur kann vereinfacht auf das Interrupt Steuer Register zugegriffen werden
*/
typedef union
{
  struct {

  unsigned char unused1 : 5;
  /*! \~english Sends an an interruption inquiry when a event on the INT2 pin appears */
  /*! \~german Sendet eine Unterbrechungsanfrage wenn ein Ereignis an dem INT2 Pin auftritt */
  unsigned char ExternalInterrupt2Enabled : 1;
  /*! \~english If the logic or the Pereferie changes an interruption inquiry releases */
  /*! \~german  Löst eine Unterbrechungsanfrage aus, wenn sich die Logik oder die Pereferie ändert */
  unsigned char ExternalInterrupt0Enabled : 1;
  /*! \~english If the logic or the Pereferie changes an interruption inquiry releases */
  /*! \~german Löst eine Unterbrechungsanfrage aus, wenn sich die Logik oder die Pereferie ändert */
  unsigned char ExternalInterrupt1Enabled : 1;
  };
} TInterrupts;

#if (_AVR_IOM8515_H_)
#endif

#endif

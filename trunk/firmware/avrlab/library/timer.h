//License: see License.txt

#ifndef __EBLibTimer_H_
  #define __EBLibTimer_H_
//Supported Devices
// Atmega8515 - OK
// Atmega8

/** \file */
/** \~english \defgroup Timer Timer - Timer/Counter 
    \~german  \defgroup Timer Timer - Timer/Zähler
    \code #include "timer.h" \endcode
    \~english With Timer, you can generate Timebase(s) or Ticks or Waveforms<br>Example:
    \~german Mit Timern, kann man Zeitbasen oder Takte oder Wellenformen erstellen<br>Beispiel:
    \code 
      #include "timer.h"  
      
      int main(void)
      {
        Timer0.ClockSelect = 2;
        Timer0.Counter = 0x32;  
        ...
      } 
    \endcode
    */

/** 
   \class TTimer8Bit
   \brief Timer8Bit
   \~english This structure represents the 8-Bit Timer registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die 8-Bit Timer register zugegriffen werden
*/
typedef union
{
  struct
  {
	//Special Function IO Register - SFIOR
    /*! \~english When this bit is written to one, the Timer/Counter1 and Timer/Counter0 prescaler will be reset.*/
	/*! \~german  Wenn dieser Wert 1 ist, wird beim Timer1 und Timer0 der Prescaler Reset ausgelöst. */
    unsigned char PrescalerReset : 1;//PSR10:
    unsigned char unuse0 : 1;
    unsigned char PUD_ : 1;//PUD
    unsigned char XMM_ : 3;//XMM
    unsigned char XMBK_ : 1;//XMBK
    unsigned char unused1 : 1;
  #if (_AVR_IOM8515_H_)
    //Output Compare Register - OCR0
    /*! \~english The Output Compare Register contains an 8-bit value that is continuously compared with the counter value (TCNT0). A match can be used to generate an output compare interrupt, or to generate a waveform output on the OC0 pin.*/
    /*! \~german  Das Output Compare Register beinhaltet einen 8Bit Wert, der ständig mit dem Counter Wert(TCNT0/CounterRegister) verglichen wird. Eine Übereinstimmung kann verwendet werden, um einen Output Compare Interrupt oder eine Sinuswelle zu erzeugen.*/
    unsigned char OutputCompareRegister;//OCR0
  #endif
  #if (_AVR_IOM8_H_)
    unsigned char unused7;//OSCCAL
  #endif
  #if (_AVR_IOM64_H_)
    unsigned char unused7[15];
    //Asynchronous Status Register - ASSR
    /*! \~english When Timer/Counter0 operates asynchronously and TCCR0 is written, this bit becomes set. When TCCR0 has been updated from the temporary storage register, this bit is cleared by hardware. A logical zero in this bit indicates that TCCR0 is ready to be updated with a new value. If a write is performed to any of the three Timer/Counter0 registers while its update busy flag is set, the updated value might get corrupted and cause an unintentional interrupt to occur. The mechanisms for reading TCNT0, OCR0, and TCCR0 are different. When reading TCNT0, the actual timer value is read. When reading OCR0 or TCCR0, the value in the temporary storage register is read.*/
	unsigned char TimerCounterControlRegister0UpdateBusy : 1;//TCR0UB
	/*! \~english When Timer/Counter0 operates asynchronously and OCR0 is written, this bit becomes set. When OCR0 has been updated from the temporary storage register, this bit is cleared by hardware. A logical zero in this bit indicates that OCR0 is ready to be updated with a new value. */
	unsigned char OutputCompareRegister0UpdateBusy : 1;//OCR0UB
	/*! \~english When Timer/Counter0 operates asynchronously and TCNT0 is written, this bit becomes set. When TCNT0 has been updated from the temporary storage register, this bit is cleared by hardware. A logical zero in this bit indicates that TCNT0 is ready to be updated with a new value.*/
	unsigned char TimerCounter0UpdateBusy : 1;//TCN0UB
	/*! \~english When AS0 is written to zero, Timer/Counter0 is clocked from the I/O clock, clkI/O. When AS0 is written to one, Timer/Counter 0 is clocked from a crystal Oscillator connected to the Timer Oscillator 1 (TOSC1) pin. When the value of AS0 is changed, the contents of TCNT0, OCR0, and TCCR0 might be corrupted. */
	unsigned char AsynchronousTimer : 1;//AS0
	unsigned char unused9 : 4;
    //Output Compare Register - OCR0
    /*! \~english The Output Compare Register contains an 8-bit value that is continuously compared with the counter value (TCNT0). A match can be used to generate an output compare interrupt, or to generate a waveform output on the OC0 pin.*/
    /*! \~german  Das Output Compare Register beinhaltet einen 8Bit Wert, der ständig mit dem Counter Wert(TCNT0/CounterRegister) verglichen wird. Eine Übereinstimmung kann verwendet werden, um einen Output Compare Interrupt oder eine Sinuswelle zu erzeugen.*/
    unsigned char OutputCompareRegister;//OCR0
  #endif
    //Timer/Counter Register - TCNT0
    /*! \~english The Timer/Counter Register gives direct access, both for read and write operations, to the Timer/Counter unit 8-bit counter. Modifying the counter (TCNT0) while the counter is running, introduces a risk of missing a Compare Match between TCNT0 and the OCR0 Register.*/
    /*! \~german  Das Timer/Counter Register gibt einen diereckten Zugriff (lesen und schreiben) auf den Einheitenzähler. Beim Verändern des Counters im Betrieb, ´führt es zum Risiko, das ein fehlender Compare Match zwischen TCNT0 und OCR0 durchgeführt wird.*/
	unsigned char Counter;//TCNT0
    //Timer/Counter Control Register - TCCR0
    /*! \~english The three Clock Select bits select the clock source to be used by the Timer/Counter.*/
    /*! \~german  Die drei Clock Select Bits geben den Takt des Timer/counters an.*/
    unsigned char ClockSelect : 3;//CS00,CS01,CS02
	/*! \~english These bit control the counting sequence of the counter, the source for the maximum (TOP) counter value, and what type of waveform generation to be used. Modes of operation supported by the Timer/Counter unit are: Normal mode, Clear Timer on Compare Match (CTC) mode, and two types of Pulse Width Modulation (PWM) modes.*/
    /*! \~german  Der Wert kontrolliert die zähl sequenz des Counters, den Ursprung für den Max-Wert des Conters und den Typ der Wellenerzeugung. Einstellungsmöglichkeiten des Counters sind: Normal Mode, Clear Timer on Compare Match mode (CTC) und zwei Arten des Pulse Width Modulation (PWM) Mode.*/
  #if (_AVR_IOM8515_H_) || (_AVR_IOM64_H_)
	unsigned char WaveformGenerationMode01 :1;//WGM01
	/*! \~english These bits control the Output Compare pin (OC0) behavior. If one or both of the COM01:0 bits are set, the OC0 output overrides the normal port functionality of the I/O pin it is connected to. However, note that the Data Direction Register (DDR) bit corresponding to the OC0 pin must be set in order to enable the output driver.*/
    /*! \~german  Diese Werte kontrollieren das Verhalten des Output Compare Pins. Wenn einer oder beide Werte gesetzt sind, wird die normale I/O Funktion des OC0 Pin umgangen. Jedoch ist zu beachten, dass das Data Direction Register etsprechend zum OC0 Pin gesetzt werden muss, um den Output Treiber zu akivieren. */
	unsigned char CompareMatchOutputMode : 2;//COM00, COM01
	/*! \~english These bit control the counting sequence of the counter, the source for the maximum (TOP) counter value, and what type of waveform generation to be used. Modes of operation supported by the Timer/Counter unit are: Normal mode, Clear Timer on Compare Match (CTC) mode, and two types of Pulse Width Modulation (PWM) modes.*/
    /*! \~german  Der Wert kontrolliert die zähl sequenz des Counters, den Ursprung für den Max-Wert des Conters und den Typ der Wellenerzeugung. Einstellungsmöglichkeiten des Counters sind: Normal Mode, Clear Timer on Compare Match mode (CTC) und zwei Arten des Pulse Width Modulation (PWM) Mode.*/
	unsigned char WaveformGenerationMode00 : 1;//WGM00
	/*! \~english The FOC0 bit is only active when the WGM00 bit specifies a non-PWM mode. However, for ensuring compatibility with future devices, this bit must be set to zero when TCCR0 is written when operating in PWM mode.*/
    /*! \~german  Der FOC0 Wert ist nur gesetzt, wenn der WGM00 Wert den Non-PWM Mode angibt. Um jedoch die Kompatibilität mit zukünftigen Geräten zu gewährleißten, muss dieses Bit zu Null gesetzt werden, wenn TCCR0 im laufendem PWM Mode geschrieben wird. */
	unsigned char ForceOutputCompare : 1;//FOC0
  #endif
  #if (_AVR_IOM8_H_)
    unsigned char unused8 : 5;
  #endif
    unsigned char unused3[4];
    //Timer/Counter Interrupt Flag Register - TIFR
  #if (_AVR_IOM8515_H_)
	/*! \~english The OCF0 bit is set (one) when a Compare Match occurs between the Timer/Counter0 and the data in OCR0 - Output Compare Register0.*/
    /*! \~german  Der OCF0 Wert ist auf 1 Gesetzt wenn ein Compare Match zwischen Timer/Counter0 und dem Daten Eingang OCR0 - Output COmpare Register 0*/
    unsigned char OutputCompareFlag : 1;//OCF0
	/*! \~english The bit TOV0 is set (one) when an overflow occurs in Timer/Counter0*/
    /*! \~german  Das Bit-TOV0 wird aus 1 gesetzt, wenn ein Overflow im Timer/Counter0 vorkommt*/
	unsigned char OverflowFlag : 1;//TOV0
	unsigned char unused4 : 1;
	unsigned char ICF1_ : 1;//ICF1
	unsigned char unsused5 : 1;
	unsigned char OCF1B_ : 1;//OCF1B
	unsigned char OCF1A_ : 1;//OCF1A
	unsigned char TOV1_ : 1;//TOV1
  #endif
  #if (_AVR_IOM8_H_) || (_AVR_IOM64_H_)
	/*! \~english The bit TOV0 is set (one) when an overflow occurs in Timer/Counter0*/
    /*! \~german  Das Bit-TOV0 wird aus 1 gesetzt, wenn ein Overflow im Timer/Counter0 vorkommt*/
	unsigned char OverflowFlag : 1;//TOV0
   #if (_AVR_IOM8_H_)
	unsigned char unused9 : 7;
   #elif (_AVR_IOM64_H_)
	/*! \~english The OCF0 bit is set (one) when a Compare Match occurs between the Timer/Counter0 and the data in OCR0 - Output Compare Register0.*/
    /*! \~german  Der OCF0 Wert ist auf 1 Gesetzt wenn ein Compare Match zwischen Timer/Counter0 und dem Daten Eingang OCR0 - Output COmpare Register 0*/
    unsigned char OutputCompareFlag : 1;//OCF0
	unsigned char unused10 : 6;
   #endif
  #endif
  //Timer/Counter Interrupt Mask Register - TIMSK
  #if (_AVR_IOM8515_H_)
    /*! \~english When the OCIE0 bit is written to one, and the I-bit in the Status Register is set (one), the Timer/Counter0 Compare Match interrupt is enabled.*/
    /*! \~german  Wenn der OCIE0 Wert mit 1 geschrieben wird und das erste Bit im Status Register auch eins ist, ist der Timer Compare Match Interrupt aktiviert*/
    unsigned char OutputCompareMatchInterruptEnabled : 1;//OCIE0
	/*! \~english When the TOIE0 bit is written to one, and the I-bit in the Status Register is set (one), the Timer/Counter0 Overflow interrupt is enabled.*/
    /*! \~german  Wenn der TOIE0 Wert mit 1 geschrieben wird und das erste Bit im Status Register auch eins ist, ist der Timer Overflow Interrupt aktiviert. */
	unsigned char OverflowInterruptEnabled : 1;//TOIE0
	unsigned char unused6 : 1;
	unsigned char TICIE1_ : 1;//TICIE1
	unsigned char unused7 : 1;
	unsigned char _OCIE1B : 1;//OCIE1B
	unsigned char _OCIE1A : 1;//OCIE1A
	unsigned char _TOIE1 : 1;//TOIE1
  #endif
  #if (_AVR_IOM8_H_) || (_AVR_IOM64_H_)
	unsigned char OverflowInterruptEnabled : 1;//TOIE0
   #if (_AVR_IOM64_H_)
    /*! \~english When the OCIE0 bit is written to one, and the I-bit in the Status Register is set (one), the Timer/Counter0 Compare Match interrupt is enabled.*/
    /*! \~german  Wenn der OCIE0 Wert mit 1 geschrieben wird und das erste Bit im Status Register auch eins ist, ist der Timer Compare Match Interrupt aktiviert*/
    unsigned char OutputCompareMatchInterruptEnabled : 1;//OCIE0
   #endif
  #endif
  };
}TTimer8Bit;



/** 
   \class TTimer16Bit
   \brief Timer16Bit
   \~english This structure represents the 16-Bit Timer registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die 16 Bit Timer Register zugegriffen werden
*/
typedef union
{
/*! \~english This struct contains new register names, for original Amel names scroll down.*/
/*! \~german  Dieses struct enthält neue Registerbezeichnungen, für original Amel Bezeichnungen siehe unten.*/
  struct
  {
	//Input Capture Register 1 - ICR1
    /*! \~english The Input Capture is updated with the counter (TCNT1) value each time an event occurs on the ICP1 pin. The Input Capture can be used for defining the counter TOP value.*/
    /*! \~german  Das Input Capture Register aktualisiert sich mit dem Wert des Counters (TCNT1), jedes mal wenn etwas an ICP1 Pin geschieht. Das Input Capture kann auch genutzt werden um den TOP Wert des Counter zu definieren.*/
    unsigned int InputCaptureRegister;//ICR1
  #if (_AVR_IOM8515_H_)
	unsigned int unused0;
  #endif
	//Output Compare Register 1 B - OCR1B
    /*! \~english The Output Compare Registers contain a 16-bit value that is continuously compared with the counter value (TCNT1). A match can be used to generate an output compare interrupt, or to generate a waveform output on the OC1x pin.*/
    /*! \~german  Das Output Compare Register beinhaltet einen 16Bit Wert, welcher ständig mit dem Counter Wert(TCNT1) verglichen wird. Eine Übereinstimmung kann dazu genutzt werden, um einen Output Compare Interrupt zu erzeugen oder um einen Wellenförmigen Output am OC1x Pin zu erzeugen.*/
  	unsigned int OutputCompareRegisterB;//OCR1B
	//Output Compare Register 1 A - OCR1A
    /*! \~english The Output Compare Registers contain a 16-bit value that is continuously compared with the counter value (TCNT1). A match can be used to generate an output compare interrupt, or to generate a waveform output on the OC1x pin.*/
    /*! \~german  Das Output Compare Register beinhaltet einen 16Bit Wert, welcher ständig mit dem Counter Wert(TCNT1) verglichen wird. Eine Übereinstimmung kann dazu genutzt werden, um einen Output Compare Interrupt zu erzeugen oder um einen Wellenförmigen Output am OC1x Pin zu erzeugen.*/
    unsigned int OutputCompareRegisterA;//OCR1A
	//Timer/Counter1 - TCNT1
    /*! \~english The Timer/Counter I/O locations give direct access, both for read and for write operations, to the Timer/Counter unit 16-bit counter.*/
    /*! \~german  Die Timer/Counter I/O Addresse gib diereckten Zugriff (Read/Write) zum 16Bit Timer/Counter.*/
    unsigned int Counter;//TCNT1
	//Timer/Counter1 Control Register B - TCCR1B
	/*! \~english The three Clock Select bits select the clock source to be used by the Timer/Counter.*/
    /*! \~german  Drei ClockSelect Bits bestimmen den Tackt des Timers/Counters.*/
	unsigned char ClockSelect : 3;//CS10,CS11,CS12
    /*! \~english Combined with the WaveformGenerationMode bits found in the TCCR1B Register, these bits control the counting sequence of the counter.*/
    /*! \~german  Gehört zu den WaveformGenerationMode Bits im TCCR1B Regisster. DIese Bits kontrollieren die Zähls Sequenz des Counters.*/
	unsigned char WaveformGenerationMode12 : 1;//WGM12
    /*! \~english Combined with the WaveformGenerationMode bits found in the TCCR1B Register, these bits control the counting sequence of the counter.*/
    /*! \~german  Gehört zu den WaveformGenerationMode Bits im TCCR1B Regisster. DIese Bits kontrollieren die Zähls Sequenz des Counters.*/
	unsigned char WaveformGenerationMode13 : 1;//WGM13
	unsigned char unused1 : 1;
	/*! \~english This bit selects which edge on the Input Capture Pin (ICP1) that is used to trigger a capture event.*/
    /*! \~german  Mit diesem Bit wählt man den Bereich des Input Capture Pin (ICP1), der als Trigger benutzt wird.*/
	unsigned char InputCaptureEdgeSelect : 1;//ICES1
	/*! \~english Setting this bit (to one) activates the Input Capture Noise Canceler. The Input Capture is therefore delayed by four Oscillator cycles when the noise canceler is enabled.*/
    /*! \~german  Wenn dieser Wert auf 1 gesetzt wird, wird der Input Capture Noise Canceler aktiviert, dadurch vershcieb sich der Iput Capure um 4 Takte.*/
	unsigned char InputCaptureNoiseCanceler : 1;//ICNC1
	//Timer/Counter1 Control Register A - TCCR1A
    /*! \~english Combined with the WaveformGenerationMode bits found in the TCCR1B Register, these bits control the counting sequence of the counter.*/
    /*! \~german  Gehört zu den WaveformGenerationMode Bits im TCCR1B Regisster. DIese Bits kontrollieren die Zähls Sequenz des Counters.*/
	unsigned char WaveformGenerationMode10 : 1;//WGM10
    /*! \~english Combined with the WaveformGenerationMode bits found in the TCCR1B Register, these bits control the counting sequence of the counter.*/
    /*! \~german  Gehört zu den WaveformGenerationMode Bits im TCCR1B Regisster. DIese Bits kontrollieren die Zähls Sequenz des Counters.*/
	unsigned char WaveformGenerationMode11 : 1;//WGM11
  #if (_AVR_IOM64_H_)
	/*! \~english The COM1C:0 control the Output Compare pins (OC1A and OC1B).*/
    /*! \~german  Der COM1C:0 steuer die Output Compare Pins (OC1A und OC1B).*/
	unsigned char CompareOutputModeC : 2;//COM1C
  #else
	/*! \~english The FOC1B bit is only active when the WaveformGenerationMode13:0 bits specifies a non-PWM mode.*/
    /*! \~german  Das FOC1B bit ist nur gesetzt, wenn das WaveformGererationMode13:0 Bit den Non_PWM Mode angibt.*/
	unsigned char ForceOutputCompareB : 1;//FOC1B
	/*! \~english The FOC1A bit is only active when the WaveformGenerationMode13:0 bits specifies a non-PWM mode.*/
    /*! \~german  Das FOC1A bit ist nur gesetzt, wenn das WaveformGererationMode13:0 Bit den Non_PWM Mode angibt.*/
	unsigned char ForceOutputCompareA : 1;//FOC1A
  #endif
	/*! \~english The COM1B:0 control the Output Compare pins (OC1A and OC1B).*/
    /*! \~german  Der COM1B:0 steuer die Output Compare Pins (OC1A und OC1B).*/
	unsigned char CompareOutputModeB : 2;//COM1B
	/*! \~english The COM1A:0 control the Output Compare pins (OC1A and OC1B).*/
    /*! \~german  Der COM1A:0 steuer die Output Compare Pins (OC1A und OC1B).*/
    unsigned char CompareOutputModelA : 2;//COM1A

  #if !(_AVR_IOM64_H_)
    //Special Function IO Register - SFIOR
    /*! \~english When this bit is written to one, the Timer/Counter1 and Timer/Counter0 prescaler will be reset.*/
	/*! \~german  Wenn dieser Wert 1 ist, wird beim Timer1 und Timer0 der Prescaler Reset ausgelöst. */
    unsigned char PrescalerReset : 1;//PSR10
    unsigned char unused8 : 7;
    unsigned char unused3[7];
  #endif
	//Timer/Counter Interrupt Flag Register - TIFR
  #if (_AVR_IOM8515_H_)
	unsigned char unused4 : 3;
	unsigned char InputCapture : 1;//ICF1
	unsigned char unused5: 1
	/*! \~english This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output Compare Register B (OCR1B).*/
    /*! \~german  */;
	unsigned char OutputCompareMatchB : 1;//OCF1B
	/*! \~english This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output Compare Register A (OCR1A).*/
    /*! \~german  */
	unsigned char OutputCompareMatchA : 1;//OCF1A
	/*! \~english The setting of this flag is dependent of the WaveformGenerationMode bit setting. In Normal and CTC modes, the TOV1 Flag is set when the timer overflows.*/
    /*! \~german  Der Status dieses Flags ist abhänging von den WaveformGenerationMode Einstellungen.Im normalen und CTC Betrieb, wird das Flag gesetzt, wenn der Timer einen Overflow hat. */
	unsigned char OverflowFlag : 1;//TOV1
  #endif
  #if (_AVR_IOM8_H_)
	unsigned char unused4 : 2;
	/*! \~english The setting of this flag is dependent of the WaveformGenerationMode bit setting. In Normal and CTC modes, the TOV1 Flag is set when the timer overflows.*/
    /*! \~german  Der Status dieses Flags ist abhänging von den WaveformGenerationMode Einstellungen.Im normalen und CTC Betrieb, wird das Flag gesetzt, wenn der Timer einen Overflow hat. */
	unsigned char OverflowFlag : 1;//TOV1
	/*! \~english This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output Compare Register B (OCR1B).*/
    /*! \~german  */;
	unsigned char OutputCompareMatchB : 1;//OCF1B
	/*! \~english This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output Compare Register A (OCR1A).*/
    /*! \~german  */
	unsigned char OutputCompareMatchA : 1;//OCF1A
	/*! \~english This flag is set when a capture event occurs on the ICP1 pin. When the Input Capture Register (ICR1) is set by the WGM13:0 to be used as the TOP value, the ICF1 Flag is set when the counter reaches the TOP value. ICF1 is automatically cleared when the Input Capture Interrupt Vector is executed. Alternatively, ICF1 can be cleared by writing a logic one to its bit location.*/
    /*! \~german  */
    unsigned char InputCaptureOccoured : 1;//ICF1  
	unsigned char unused9 : 1;  
  #endif  
	//Timer/Counter Interrupt Mask Register - TIMSK
  #if (_AVR_IOM8515_H_)
    unsigned char unused6 : 3;
    /*! \~english When this bit is written to one, and the Interrupt-flag in the Status Register is set (interrupts globally enabled), the Timer/Counter1 Input Capture interrupt is enabled.*/
    /*! \~german  Wenn dieses Bit und das Interrupt Flag im Status Register gesetzt ist, ist der Timer7Counter Input Capture Interrupt aktiv*/
    unsigned char InputCaptureInterrupt1 : 1;//TICIE1
    unsigned char unused7 : 1;
    /*! \~english (first A register then B register)When this bit is written to one, and the Interrupt-flag in the Status Register is set, the Timer/Counter1 Output Compare Match interrupt is enabled.*/
    /*! \~german  (zuerst das Register A, dann das Register B)Wenn das Bit und das  Interrupt-Flag im Statusregister gesetzt ist, ist der Timer/Conter Output Compare Match Interrupt aktiv*/;
    unsigned char OutputCompareMatchInterruptB : 1;//OCIE1B
    unsigned char OutputCompareMatchInterruptA : 1;//OCIE1A
    /*! \~english When this bit is written to one, and the Interrupt-flag in the Status Register is set, the Timer/Counter1 overflow interrupt is enabled*/
    /*! \~german  Wenn das Bit 1 ist und das Interrupt-Flag im Statusregister gesetzt ist, ist der Timer/Counter Overflow Interrupt aktiv.*/
    unsigned char OverflowInterruptEnabled : 1;//TOIE1
  #endif
  #if (_AVR_IOM8_H_)
    unsigned char unused6 : 2;
    /*! \~english When this bit is written to one, and the Interrupt-flag in the Status Register is set, the Timer/Counter1 overflow interrupt is enabled*/
    /*! \~german  Wenn das Bit 1 ist und das Interrupt-Flag im Statusregister gesetzt ist, ist der Timer/Counter Overflow Interrupt aktiv.*/
    unsigned char OverflowInterruptEnabled : 1;//TOIE1
    /*! \~english (first A register then B register)When this bit is written to one, and the Interrupt-flag in the Status Register is set, the Timer/Counter1 Output Compare Match interrupt is enabled.*/
    /*! \~german  (zuerst das Register A, dann das Register B)Wenn das Bit und das  Interrupt-Flag im Statusregister gesetzt ist, ist der Timer/Conter Output Compare Match Interrupt aktiv*/;
    unsigned char OutputCompareMatchInterruptB : 1;//OCIE1B
    unsigned char OutputCompareMatchInterruptA : 1;//OCIE1A
    /*! \~english When this bit is written to one, and the Interrupt-flag in the Status Register is set (interrupts globally enabled), the Timer/Counter1 Input Capture interrupt is enabled.*/
    /*! \~german  Wenn dieses Bit und das Interrupt Flag im Status Register gesetzt ist, ist der Timer7Counter Input Capture Interrupt aktiv*/
    unsigned char InputCaptureInterrupt1 : 1;//TICIE1
    unsigned char unused7 : 2;
  #endif
  };
}TTimer16Bit;


#if (_AVR_IOM8515_H_) || (_AVR_IOM8_H_) || (_AVR_IOM64_H_)
 #define Timer  (*(volatile TTimer8Bit *)&SFIOR)
 #define Timer0 (*(volatile TTimer8Bit *)&SFIOR)
 #if !(_AVR_IOM64_H_)
  #define Timer1 (*(volatile TTimer16Bit *)&ICR1)
 #endif 
#endif

#endif

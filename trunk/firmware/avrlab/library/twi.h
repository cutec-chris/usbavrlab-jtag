//License: see License.txt

#ifndef __EBLibTWI_H_
  #define __EBLibTWI_H_

//Supported Devices:
// Atmega64
// Atmega8

/** \file */
/** \~english \defgroup TWI TWI - Two Wire Interface (I2C) 
    \~german  \defgroup TWI TWI - Zweidrahtinterface (I2C)
    \code #include "twi.h" \endcode
    \~english With TWI (I2C) you can communicate with many perpherials<br>Example:
    \~german Mit TWI (I2C) können Sie mit unterschiedlichsten externen Komponenten kommunizieren<br>Beispiel:
    \code 
      #include "twi.h"  
      
      int main(void)
      {
      ...          
      } 
    \endcode
    */

/** 
   \class TTwiControl
   \brief TwiControl
   \~english This structure represents the TWI registers 
   \~german Mithilfe dieser Struktur kann vereinfacht auf die TWI register zugegriffen werden
*/
typedef union
{
  struct
  {
    //TWBR TWI Bit Rate Register
    /*! \~english selects the division factor for the bit rate generator. The bit rate generator is a frequency divider which generates the SCL clock frequency in the Master modes. */
    /*! \~german  Gibt den Divisionsfaktor für den Bitrate Generator an. Der Bit Raten generator ist ein Teiler der den Takt für SCL im Master Modus erzeugt.*/
    unsigned char BitRateDivider;
    //TWSR TWI Status register   
    unsigned char Prescaler : 2;
    unsigned char unused0 : 1;
    unsigned char Status : 5;
    //TWAR TWI Address register    
    unsigned char TWGCE_ : 1;
    unsigned char Address : 7;
    //TWDR TWI Data Register
    unsigned char Data;
#if (_AVR_IOM8_H_)
   unsigned char unused2[33];
#endif        
	//TWCR TIW Control Register
    /*! \~english This bit is set by hardware when the TWI has finished its current job and expects application software response. If the I-bit in SREG and TWIE in TWCR are set, the MCU will jump to the TWI Interrupt Vector. While the TWINT flag is set, the SCL low period is stretched. The TWINT flag must be cleared by software by writing a logic one to it. Note that this flag is not automatically cleared by hardware when executing the interrupt routine. Also note that clearing this flag starts the operation of the TWI, so all accesses to the TWI Address Register (TWAR), TWI Status Register (TWSR), and TWI Data Register (TWDR) must be complete before clearing this flag.*/
    unsigned char InterruptEnabled : 1;//TWIE
    unsigned char unused1 : 1;
    /*! \english The TWEN bit enables TWI operation and activates the TWI interface. When TWEN is written to one, the TWI takes control over the I/O pins connected to the SCL and SDA pins, enabling the slew-rate limiters and spike filters. If this bit is written to zero, the TWI is switched off and all TWI transmissions are terminated, regardless of any ongoing operation.*/      
    unsigned char Enabled : 1; //TWEN        
    /*! \~english The TWWC bit is set when attempting to write to the TWI Data Register – TWDR when TWINT is low. This flag is cleared by writing the TWDR Register when TWINT is high.*/ 
    unsigned char WriteCollission : 1; //TWWC
    /*! \~english Writing the TWSTO bit to one in Master mode will generate a STOP condition on the Two-wire Serial Bus. When the STOP condition is executed on the bus, the TWSTO bit is cleared automatically. In Slave mode, setting the TWSTO bit can be used to recover from an error condition. This will not generate a STOP condition, but the TWI returns to a well-defined unaddressed Slave mode and releases the SCL and SDA lines to a high impedance state.*/
    unsigned char StopCondition : 1; //TWSTO  
    /*! \~english The application writes the TWSTA bit to one when it desires to become a Master on the Twowire Serial Bus. The TWI hardware checks if the bus is available, and generates a START condition on the bus if it is free. However, if the bus is not free, the TWI waits until a STOP condition is detected, and then generates a new START condition to claim the Bus Master status. TWSTA must be cleared by software when the START condition has been transmitted.*/
    unsigned char StartCondition : 1;//TWSTA
    /*! \~english The TWEA bit controls the generation of the acknowledge pulse. If the TWEA bit is written to one, the ACK pulse is generated on the TWI bus if the following conditions are met:<br>1. The device’s own slave address has been received.<br>2. A general call has been received, while the TWGCE bit in the TWAR is set.<br>3. A data byte has been received in Master Receiver or Slave Receiver mode.<br>By writing the TWEA bit to zero, the device can be virtually disconnected from the Two-wire Serial Bus temporarily. Address recognition can then be resumed by writing the TWEA bit to one again.*/
    unsigned char EnableAcknowledge : 1;//TWEA
    /*! \~english This bit is set by hardware when the TWI has finished its current job and expects application software response. If the I-bit in SREG and TWIE in TWCR are set, the MCU will jump to the TWI Interrupt Vector. While the TWINT flag is set, the SCL low period is stretched. The TWINT flag must be cleared by software by writing a logic one to it. Note that this flag is not automatically cleared by hardware when executing the interrupt routine. Also note that clearing this flag starts the operation of the TWI, so all accesses to the TWI Address Register (TWAR), TWI Status Register (TWSR), and TWI Data Register (TWDR) must be complete before clearing this flag.*/
    unsigned char InterruptOccoured : 1;//TWINT
  };
} TTwiControl;

#if (_AVR_IOM64_H_) || (_AVR_IOM8_H_)  
 #define TwiControl  (*(volatile TTwiControl *)&TWBR)
#endif

#endif

//License: see License.txt

/** \file */
/** \~english \defgroup Messages Messages - Asyncron Programming routines 
    \~german  \defgroup Messages Messages - Routinen für asynchrone Programmierung
    \~english Message Handling Routines \~german Narichten verarbeitung 
    \code #include "messages.h" \endcode
    \~english With the message handling routines you can easyly with low code size (~350 byte) send messages at example from interrupt routines and handle them in the main program.<br>Example:
    \~german Mit den Nachrichten-verarbeitungs Routinen können Sie einfach Nachrichten senden und bearbeiten z.b. können Sie aus einer Interrupt Routine eine Nachricht senden, und diese im Hauptprogramm verarbeiten.<br>Beispiel:
    \code 
      #include "messages.h"  
      #include "gpio.h"
      
      #define Led_Port PortB.Output.Bit2
      #define Led_DDR  PortB.DataDirection.Bit2
      #define MSG_LED  100
                           
      void ProcessMessage(TMessage Msg)
      {
        switch (Msg.Message)
        {
        case MSG_LED:
          {
            Led_Port = (char)*Param1;
          }
        }
      }
      
      int main(void)
      {
        char a = 1; 
        Led_DDR = DataDirectionOutput;
        SendMessage(MSG_LED,&a,NULL);       
        while (1)  
          if (MessageThere())
            ProcessMessage(GetMessage)          
      } 
    \endcode
    */

/*! \~english Define here how many messages should be buffered. */
/*! \~german Definieren Sie hier wie viele Nachrichten zwischengespeichert werden sollen. */
#define MESSAGE_COUNT 10

typedef struct 
{
  unsigned char Active : 1;
  unsigned char Message : 7;
  void *Param1;
  void *Param2;
} TMessage;

/*! \brief SendMessage \~german eine Routine zum senden von Nachrichten \~english An function to send a message
 *  \param \~german Msg Die Nachrichtennummer Msg referenziert die Naricht \~english The message number Msg is an reference for the message
 *  \param \~german Param1 Param1 ist ein Zeiger auf jede Datenstruktur die Sie möchten \~english Param1 is an pointer to any data you wish
 *  \param \~german Param2 Param1 ist ein Zeiger auf jede Datenstruktur die Sie möchten \~english Param2 is an pointer to any data you wish
 */
void SendMessage(unsigned char Msg,void *Param1,void *Param2);

/*! \brief MessageThere \~german eine Routine zum prüfen, ob neuen Narichten vorhanden sind \~english An function to check for new Messages
 */
unsigned char MessageThere(void);
/*! \brief GetMessage \~german Eine Routine um die nächste Naricht aus dem Narichten Buffer abzuholen \~english An function to get the next new message
 */
TMessage GetMessage(void);

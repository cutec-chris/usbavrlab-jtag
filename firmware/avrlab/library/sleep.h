//License: see License.txt

#ifndef __EBLibSleep_H_
 #define __EBLibSleep_H_

/** \file */
/** \~english \defgroup Sleep Sleep - Routines to Sleep a while
    \~german  \defgroup Sleep Sleep - Routinen zu warten
    \code #include "sleep.h" \endcode 
    \~english This are routines to wait<br>Example:
    \~german Routines to wait<br>Beispiel:
    \code 
      #include "lcd.h"  
      
      int main(void)
      {
        Sleep(10,Seconds);
      } 
    \endcode
    
    */

#define Seconds 1
#define Milliseconds 2
#define Microseconds 3

void Sleep(unsigned int Time,unsigned char TimeBase);

#endif


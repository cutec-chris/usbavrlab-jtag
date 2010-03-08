//License: see License.txt
#include "sleep.h"
#include <util/delay.h>

void Sleep(unsigned int Time,unsigned char TimeBase)
{
  switch(TimeBase)
  {
  case Microseconds: while (Time) { _delay_us(1); Time--;break;}
  case Milliseconds: while (Time) { _delay_ms(1); Time--;break;}
  case Seconds: while (Time) { _delay_us(500); _delay_us(500); Time--;break;}
  }
}

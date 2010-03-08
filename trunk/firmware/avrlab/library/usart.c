//License: see License.txt

#include <avr/io.h>
#include <util/setbaud.h>


void Usart0_init(unsigned int baudrate)
{
  Usart0.BaudRateHeigh = UBRRH_VALUE()
}

void Usart0_print(char *string)
{
//  while(*string) 
//    Lcd_write(0,*string++);
}

void Usart0_printP(char *string)
{
//  while(pgm_read_byte(string)) 
//    Lcd_write(0,pgm_read_byte(string++));
}

const TUsart Usart = {Usart0_init,Usart0_print,Usart0_printP};

const TUsart Usart1 = {Usart1_init,Usart1_print,Usart1_printP};


//License: see License.txt

#include "messages.h"

TMessage Messages[MESSAGE_COUNT];

void SendMessage(unsigned char Msg,void *Param1,void *Param2)
{
  unsigned char i = 0;
  while (i < MESSAGE_COUNT)
    if (!(Messages[i].Active))
	  {
	    Messages[i].Active = 1;
		Messages[i].Message = Msg;
		Messages[i].Param1 = Param1;
		Messages[i].Param2 = Param2;
		break;
	  }
}

unsigned char MessageThere(void)
{
  unsigned char i = 0;
  while (i < MESSAGE_COUNT)
    if (!(Messages[i].Active))
	  return 1;
  return 0;
}

TMessage GetMessage(void)
{
  TMessage Msg;
  unsigned char i = 0;
  while (i < MESSAGE_COUNT)
    if (!(Messages[i].Active))
	  {
	    Msg = Messages[i];
		Messages[i].Active = 0;
		return Msg;
	  }
  return Msg;
}

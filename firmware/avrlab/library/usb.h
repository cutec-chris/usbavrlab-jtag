//License: see License.txt

#if (_AVR_IOUSBXX2_H_)
/** 
   \class TUsb
   \brief Usb
   \~english This Structurer represents the USB Controller of the Atmel AT90USBxxx devices
   \~german Mithilfe dieser Struktur kann vereinfacht auf den USB Controller der Atmel AT90USBxxx Controller zugegriffen werden
*/
typedef union
{
  struct
  {
  //USBCON
  unsigned char unused0 : 5;
  /*! \~english Set to disable the clock inputs (the "Resume Detection" is still active). This reduces the power consumption. Clear to enable the clock inputs. */
  /*! \~german Setzen Sie dieses Bit um den USB Takt zu deaktivieren (die "Reaktivierungs Erkennung" bleibt aktiv). Dies senkt den Stromverbrauch. */
  unsigned char FreezeClock : 1; //FRZCLK
  unsigned char unused1 : 1;
  /*! \~english Set to enable the USB controller. Clear to disable and reset the USB controller, to disable the USB transceiver and to disable the USB controller clock inputs. */
  /*! \~german Setzen Sie diese Bit um den USB Controller zu aktivieren, setzen Sie es auf 0 um den USB Controller zu deaktivieren und resetten. */
  unsigned char Enabled : 1; //USBE
  
  unsigned char unused2[2];
  //UDPADH
  unsigned char unused3 : 7;
  /*! \~english Set this bit to directly read the content the Dual-Port RAM (DPR) data through the UEDATX or UPDATX registers. */
  /*! \~german Setzen Sie dieses Bit um direkt aus dem Dual-Port RAM des USB Controllers über die UEDATX oder UPDATX Register. */
  unsigned char DPRAMDirectAccess : 1; //DPACC
  //UDPADL
  unsigned char udpadl;

  unsigned char unused4[3];  
  //UDCON
  unsigned char detach : 1;
  unsigned char rmwkup : 1;
  unsigned char rstcpu : 1;
  unsigned char unused5 : 5;
  //UDINT
  unsigned char suspi : 1;
  unsigned char unused6 : 1;
  unsigned char sofi : 1;
  unsigned char eorsti : 1;
  unsigned char wakeupi : 1;
  unsigned char eorsmi : 1;
  unsigned char uprsmi : 1;
  unsigned char unused7 : 1;
  //UDIEN
  unsigned char suspe : 1; 
  unsigned char unused8 : 1;
  unsigned char sofe : 1;
  unsigned char eorste : 1;
  unsigned char wakeupe : 1;
  unsigned char eorsme : 1;
  unsigned char uprsme : 1;
  unsigned char unused9 : 1;
  //UDADDR
  unsigned char unused10 : 7;
  unsigned char adden : 1;
  //UDFNUM
  unsigned int  udfnum;

  unsigned char unused11 : 4;
  unsigned char fncerr : 1;
  unsigned char unused12 : 3;

  unsigned char unused13;
  //UEINTX
  unsigned char txini : 1;
  unsigned char stalledi : 1;
  unsigned char rxouti : 1;
  unsigned char rxstpi : 1;
  unsigned char nakouti : 1;
  unsigned char rwal : 1;
  unsigned char nakini : 1;
  unsigned char fifocon : 1;
  //UENUM
  unsigned char epnum0 : 1;  
  unsigned char epnum1 : 1;  
  unsigned char epnum2 : 1;  
  unsigned char unused14 : 5;
  //UERST
  unsigned char eprst0 : 1;
  unsigned char eprst1 : 1;
  unsigned char eprst2 : 1;
  unsigned char eprst3 : 1;
  unsigned char eprst4 : 1;
  unsigned char unused15 : 3;
  //UECONX
  unsigned char epen : 1;
  unsigned char unused16 : 2;
  unsigned char rstdt : 1;
  unsigned char stallrqc : 1;
  unsigned char stallrq : 1;
  unsigned char unused17 : 2; 
  //UECFG0X
  unsigned char epdir : 1;
  unsigned char unused18 : 5;
  unsigned char eptype0 : 1;
  unsigned char eptype1 : 1;
  //UECFG1X
  unsigned char unused19 : 1;
  unsigned char alloc : 1;
  unsigned char epbk0 : 1;
  unsigned char epbk1 : 1;
  unsigned char epsize0 : 1;
  unsigned char epsize1 : 1;
  unsigned char epsize2 : 1;
  unsigned char unused20 : 1;
  //UESTA0X
  unsigned char nbusybk0 : 1;
  unsigned char nbusybk1 : 1;
  unsigned char dtseq0 : 1;
  unsigned char dtseq1 : 1;
  unsigned char unused21 : 1;
  unsigned char underfi : 1;
  unsigned char overfi : 1;
  unsigned char cfgok : 1;
  //UESTA1X
  unsigned char currbk0 : 1;
  unsigned char currbk1 : 1;
  unsigned char ctrldir : 1;
  unsigned char unused22 : 5;
  //UEIENX
  unsigned char txine : 1;
  unsigned char stallede : 1;
  unsigned char rxoute : 1;
  unsigned char rxstpe : 1;
  unsigned char nakoute : 1;
  unsigned char unused23 : 1;
  unsigned char nakine : 1;
  unsigned char flerre : 1;

  unsigned char uedatx;

  unsigned char uebclx;

  unsigned char unused24;
  //UEINT
  unsigned char epint0 : 1;
  unsigned char epint1 : 1;
  unsigned char epint2 : 1;
  unsigned char epint3 : 1;
  unsigned char epint4 : 1;
  unsigned char unused25 : 3;
  };
} TUsbController;

#define UsbController (*(volatile TUsbController *)&USBCON)
#endif


#ifndef _XMODEM_H_
#define _XMODEM_H_
#ifdef INCLUDE_SERIAL_FLASH
#include <serialdriver.h>
#endif


#define     XMODEM_OK           0          
#define     XMODEM_COMPLETED    1          
#define     XMODEM_CANCEL       2          
#define     XMODEM_ABORT        3          
#define     XMODEM_TIMEOUT      4          
#define     XMODEM_ERROR        5          
#define     XMODEM_NOTHING      6
#define     XMODEM_CONTINUE     7
#define     XMODEM_END          8          

#define     XMODEM_NONE         0
#define     XMODEM_SEND         1
#define     XMODEM_RECEIVE      2

#define     SOH	            0x01    /* Start of Frame char  */
#define     STX             0x02    /*                      */
#define     EOT	            0x04    /* End of Transmit char */
#define     ACK	            0x06    /* Acknowledge char     */
#define     BSP	            0x08    /* back space     */
#define     NAK	            0x15    /* No Ack char          */
#define     CAN	            0x18    /* Cancel char          */
#define     XV_TIMEOUT         -1

#define	XpktSOH		1
#define	XpktBLK		2
#define	XpktBLK2	3
#define	XpktDATA	4

#define	XnakNAK	1
#define	XnakC	2
  
#define	XoptCheck	1
#define	XoptCRC		2
#define	Xopt1K		3


#define	pause(x)	    OSdelayshort(x*10)

UNSIGNED8 XMODEM_Check_Tran_Done(void);
UNSIGNED32 XMODEM_PUT_PACKETS(UNSIGNED8* ptr, UNSIGNED32 length);
void XMODEM_PUT_CHAR(UNSIGNED8* Data);
void XMODEM_PUT_Message(UNSIGNED8* String);
UNSIGNED16 XMODEM_GET_CHARS(UNSIGNED8 *Buffer, UNSIGNED16 bufSize,UNSIGNED16 *len);
UNSIGNED16 XMODEM_GET_Packet(UNSIGNED8 *Buffer, UNSIGNED16 bufSize);
void XMODEM_RX_Flush(void);
UNSIGNED8 XMODEM_SendNAK(void);
void XMODEM_SetOption(UNSIGNED8 option);
void XMODEM_Cancel(void);
UNSIGNED16 XMODEM_CalcCheck(UNSIGNED8* PktBuf);
UNSIGNED16 Calculate_CRC(UNSIGNED16 crc,UNSIGNED8 ch);
void XMODEM_Config (UNSIGNED8 XM_Mode,UNSIGNED8 XM_Type);

//gloabl xmodem interface
UNSIGNED8   XMODEM_Upload(UNSIGNED8 *, UNSIGNED32, UNSIGNED32 *);    
UNSIGNED8   XMODEM_Download(UNSIGNED8 *, UNSIGNED32, UNSIGNED32 *);  


#define     CRC_MODE        0
#define     CHECKSUM_MODE  1

//number of interface , which could apply xmodem 
#define NumOfXMInterface 1

#ifdef INCLUDE_SERIAL_FLASH

extern MSTP_HANDLE    *serPort[2];

//use FC bus as xmodem interface
#define xmodem_Bus FIELD_BUS

//128 bytes + 3 header +2 crc
#define XMODEM_BUFFER_SIZE 128+5 

 //64 is perfect for MSTP in FC buss
#define MSTP_TRANS_SIZE 64 

// it is 5ms for MSTP in FC buss
#define MSTP_INTERVAL 5  

//RX timeout value is 1s == 1000ms/MSTP_INTERVAL
#define MSTP_RX_DETECT 200  

//blank data of N25Q serial flash is 0xFF
#define SERIAL_FLASH_BLANK_DATA 0xFF 

//timeout is 20s for wait download data from utility, it is modifiable
#define UPLOAD_TIMEOUT_SECOND 20  

//timeout is 20s for receive data from utility, it is modifiable
#define DOWNLOAD_TIMEOUT_SECOND 20 

//baudrate 9600bps and 76800bps
#define XM_Baudrate_9600  2
#define XM_Baudrate_76800 5

#define SERIAL_BETTER_PERFORMANCE

#ifdef SERIAL_BETTER_PERFORMANCE
//Macro for setting or changing baud rate.
//"HANDLE" is input declared as type 'MSTP_HANDLE *'
// UNSIGNED8 BAUD_RATE
#define XM_MSTP_SET_BAUD_RATE(HANDLE, BAUD_RATE)                            \
{                                                                                   \
    MSTP_PORT_ARGUMENTS  LocalArguments;                                            \
    LocalArguments.BaudRate = BAUD_RATE;                                            \
    MSTPSI_Ioctl(HANDLE, IOCTL_SET_BAUD_RATE, &LocalArguments);            \
}
#endif 
#endif

typedef struct
{
  void      ( * InterfaceConfig)(UNSIGNED8 );
  UNSIGNED16( * InterfaceGetData)(UNSIGNED8 *, UNSIGNED16);
  void      ( * InterfaceSendData)(UNSIGNED8 *, UNSIGNED16);
  UNSIGNED8 ( * InterfaceTXEnd)(void);
  void      ( * InterfaceTimeDelay)(void);
} XM_Interface_Entry;

typedef struct  
{
  UNSIGNED32	PktBufPtr; 
  UNSIGNED32	PktNumOffset; 
  UNSIGNED32	PacketLen; 
  UNSIGNED32	BufferLen; 
  UNSIGNED32	DataLen;
  UNSIGNED8	*PktBuf; 
  UNSIGNED8	PktNum; 
  UNSIGNED8	PktNumSent; 
  UNSIGNED8	XMode; 
  UNSIGNED8	XOption; 
  UNSIGNED8	NAKMode; 
  UNSIGNED8	NAKCount; 
  UNSIGNED8	CheckLen; 
  UNSIGNED8	TimeOut;
  XM_Interface_Entry XM_Interface[NumOfXMInterface];
  void      ( * ProgramData)(UNSIGNED32 , UNSIGNED8* , UNSIGNED32);
} XmodemCB;

#endif

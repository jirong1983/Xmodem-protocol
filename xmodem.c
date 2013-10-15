#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "..\hardware.h"   
#include "..\main.h"
#include "..\N25Q.h"
#include "xmodem.h"

/************************************************************************
*                          C O N S T A N T S
*************************************************************************
*/                
#ifndef PUBLIC
#define	PUBLIC
#endif
#ifndef PRIVATE
#define	PRIVATE static
#endif 



/************************************************************************
*                         D A T A   T Y P E S
*************************************************************************
*/
typedef struct XmodemCB 
{
	uint32	PktBufCount; 
	uint32	PktBufPtr; 
	uint32	PktNumOffset; 
	uint32	DataLen; 
	uint8	PktBuf[1030]; 
	uint8	PktNum; 
	uint8	PktNumSent; 
	uint8	XMode; 
	uint8	XOption; 
	uint8	PktReadMode; 
	uint8	NAKMode; 
	sint	NAKCount; 
	uint8	CheckLen; 
	uint8	TimeOut; 
	uint8	TOutShort; 
	uint8	TOutLong; 
} XMODEM_CB_S, *XMCB; 

/************************************************************************
*                             M A C R O S
*************************************************************************
*/

/************************************************************************
*              F U N C T I O N   D E C L A R A T I O N S
*************************************************************************
*/
PRIVATE uint8   XMODEM_Read1Byte(sint*); 
PRIVATE void    XMODEM_SetOption(uint8); 
PRIVATE sint  XMODEM_Write(uint8*, uint32);
PRIVATE void    XMODEM_Cancel(void);
PRIVATE uint8   XMODEM_SendNAK(void);
//send bufeer
PRIVATE uint16  XMODEM_CalcCheck(uint8*); 
PRIVATE uint8   XMODEM_SendFrame(void);                
PUBLIC  uint8   XMODEM_SendBuffer(uint8 *, uint32, uint32 *);    
//receive buffer
PRIVATE uint8   XMODEM_RcvFrame(void);              
PUBLIC  uint8   XMODEM_RcvBuffer(uint8 *, uint32, uint32 *);     


/************************************************************************
*                        P U B L I C   D A T A
*************************************************************************
*/   
XMODEM_CB_S   XV;
XMCB          xv;


PRIVATE uint32   send_length;  
#ifndef SF_Xmodem
PRIVATE uint8*  send_ptr;
#else
PRIVATE uint32  send_ptr;
#endif

PUBLIC  uint32  Xmodem_Mode = CRC_MODE;

/************************************************************************
*                       P R I V A T E   D A T A
*************************************************************************
*/


/************************************************************************
*        F U N C T I O N   D E F I N I T I O N S
*************************************************************************
*/
PRIVATE	uint16	Calculate_CRC(uint16 crc,uint8 ch);

//used to wait, this moment CPU will do nothing but just count systemticks
PRIVATE void pause(uint32 ms)
{
	volatile uint32 TimeBase;
	uint32  TimePassed;

	TimeBase = system_ticks;
	do {
		TimePassed = system_ticks;
		if ( TimePassed < TimeBase ) {
			TimePassed += 0xFFFFFFFF - TimeBase;
		}
		else {
			TimePassed -= TimeBase;
		}
	}
	while ( TimePassed < ms );
}

/*************************************************************************
 *                 Code Begin -- XMODEM Sending Part                     *
 *************************************************************************/

/************************************************************************
 * Procedure  : XMODEM_SendFrame( void )
 * Input      : none.
 * Output     : none.
 * Description: It sends a frame (128 bytes data + header) to RS-232
 *              and wait for acknowledge (cancel, timeout).
 *              if ACK come => XMDMStatus = XMODEM_OK
 *              if timeout  => XMDMStatus = XMODEM_ABORT
 *              if CAN come => XMDMStatus = XMODEM_CANCEL
 ************************************************************************/
 
PRIVATE uint8 XMODEM_SendFrame(void)  
{                                                 
    sint       i,ch;
    uint16    Check;
    uint8     SendFlag;
    uint8     canCount;
    #ifdef SF_Xmodem
    uint8 SF_Block[128];
	#endif

	xv = &XV;
    SendFlag = FALSE; 
    
    if (xv->PktBufCount==0)
    {
        canCount=0;
        XMODEM_Read1Byte(&ch);
        do
        {
        	#if 1
            if (ch==XV_TIMEOUT)  //frank change to 0xFF
            //if (ch==0xFF)
            {
                xv->XMode=0;
                return XMODEM_TIMEOUT;
            } 
			#endif
            switch (ch&0xff) 
            {   
                case ACK:
                    if(xv->XMode==0)      
                    {
                        pause(10);
                        return (XMODEM_COMPLETED);
                    }
                    else if (xv->PktNumSent==(uint8)(xv->PktNum+1))
                    {
                        xv->PktNum = xv->PktNumSent;
                        if (xv->PktNum==0)
                            xv->PktNumOffset = xv->PktNumOffset + 256;
                        SendFlag = TRUE;
                    }
                    break;

                case NAK:
                    SendFlag = TRUE;
                    break;

                case CAN:
                    if(++canCount>=2) return  XMODEM_ABORT;
                    break;

                case 'C':
                    if ((xv->PktNum==0) && (xv->PktNumOffset==0))
                    {
                        if ((xv->XOption==XoptCheck) && (xv->PktNumSent==0)) 
                            XMODEM_SetOption(XoptCRC);
                        if (xv->XOption!=XoptCheck) 
                            SendFlag = TRUE;
                    }
                    break;
            }
            if (! SendFlag) XMODEM_Read1Byte(&ch);
        } while (!SendFlag);
        
        if (xv->PktNumSent==xv->PktNum) /* make a new packet */
        {
            xv->PktNumSent++;
            if (xv->DataLen==128)
                xv->PktBuf[0] = SOH;
            else
                xv->PktBuf[0] = STX;
            xv->PktBuf[1] = xv->PktNumSent;
            xv->PktBuf[2] = ~ ((uint8)xv->PktNumSent);

            i = 1;
			
            #ifndef SF_Xmodem
            while ((i<=xv->DataLen) && send_length>0)
            {
				xv->PktBuf[2+i] = *send_ptr++;
                i++;
                send_length--;
            }
            #else
            //read 128 bytes or less data from serial flash
            if((i<=xv->DataLen) && send_length>0)
            {
			  while(Flash_Success!= FlashRead((uint32)send_ptr,&xv->PktBuf[2+i],xv->DataLen));
			  {
			    send_ptr += xv->DataLen;
			    i += xv->DataLen;
				send_length -= xv->DataLen;
			  }
            }
            #endif
			
            if (i>1)
            {
                while (i<=xv->DataLen)
                {
                    xv->PktBuf[2+i] = 0x1A;
                    i++;
                }

                Check = XMODEM_CalcCheck(xv->PktBuf);
                if (xv->CheckLen==1) /* Checksum */
                    xv->PktBuf[xv->DataLen+3] = (uint8)Check;
                else 
                {
                    xv->PktBuf[xv->DataLen+3] = (uint8)((Check&0xff00)>>8);
                    xv->PktBuf[xv->DataLen+4] = (uint8)(Check&0xff);
                }
                xv->PktBufCount = 3 + xv->DataLen + xv->CheckLen;
            }
            else
            { /* send EOT */
                pause(2);
                xv->XMode=0;
                xv->PktBuf[0] = EOT;
                xv->PktBufCount = 1;
                xv->PktBufCount -= XMODEM_Write(&xv->PktBuf[xv->PktBufPtr],
                                   xv->PktBufCount);
                xv->PktBufCount -= XMODEM_Write(&xv->PktBuf[xv->PktBufPtr],
                                   xv->PktBufCount);
                
                flush_alt485();
                return (XMODEM_COMPLETED);
            }

            xv->PktBufPtr = 0;
        }
        else 
        { /* resend packet */
            xv->PktBufCount = 3 + xv->DataLen + xv->CheckLen;
            xv->PktBufPtr = 0;
        }
    }

    xv->PktBufCount -= XMODEM_Write(&xv->PktBuf[xv->PktBufPtr],
                                     xv->PktBufCount);
    return (uint8)XMODEM_OK;

}                                                 

/************************************************************************
* Procedure  : XMODEM_SendBuffer(public)                                *
* Input      : *data_ptr   : send buffer start address                  *
*              data_length : send buffer length                         *
* Output     : none                                                     *
* Description: It send a buffer to RS-232 Port by xmodem protocol       *
*************************************************************************/
PUBLIC uint8 XMODEM_SendBuffer
(   uint8 *data_ptr,
    uint32 data_length, 
    uint32 *snd_len
)
{                                                 
    uint8     ret;
	xv = &XV;
       
    *snd_len = 0;                                 
    send_length = data_length;   
	#ifndef SF_Xmodem
    send_ptr =   data_ptr; 
	#else
	send_ptr =(uint32)data_ptr; 
	#endif

    xv->XMode = XMODEM_SEND;
    xv->PktNumOffset = 0;
    xv->PktNum = 0;
    xv->PktNumSent = 0;
    xv->PktBufCount=0;
    xv->TOutShort = TimeOutShort;
    xv->TOutLong = TimeOutLong;
    xv->XOption = XoptCheck;
    XMODEM_SetOption( xv->XOption );
    xv->NAKMode = XnakNAK;
    xv->NAKCount = 10;

    xv->TimeOut = TimeOutVeryLong;

    PUT_Message("Starting XMODEM download...\r\n");

    while (send_length >= 0)                      
    {       
    //frank read from serial flash to global subsector memory
    
        ret=XMODEM_SendFrame();
        switch (ret)
        {
            case XMODEM_ABORT:
                return (XMODEM_ABORT);
                
            case XMODEM_COMPLETED:
                return(XMODEM_COMPLETED);                   

            case XMODEM_OK:
                *snd_len += xv->DataLen;
                break;
        }
		nop();
    }                                             
    return(XMODEM_COMPLETED);                   
}     



/***********************************************************************
 * Procedure  : XMODEM_RcvFrame(PRIVATE)                               *
 * Input      : timeout (in ms)                                        *
 * Output     : result code.                                           *
 * Description: Receive a XMODEM Frame (with timeout).                 *
 * Return Value:                                                       *
 *    XMODEM_CANCEL     : remote send CAN to cancel transfer           *
 *    XMODEM_ERROR      : an invalid packet is received                *
 *    XMODEM_COMPLETED  : a valid packet is received                   *
 *    XMODEM_TIMEOUT    : timeout reached                              *
 *    XMODEM_END        : a good packet is received                    *
 ***********************************************************************/
PRIVATE uint8 XMODEM_RcvFrame(void)
{
    uint8  reply,d;
    uint8  GetPkt = 0;
    uint8  errorflag=FALSE;
    sint    ch;
    uint16 Check=0,CheckSum;
	uint16 i = 0;
    
	xv = &XV;
    GetPkt = FALSE;

    while( (!errorflag) && (! GetPkt) )
    {
        XMODEM_Read1Byte(&ch);
//	#ifdef Frank_test	
//		if((ch==XV_TIMEOUT) &&(GetPkt == 0))
//			{
//			errorflag=TRUE;
//			}
//		else
//			{
//			xv->PktBuf[i++] = (uint8)(ch&0xff);
//			GetPkt ++;
//			}

//      if((GetPkt > 0) &&(ch==XV_TIMEOUT))
//	  	nop();
//		if(i > 10)
//		break;
//	}
//	if(errorflag)		
//			return (XMODEM_SendNAK());
//	else
//		return XMODEM_OK;
//	#else
        switch (xv->PktReadMode)
        {
            case XpktSOH:
                if (ch==SOH)
                {
                    xv->PktBuf[0] = (uint8)(ch&0xff);
                    xv->PktReadMode = XpktBLK;
                    if (xv->XOption==Xopt1K) XMODEM_SetOption(XoptCRC);
                    xv->TimeOut=xv->TOutShort;
                }
                else if (ch==STX)
                {
                    xv->PktBuf[0] = (uint8)(ch&0xff);
                    xv->PktReadMode = XpktBLK;
                    XMODEM_SetOption(Xopt1K);
                    xv->TimeOut=xv->TOutShort;
                }
                else if (ch==EOT)
                {
                    pause(2);
                    reply = ACK;
                    XMODEM_Write(&reply, 1);
                    flush_alt485();
                    return XMODEM_COMPLETED;
                }
                else if (ch==CAN)
                {
                    XMODEM_Read1Byte(&ch);
                    if( (ch&0xff) == CAN ) return XMODEM_CANCEL;
                    else   errorflag=TRUE;
                }
                else if(ch==XV_TIMEOUT)
                {
                    errorflag=TRUE;
                }
                break;
            
            case XpktBLK:
                xv->PktBuf[1] = (uint8)(ch&0xff);
                xv->PktReadMode = XpktBLK2;
                xv->TimeOut=xv->TOutShort;
				if(xv->PktBuf[1] == 0x02)
					nop();
                break;
                
            case XpktBLK2:

                xv->PktBuf[2] = (uint8)(ch&0xff);
                if ((xv->PktBuf[2] ^ xv->PktBuf[1]) == 0xff)
                {
                    xv->PktBufPtr = 3;
                    xv->PktBufCount = xv->DataLen;                    
                    xv->PktReadMode = XpktDATA;
                    xv->TimeOut=xv->TOutShort;
                    Check=0;
                }
                else
                {
                    errorflag=TRUE;
                }
                break;
            
            case XpktDATA:
                if(ch!=XV_TIMEOUT) 
					
                {

					xv->PktBuf[xv->PktBufPtr] = (uint8)(ch&0xff);
                    if (xv->CheckLen==1)                /* CheckSum */
                        Check += (uint8)(xv->PktBuf[xv->PktBufPtr]);
                    else                                /* CRC */
                        Check = Calculate_CRC(Check,
                                (uint8)xv->PktBuf[xv->PktBufPtr]);

                    xv->PktBufPtr++;
                    xv->PktBufCount--;
                    GetPkt = xv->PktBufCount==0;
                    if (GetPkt)
                        xv->PktReadMode = XpktSOH;
                    else
                        xv->TimeOut=xv->TOutShort;
                }
                else
                {
					
					errorflag=TRUE;
                }
                break;
        }
    }
    
    if(!errorflag)
    {
        if ((xv->PktBuf[1]==0) && (xv->PktNum==0) && (xv->PktNumOffset==0))
        {
            if (xv->NAKMode==XnakNAK)
                xv->NAKCount = 10;
            else
                xv->NAKCount = 3;
            XMODEM_SendNAK();
            return XMODEM_NOTHING;
        }
    
        if (xv->CheckLen==1)                /* CheckSum */
        {
            XMODEM_Read1Byte(&ch);
            CheckSum =ch&0xff;
            Check&=0xff;
        }
        else                                /* CRC */
        {   
            XMODEM_Read1Byte(&ch);
            CheckSum=(ch&0xff)<<8;
            XMODEM_Read1Byte(&ch);
            CheckSum+=(ch&0xff);
        }
        if (Check!=CheckSum)
        {
        	pause(2);
            XMODEM_SendNAK();
            return XMODEM_NOTHING;
        }
    
        d = xv->PktBuf[1] - xv->PktNum;
        if (d>1)
        {
        	pause(2);
            XMODEM_Cancel();
            return XMODEM_CANCEL;
        }

        xv->NAKMode = XnakC;//XnakC;//XnakNAK;
        xv->NAKCount = 3;//3;//10;
    
        if (d==0) return XMODEM_NOTHING;
        xv->PktNum = xv->PktBuf[1];
        if (xv->PktNum==0)
        xv->PktNumOffset = xv->PktNumOffset + 256;
    
        xv->TimeOut=xv->TOutLong;
        return XMODEM_OK;
    }
    else
    {
    	pause(2);
        return (XMODEM_SendNAK());
    }
//	#endif
}

/************************************************************************
 * Procedure  : XMODEM_RcvBuffer(public)
 * Input      : Destination Pointer & Max. Data Length
 * Output     : result code.
 * Description: Start a XMODEM Receive operation to receive a block of data.
 *
 * Return Value:
 *    XMODEM_COMPLETED  : Buffer Receive OK.
 *    XMODEM_CANCEL     : Remote Cancel Operation. Receive Unfinished.
 ************************************************************************/
PUBLIC uint8 XMODEM_RcvBuffer(uint8 *data_ptr, uint32 data_length, uint32 *rcv_len)
{
    uint16 cpy_length;
    uint8  ret;
	uint8 reply;
	#ifdef SF_Xmodem
	uint32 addr = (uint32)data_ptr;
	#endif

	xv = &XV;
    PUT_Message("\n\rStarting XMODEM upload ");
    SET_TIMEOUT(0);
    switch (Xmodem_Mode)
    {                                       /* default CRC mode */
    case CHECKSUM_MODE:
        ret = 'K';
        PUT_Message("(checksum mode)....\n\r");
        break;
    case CRC_MODE:
    default:
        PUT_Message("(CRC mode)....\n\r");
        ret = 'C';
        break;
    }
    xv->XMode = XMODEM_RECEIVE;
    xv->PktNumOffset = 0;
    xv->PktNum = 0;
    xv->PktNumSent = 0;
    xv->PktBufCount = 0;
    xv->TOutShort = TimeOutShort;
    xv->TOutLong = TimeOutLong;
    xv->NAKCount = 50;

    if(ret=='C') {
        xv->XOption=XoptCRC;
        xv->NAKMode=XnakC;
    }
    else {
        xv->XOption=XoptCheck;
        xv->NAKMode=XnakNAK;
    }
    XMODEM_SetOption(xv->XOption);
    
    *rcv_len = 0;
    flush_alt485();
    XMODEM_SendNAK();    
    while (1)
    {
        ret=XMODEM_RcvFrame();       
        switch (ret)
        {
            case XMODEM_OK:
                if (data_length < xv->DataLen)
                    cpy_length = data_length;
                else
                    cpy_length = xv->DataLen;
                *rcv_len += cpy_length;
				
                #ifndef SF_Xmodem
		        pause(2);				
                memcpy(data_ptr, &xv->PktBuf[3], cpy_length);
				data_ptr += cpy_length;
				#else
                //program data(128 or less bytes) to serial flash
                FlashPageProgram((uint32)addr,&xv->PktBuf[3],cpy_length);
				addr +=cpy_length;
				#endif
		
                data_length -= cpy_length;
				
                reply = ACK;                           /* send ACK */
                XMODEM_Write(&reply, 1);
				
                break;
                
            case XMODEM_TIMEOUT:
                pause(1);
                return XMODEM_TIMEOUT;

            case XMODEM_CANCEL:
                pause(1);
                return XMODEM_CANCEL;

            case XMODEM_COMPLETED:
                pause(1);
				
                return(XMODEM_COMPLETED);
        }
    }

    return(XMODEM_COMPLETED);
}


PRIVATE uint8 XMODEM_SendNAK(void)
{
    uint8 ch;
	xv = &XV;

    /* flush comm buffer */
    flush_alt485();
    
    xv->NAKCount--;
    if (xv->NAKCount<0)
    {
#if 0
        if (xv->NAKMode==XnakC)
        {
            XMODEM_SetOption(XoptCheck);
            xv->NAKMode = XnakNAK;
            xv->NAKCount = 9;
        }
        else
#endif
        {
            XMODEM_Cancel();
            return (XMODEM_TIMEOUT);
        }
    }
    if (xv->NAKMode==XnakNAK)
    {
        ch = NAK;
        if ((xv->PktNum==0) && (xv->PktNumOffset==0))
            xv->TimeOut = TimeOutInit;
        else
            xv->TimeOut = xv->TOutLong;
    }
    else 
    {
        ch = 'C';
        xv->TimeOut = TimeOutInit;
    }
    XMODEM_Write(&ch,1);
    xv->PktReadMode = XpktSOH;
    
    return (XMODEM_NOTHING);
}


PRIVATE void XMODEM_SetOption(uint8 option)
{
	xv = &XV;
    xv->XOption = option;
    
    switch (xv->XOption)
    {
        case XoptCheck:                 /* Checksum */
            xv->DataLen = 128;
            xv->CheckLen = 1;
            break;
        case XoptCRC:                   /* CRC */
            xv->DataLen = 128;
            xv->CheckLen = 2;
            break;
        case Xopt1K:                    /* 1K */
            xv->DataLen = 1024;
            xv->CheckLen = 2;
            break;
    }    
}

 uint8 SF_Timeout = TRUE;  
PRIVATE uint8 XMODEM_Read1Byte(sint *ch)
{
    uint8 Received = FALSE;
	
	xv = &XV;
    SET_TIMEOUT(xv->TimeOut*1000);
    *ch=GET_CHAR(xv->TimeOut);
	if(SF_Timeout == FALSE)
		{
		*ch = -1;
		SF_Timeout = TRUE;
		}

    SET_TIMEOUT(0);
    return (XMODEM_NOTHING);
}

PRIVATE  sint XMODEM_Write(uint8* ptr, uint32 length)
{
    sint i;
    
    i=0;
    while( i < length )
    {
        PUT_CHAR(*ptr++);
        i++;
    }
    return i;
}


PRIVATE void XMODEM_Cancel(void)
{
    uint8 ch;
	xv = &XV;
  
    ch = CAN;
    XMODEM_Write(&ch,1);
    XMODEM_Write(&ch,1);
    ch=BSP;
    XMODEM_Write(&ch,1);
    XMODEM_Write(&ch,1);
    xv->XMode = 0; 
}


PRIVATE uint16 XMODEM_CalcCheck(uint8* PktBuf)
{
    sint     i;
    uint16    Check;
	xv = &XV;

    if (xv->CheckLen==1)                /* CheckSum */
    {
        Check = 0;
        for (i = 0 ; i < xv->DataLen ; i++)
            Check = Check + (uint8)(PktBuf[3+i]);
        return (Check & 0xff);
    }
    else                                /* CRC */
    {   
        Check = 0;
        for (i = 0 ; i < xv->DataLen ; i++)
            Check = Calculate_CRC(Check,PktBuf[3+i]);
        return (Check);
    }
}

PRIVATE uint16 Calculate_CRC(uint16 crc,uint8 ch)
{                                                 
    uint8 i;					 
    uint16 ch1, accum;				
                                                  
    accum = 0;                                    
    ch1 = ((crc >> 8) ^ ch) << 8;                 
    for (i = 8; i > 0; i--)                       
    {                                             
        if ((ch1 ^ accum) & 0x8000)               
            accum = (accum << 1) ^ 0x1021;        
        else                                      
            accum <<= 1;                          
        ch1 <<= 1;                                
    }                                             
    return( (crc << 8) ^ accum);                  
} 

/************************************************************************
 *                           End of XMODEM.C
 ************************************************************************/

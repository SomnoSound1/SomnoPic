//##############################################################################
//	filename:		HSU_PROT.c
//
// 	Implements the HSU-protocol Communication
//
//##############################################################################
//
//  	Author:			V.SchK
//  	Company:		HS-Ulm
//
//	Revision:		x.x
//	Date:			Jan. 2006
//	Assembled using		MPLAB 7.30 C18 3.01
//
//	SchK	12.01.06 	start the project
//	SchK
//
//
//	todo	- ?
//		-
//		-
//
//##############################################################################
//
/** I N C L U D E S ***********************************************************/
#include <p18cxxx.h>
#include <usart.h>
#include <delays.h>
#include <timers.h>
#include <spi.h>
#include <adc.h>
#include <string.h>
#include <stdlib.h>

#include "hsuprot.h"
//#include "global_def.h"

//#pragma udata access ACCESSRAM
#pragma udata
/** P U B L I C  V A R I A B L E s ********************************************/
struct serialPck inPck;
struct serialPck outPck;

//#pragma udata

/** P R I V A T E  V A R I A B L E S ******************************************/


/** G L O B A L  P R O T O T Y P E S ****************************************/
//void HSUprotInit(void);
//void RXbyteImport(char newByte);
//void SendPck(void);

//## P R I V A T E  P R O T O T Y P E S ########################################


/////////////////////////////////////////////////////////////////////////
//                           HSUprotInit()                             //
/////////////////////////////////////////////////////////////////////////

void HSUprotInit()
{
    inPck.Flags.ALL = 0;
    inPck.Flags.PCKCMPL = 1;
    outPck.Flags.ALL = 0;
}


/////////////////////////////////////////////////////////////////////////
//                            RXbyteImport()                           //
/////////////////////////////////////////////////////////////////////////

void RXbyteImport(char newByte)
{
    unsigned char j;

    switch (newByte) {
        case STARTFLAG:                             // always do (STARTFLAG)
            inPck.ID.all = 0;                       //  clear IDenntity Byte
            inPck.Size = 0;                         //  reset DataBuffer
            inPck.ChkSum = 0;
            if (inPck.Flags.PCKCMPL){               // prev. Pack. complete ?
                inPck.Flags.ALL = 0;                //  reset Command Flags
            } else {                                //   no -> StopError
                inPck.Flags.ALL = 0;                //  reset Command Flags
                return;
            }
            break;
        case ESCFLAG:
            if ((inPck.Flags.PCKCMPL) || (inPck.Flags.PCKESC)) { // new cmd must start with STARTFLAG
                inPck.Flags.PERRCMD = 1;    // error  ???
                return;
            } else
                inPck.Flags.PCKESC = 1;             // set EscapeFlag
            break;
        case STOPFLAG:
            if ((inPck.Flags.PCKCMPL) || (inPck.Size < (unsigned) 1)) {
                inPck.Flags.PERRSTART = 1;          // missing StartFlag
                return;
            } else {
                inPck.ChkSum = inPck.ID.all;        // calc. CHKSUM
                for (j = 0; j < (inPck.Size - 1); j++) {
                    if ((inPck.Bytes[j] == STARTFLAG)
                            || (inPck.Bytes[j] == STOPFLAG)
                            || (inPck.Bytes[j] == ESCFLAG)
                            )
                        inPck.ChkSum += (inPck.Bytes[j] & ~ESCMASK);
                    else
                        inPck.ChkSum += inPck.Bytes[j];
                }
            }
            if (inPck.ChkSum != inPck.Bytes[--inPck.Size]) {
                inPck.Flags.PERRCHKSUM = 1;         // CheckSumError
                return;
            } else {
                inPck.Flags.PCKCMPL = 1;            // set StopFlag
                inPck.Flags.CMDTODO = 1;            //  flag -> main
                return;
            }//else (!missing startflag)
            break;
        default:
            if (inPck.Flags.PCKCMPL) {              // StartError
                inPck.Flags.PERRSTART = 1;          //   (missing flag)
                return;
            }
            if (inPck.Flags.ALL == (unsigned) 0)    // new transmission
            { //   -> this is PackageID
                inPck.Flags.PCKID = 1;              //   set flag
                inPck.ID.all = newByte;
                if (inPck.Flags.PCKESC)             //   check EscapeFlag
                {
                    inPck.ID.all |= ESCMASK;        //   apply EscapeMask
                    inPck.Flags.PCKESC = (unsigned) 0; //   clear EscapeFlag
                }
            } else // running transmisssion
            {
                inPck.Bytes[(++inPck.Size) - 1] = newByte;
                if (inPck.Flags.PCKESC)             //   check EscapeFlag
                {
                    inPck.Bytes[inPck.Size - 1] |= ESCMASK;
                    inPck.Flags.PCKESC = 0;         //   clear EscapeFlag
                }
            }
            break;                                  // default
    }//switch (newByte)
    //return;
}
/////////////////////////////////////////////////////////////////////////
//                              SendPck()                              //
/////////////////////////////////////////////////////////////////////////
void SendPck(struct serialPck sendPck)
{
//	overlay
    unsigned char j;

    while(!TXSTA1bits.TRMT1);
    TXREG1 = STARTFLAG;                      // STARTFLAG
    while(!TXSTA1bits.TRMT1);
    TXREG1 = sendPck.ID.all;                // ID (must not be a FLAG !!!)
    while(!TXSTA1bits.TRMT1);
    TXREG1 = PCK_SIZE+4; //+START +ID +SIZE +STOP
    //sendPck->ChkSum = sendPck->ID.all;

    for (j=0; j<sendPck.Size; j++)
    {
//        if ((sendPck->Bytes[j]==STARTFLAG)  // check the bytes for flags
//           ||(sendPck->Bytes[j]==STOPFLAG)  //   if a flag -> send the ESCAPE
//           ||(sendPck->Bytes[j]==ESCFLAG))  //   and mask the byte
//        {                                   //   then transmit it
//            while(!TXSTA1bits.TRMT1);
//            TXREG1 = ESCFLAG;
//            sendPck->Bytes[j] &= ~ESCMASK;
//        }
        while(!TXSTA1bits.TRMT1);
        TXREG1 = sendPck.Bytes[j];
        //sendPck->ChkSum += sendPck->Bytes[j];
    }
//    if ((sendPck->ChkSum==STARTFLAG)        // check the checksum for flags
//      ||(sendPck->ChkSum==STOPFLAG)         //   if a flag -> send the ESCAPE
//      ||(sendPck->ChkSum==ESCFLAG))         //   mask the checksum
//    {                                       //   then transmit it
//        while(!TXSTA1bits.TRMT1);
//        TXREG1 = ESCFLAG;
//        sendPck->ChkSum &= ~ESCMASK;
//    }

    while(!TXSTA1bits.TRMT1);
    TXREG1 = STOPFLAG;                       // STOPTFLAG
    sendPck.Flags.CMDTODO = 0;
}


/////////////////////////////////////////////////////////////////////////
//                              MakePck()                              //
/////////////////////////////////////////////////////////////////////////

void MakePck(struct serialPck* makeBuffer)
{
    //	overlay
    unsigned char i;

    outPck.Bytes[0] = STARTFLAG;                    // STARTFLAG

    if ((makeBuffer->ID.all == STARTFLAG)
            || (makeBuffer->ID.all == STOPFLAG)     // ID != FLAG !!!
            || (makeBuffer->ID.all == ESCFLAG))
        return; // false;

    outPck.ID.all = makeBuffer->ID.all;
    outPck.Bytes[1] = makeBuffer->ID.all;
    outPck.Size = 2;
    outPck.ChkSum = makeBuffer->ID.all;

    for (i = 0; i < makeBuffer->Size - 1; i++)      // DATA
    {
        if ((makeBuffer->Bytes[i] == STARTFLAG)
                || (makeBuffer->Bytes[i] == STOPFLAG)
                || (makeBuffer->Bytes[i] == ESCFLAG))
        {
            outPck.Bytes[outPck.Size++] = ESCFLAG;
            if (!outPck.Size)
                return; // false;                   // overflow occured
            makeBuffer->Bytes[i] &= ~ESCMASK;
        }
        outPck.Bytes[outPck.Size++] = makeBuffer->Bytes[i];
        if (!outPck.Size)
            return; // false;                       // overflow occured
        outPck.ChkSum += makeBuffer->Bytes[i];      // calc. chkSum;
    }

    if ((outPck.ChkSum == STARTFLAG)
            || (outPck.ChkSum == STOPFLAG)
            || (outPck.ChkSum == ESCFLAG))
    {
        outPck.Bytes[outPck.Size++] = ESCFLAG;
        outPck.ChkSum &= ~ESCMASK;
    }
    outPck.Bytes[outPck.Size++] = outPck.ChkSum;
    outPck.Bytes[outPck.Size++] = STOPFLAG;
    if (outPck.Size < (unsigned) 4)
        return; // false;                           // overflow occured
    //  makeBuffer = outPck;

    outPck.Flags.CMDTODO = 1;
    return; // true;
}


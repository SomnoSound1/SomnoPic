/* 
 * File:   hsuprot.h
 * Author: kalkbren
 *
 * Created on 24. September 2014, 14:19
 */

#ifndef HSUPROT_H
#define	HSUPROT_H

//#include "global_def.h"

//## D E F I N I T I O N S #####################################################

#define PCK_SIZE	52	// ->global_def.h

/*
//## Pins (if USART not used) define them in "global_def.h"
#define RS_nRES		1	// out request to send
#define RS_nRTS		1	// out request to send
#define RS_nCTS		0	// in clear to send
#define RS_nDTR		1	// out data terminal ready
#define RS_nDCD		1	// in data carrier detected
#define RS_nDSR		1	// in data set ready
#define RS_nRI		1	// in ring index

//## Pins define them in "global_def.h"
#define RS_nRES		PORTxbits.Rxy	// out request to send
#define RS_nRTS		PORTxbits.Rxy	// out request to send
#define RS_nCTS		PORTxbits.Rxy	// in clear to send
#define RS_nDTR		PORTxbits.Rxy	// out data terminal ready
#define RS_nDCD		PORTxbits.Rxy	// in data carrier detected
#define RS_nDSR		PORTxbits.Rxy	// in data set ready
#define RS_nRI		PORTxbits.Rxy	// in ring index
*/

//##############################################################################

struct serialPck // structSerialPck
{
    union {
        unsigned char all;
        struct {
            unsigned CH1 : 1; // channel 1 included
            unsigned CH2 : 1; // channel 2 included
            unsigned CH3 : 1; // channel 3 included
            unsigned CH4 : 1; // channel 4 included
            unsigned CH5 : 1; // channel 5 included
            unsigned SIZE : 1; // PACKAGESIZE instead ESCAPEFLAGS and CHECKSUM
            unsigned TIME : 1; // time channel included
            unsigned MESSAGE : 1; // MSB = 1 -> messsage ; MSB = 0 -> data
        };
    } ID;

    unsigned char Size;
    unsigned char Bytes[PCK_SIZE];
    unsigned char ChkSum;

    union {
        struct {
            unsigned PERRSTOP : 1; // communication record errors
            unsigned PERRSTART : 1;
            unsigned PERRCMD : 1;
            unsigned PERRCHKSUM : 1;
            unsigned PCKESC : 1; // previous byte was an escapeflag
            unsigned PCKID : 1; // IDentity byte received
            unsigned CMDTODO : 1; // ToDo (packet by main)
            unsigned PCKCMPL : 1; // Complete (previous packet complete)
        };
        struct {
            unsigned PERR : 4;
            unsigned free : 4;
        };
        unsigned char ALL;
    } Flags;
};

#define SIZE_1_DATA (unsigned)1
#define SIZE_2_DATA (unsigned)2
#define SIZE_3_DATA (unsigned)3
#define SIZE_4_DATA (unsigned)4



//Message-Data IDs
#define IDMESSAGE   0x80    // MSB = 1 -> messsage ; MSB = 0 -> data
#define IDTIME      0x40    // time channel included
#define	IDSIZE      0x20    // PACKAGESIZE instead of ESCAPEFLAGS and CHECKSUM
#define	IDCH5       0x10    // channel 5 included
#define	IDCH4       0x08    // channel 4 included
#define	IDCH3       0x04    // channel 3 included
#define	IDCH2       0x02    // channel 2 included
#define	IDCH1       0x01    // channel 1 included

typedef enum {              // MESSAGE IDs

    DID_CH1     = 0x01,
    DID_CH2     = 0x02,
    DID_CH3     = 0x04,
    DID_CH4     = 0x08,
    DID_CH5     = 0x10,
    DID_SIZE    = 0x20,
    DID_TIME    = 0x40,
    MSG_BIT     = 0x80,
    MID_BUSY    = 0x8F,
    MID_ERROR   = 0xFF
}TYPE_MSG_ID;

union ComFlags {

    struct {
        unsigned PERRSTOP       : 1; // communication package errors
        unsigned PERRSTART      : 1;
        unsigned PERRCMD        : 1;
        unsigned PERRCHKSUM     : 1;
        unsigned ERRCMD         : 1; // unknown command
        unsigned ERROVERFLOW    : 1; // Overflow error
        unsigned ERRFRAME       : 1; // Framing error
        unsigned DATAOVERFLOW   : 1; // Error
    };
    unsigned char ALL;
};

//## G L O B A L  V A R I A B L E s ############################################
extern struct serialPck inPck;
extern struct serialPck outPck;

//## G L O B A L  P R O T O T Y P E S ##########################################
//void HSUprotUSARTinit(void);
void HSUprotInit(void);
void RXbyteImport(char newByte);
void MakePck(struct serialPck* makeBuffer);
void SendPck(struct serialPck sendBuffer);


//## C O M M U N I C A T I O N   F L A G S #####################################
#define STARTFLAG	0xAA
#define STOPFLAG	0xAB
#define ESCFLAG		0xAF
#define ESCMASK		0x80

// Sensor Error-Data (bits)
#define	SERR		0x80
#define	SERR_FRAME	0x40
#define	SERR_OVR	0x20
//#define	SERR...		0x10
#define	SERR_CHKS	0x08
#define	SERR_CMD	0x04
#define	SERR_START	0x02
#define	SERR_STOP	0x01

//CommError bits
#define	ERRFLAG         0xFF

#define IDERRROR	0xFF

#endif


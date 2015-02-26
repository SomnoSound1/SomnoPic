#include <p18cxxx.h>
#include <delays.h>
#include <timers.h>
#include <spi.h>
#include <usart.h>
#include <adc.h>
#include <string.h>
#include <stdlib.h>

#include "uC_main.h"
#include "functions.h"
#include "hsuprot.h"
#include "global_def.h"

#pragma udata
const far rom char strDevice[]  = "SomnoSound";
const far rom char strVersion[] = "Version 0.1";
const far rom char strDate[]    = "26. September 2014";
const far rom char strManu[]    = "HS-ULM";
const far rom char strMisc[]    = "CK";
unsigned char TM0_LowByte, TM1_LowByte;
unsigned char TM0_HightByte, TM1_HightByte;
union Timers timer0, timer1;
struct s_dataBuffer  dataBuffer;     // daten fuer PC
unsigned char buff_len;
unsigned char tx_buffer[30];
unsigned char rx_count;
unsigned char rx_buffer[50];
unsigned char i, tick_counts;
unsigned char dev_id;
char FS_SEL, AFS_SEL, MOT_EN, MOT_THR, MOT_DUR, MOT_INT;

//#pragma code high_vector=0x08
//#pragma interrupt high_isr
//void high_isr(void) {

#pragma code HIGH_INTERRUPTS_VECTOR = 0x08
void interrupt_HIGH (void){

    //AD-Wandler fertig
    if(PIE1bits.ADIE && PIR1bits.ADIF)
    {
        if (mode == 1 && WT12_CON)
        GetData();        
        PIR1bits.ADIF = 0;
    }

//    // INT0 von INT-Ausgang MPU6000
//    if( (INTCONbits.INT0IF == 1) && (INTCONbits.INT0IE == 1))
//    {
//        INTCONbits.INT0IF = 0;	//Interruptbit von INT0 zurücksetzen
//        LED_BLAU = 1;
//    }

    // USART1 receive-interrupt, wird von BT-Modul bedient
    if((PIR1bits.RC1IF == 1) && (PIE1bits.RC1IE == 1))
    {
        PIR1bits.RC1IF = 0;
        RXbyteImport(RC1REG);
        
        if((inPck.Flags.PCKCMPL)&& (inPck.ID.MESSAGE))
        {
            //OPEN
            if(inPck.Bytes[0] == 0x03 );{
            mode = 1;}
            //INIT
            if(inPck.Bytes[0] == 0x00);{
            SendMessage(5);}
            //DEVICE
            if(inPck.Bytes[0] == 0x0F);{
            SendMessage(strDevice);
                                       }
        }
    }

    _asm
    retfie 1
    _endasm
}

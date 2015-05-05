//##############################################################################
//    	filename:        	
//
//     	main file for demo projects
//
//##############################################################################
//
//##############################################################################
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

//--- C O N F I G U R A T I O N  B I T S --------------------------------------
#pragma config FOSC = INTIO67
#pragma config WDTEN = OFF
#pragma config PWRTEN = OFF
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config DEBUG = ON

#pragma udata
char mode;
union u_status statusData;     // (siehe oben)

//--- P R I V A T E   P R O T O T Y P E S --------------------------------------
void __init();

//##############################################################################
// Function:        void __main(void)
//					called from the c18 startup code
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void main()
{
    //Timer anschalten
    //T1CONbits.TMR1ON = 1;
    while(1)
    {
        //AD-Konverter ausschalten
        ADCON0bits.ADON = 0;

        mode = 0;
        LED_GRUEN = 1;

        // Hier warten bis wir verbunden sind
        while(!WT12_CON)
            if(LED_BLAU) LED_BLAU = 0;

        LED_BLAU = 1;

        // USART1 receive interrupt enable, wenn BlueTooth verbunden
        InitBluetooth();
        mode = 0;
        // wir warten, bis über BT ein mode command kommt
        while(mode == 0 && WT12_CON);

        LED_GRUEN = 0;

        //AD-Konverter anschalten
        ADCON0bits.ADON = 1;
        
        while(mode == 1 && WT12_CON)
        {
            //Send Data
            if(statusData.newData){
                //TESTPIN = !TESTPIN;                
                ReadIMUData();
                AddPackageNumber();
                AddSyncTrigger();
                //ReadIMUDataTest();
                TransmitDataViaBT();
            }
        }

    }
}







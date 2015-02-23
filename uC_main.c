//##############################################################################
//    	filename:        	
//
//     	main file for demo projects
//
//##############################################################################
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Debugging nicht möglich weil zweite serielle Schnittstelle auf PGD/PGC
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
#include "hsuprot.h"
//#include "hsuprot.h"

//--- C O N F I G U R A T I O N  B I T S --------------------------------------
#pragma config FOSC = INTIO67
#pragma config WDTEN = OFF
#pragma config PWRTEN = OFF
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config DEBUG = ON

//--- P R I V A T E   P R O T O T Y P E S --------------------------------------
void __init();

#define    FOSC        16000000  // internal clock
#define    BAUDRATE    230400


#define    SAMPRATE    1385
//#define    SAMPRATE    750
#define    PRESCALE    T0_PS_1_32
#define    PRETEILER   32

#define FlashSize 0x2000000      // 32 MB


const far rom char strDevice[]  = "SomnoSound";
const far rom char strVersion[] = "Version 0.1";
const far rom char strDate[]    = "26. September 2014";
const far rom char strManu[]    = "HS-ULM";
const far rom char strMisc[]    = "CK";

union Timers timer0, timer1;
unsigned char TM0_LowByte, TM1_LowByte;
unsigned char TM0_HightByte, TM1_HightByte;
unsigned int spbrg;
unsigned char go_on, i, tick_counts;

char Testbyte = 3;
unsigned char buff_len;
unsigned char tx_buffer[30];
unsigned char rx_count;
unsigned char rx_buffer[50];

unsigned char connected, mode;
unsigned char dev_id;
unsigned char data[14];
char FS_SEL, AFS_SEL, MOT_EN, MOT_THR, MOT_DUR, MOT_INT;

unsigned char Manuf, Dev1, Dev2, status;
unsigned long int addr, old_addr;
unsigned int sector, old_sector;

int batt_voltage;
char POWER_OK;

union u_status       statusData;     // (siehe oben)
struct s_dataBuffer  dataBuffer;     // daten fuer PC

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
    addr = 0x00000000;
    old_addr = addr;
    sector = 0;
    old_sector = sector;

    go_on == 0;
    connected = 0;
    mode = 0;

//------------------------------------------------------------------------------
    spbrg = 16;
    //spbrg = 34;
   // USART2 receive interrupt enable
    Open2USART( USART_TX_INT_OFF
            & USART_RX_INT_ON
            & USART_ASYNCH_MODE
            & USART_EIGHT_BIT
            & USART_CONT_RX
            & USART_BRGH_HIGH,
            spbrg);

    BAUDCON2bits.BRG16 = 1;
//------------------------------------------------------------------------------
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

    // wait until WT12 is connected
    // connected will be set on interrupt on change
    while(connected == 0)
    {
        // mode command kam über USB, dann brauchen wir das BlueTooth modul nicht
        if(mode != 0)
            break;
    }

    // USART1 receive interrupt enable, wenn BlueTooth verbunden
    if(connected == 1)
    {
            Open1USART( USART_TX_INT_OFF &
        	USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT & 	// Transmission Width 8 bit
		USART_CONT_RX &		// Reception mode
		USART_BRGH_HIGH &
                USART_SYNC_SLAVE &
                USART_ADDEN_OFF,
		spbrg);

        BAUDCON1bits.BRG16 = 1;

        mode = 0;

        // wir warten, bis über BT ein mode command kommt
        while(mode == 0)
        {

        }
    }

        while(POWER_OK == 1)
    {
        // warte auf den timer-interrupt
//        while(go_on == 0);
//        go_on = 0;

        // die komplette Datenaufnahme und Übertragung (bis TESTPIN = 0)
        // dauert hier zwischen 2 und 2.5 ms (je nach Maskierung der Daten)
//        TESTPIN = 1;

        // Sensordaten lesen
        // Sensor lesen (14 Byte), Daten auf Flash schreiben und via BlueTooth übertragen
        // dauert in Summe ca. 5ms

        // read 14 bytes from MPU6000
        // 3*2byte accel(xyz), 3*2byte gyro(xyz), 2 byte temperature
//        if((mode & 1) == 1)
//            //

        // daten in flash schreiben
//        if((mode & 8) == 8)
//            WriteToFlash(addr, data, 14);
//
//        // daten aus flash lesen
//        if((mode & 16) == 16)
//            ReadFromFlash(addr, data, 14);
//
        // daten via BlueTooth übertragen
        if(((mode & 3) == 3) && statusData.newData){
            TransmitDataViaBT();
        }

        //        // daten via USB übertragen
//        if((mode & 4) == 4)
//            TransmitDataViaUSB(data, 14);
//
//        // beim Schreiben oder Lesen des Speichers, Adresse um 14 Byte erhöhen
//        if( ((mode & 8) == 8) || ((mode & 16) == 16) )
//            IncrementFlashAddr(14);

        // Sleep-Mode
//        if((mode & 32) == 32)
//            GoToSleep();

//        TESTPIN = 0;
    }


    T1CONbits.TMR1ON = 0;
    Close1USART();
    Close2USART();
    CloseSPI1();
    CloseSPI2();
    CloseADC();
}
// ----------------------------------------------------------------------------
#pragma code HIGH_INTERRUPTS_VECTOR = 0x08
void interrupt_HIGH (void)
{
    //Timer0
//    if(INTCONbits.TMR0IF && INTCONbits.TMR0IE)
//    {
//	TMR0H = TM0_HightByte;	// Write high byte to Timer0
//  	TMR0L = TM0_LowByte;	// Write low byte to Timer0
//        ConvertADC();
//
//    }

    //Timer1
    if(PIR1bits.TMR1IF  && PIE1bits.TMR1IE)
    {      
        ADCON0bits.GO_DONE = 1;  //Start conversion

        //while(ADCON0bits.GO_DONE); //wait for the conversion to finish
        TMR1H = TM1_HightByte;	// Write high byte to Timer0
  	TMR1L = TM1_LowByte;	// Write low byte to Timer0
        PIR1bits.TMR1IF = 0;
        
    }

    //AD-Wandler
    if(PIE1bits.ADIE && PIR1bits.ADIF)
    {      
       
        TESTPIN = 1;
        ReadSensorData();
        //ReadSensorDataTest();
        if(dataBuffer.idx == MAX_AUDIO || dataBuffer.idx >=(MAX_AUDIO+MAX_DATA))
        {
            //ReadIMUDataTest();
            ReadIMUData();
        }        
        if(dataBuffer.idx == MAX_DATA){
            if(statusData.newData)
                statusData.dataOVF = 1;
            statusData.newData = 1;
        }
        if(dataBuffer.idx >= (MAX_DATA*2)){
            if(statusData.newData)
                statusData.dataOVF = 1;
            dataBuffer.idx = 0;
            statusData.newData = 1;
        }
        TESTPIN = 0;        
        PIR1bits.ADIF = 0;
    }

    // INT0 von INT-Ausgang MPU6000
    if( (INTCONbits.INT0IF == 1) && (INTCONbits.INT0IE == 1))
    {
        INTCONbits.INT0IF = 0;	//Interruptbit von INT0 zurücksetzen
        LED_BLAU = 1;
    }

    // interrupt on change on RB5 -> WT12 connection
    if( (INTCONbits.RBIF == 1) && (IOCBbits.IOCB5 == 1))
    {
        INTCONbits.RBIF = 0;	//Interruptbit zurücksetzen
        connected = WT12_CON;

        LED_BLAU = connected;
        LED_GRUEN = !connected;

        //Falls die Verbindung beendet wird oder abbricht wird der Sensor gerebooted.
        if (!connected)
        {
            Reset();
        }
        

    }

    // USART1 receive-interrupt, wird von BT-Modul bedient
    if((PIR1bits.RC1IF == 1) && (PIE1bits.RC1IE == 1))
    {
        PIR1bits.RC1IF = 0;
        RXbyteImport(RC1REG);
        //mode = 3;
        if((inPck.Flags.PCKCMPL)&& (inPck.ID.MESSAGE))
        {
            //OPEN
            if(inPck.Bytes[0] == 0x03 );{
            mode = 3;}
            //INIT
            if(inPck.Bytes[0] == 0x00);{
            SendMessage(5);}
            //DEVICE
            if(inPck.Bytes[0] == 0x0F);{
            SendMessage(strDevice);}

        }
    }

    // USART2 receive-interrupt, wird von USB bedient
    if((PIR3bits.RC2IF == 1) && (PIE3bits.RC2IE == 1))
    {
        PIR3bits.RC2IF = 0;

//        mode = RC2REG;
//
//        if(mode > 31)
//            mode = 0;
//
//        // flash schreiben -> wir retten die Adresse, ab der geschrieben wird
//        if( (mode & 8) == 8)
//        {
//            old_addr = addr;
//            old_sector = sector;
//        }
//
//        // flash lesen -> wir restaurieren die Adresse, ab der geschrieben wurde
//        if( (mode & 16) == 16)
//        {
//            addr = old_addr;
//            sector = old_sector;
//        }
    }

    _asm
    retfie 1
    _endasm
}

//##############################################################################
// Function:        void __init(void)
//
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void __init()
{
    //OSCCON = 0b01100000;  // 8 MHz internal clock
    OSCCON = 0b01110000;    // 16 MHz internal clock, wegen USART mit 115200Baud

    ANSELA = 0b00100001;    // enable analog input on RA5
    TRISA = 0b00100001;     // OSC, OSC, AD_BATT, LED_ROT, PAD2 I/O, Taster, AD_1, AD_0

    ANSELB = 0b00000000;    // disable analog inputs on PORTB
    TRISB = 0b10100101;     // PGD/RX, PGC/TX, WT12_CON, CS2, SDO2, SDI2, SCK2, INT_SENSOR
    WPUBbits.WPUB7 = 1;     // pull up an RB7 enabled (RX)
    WPUBbits.WPUB6 = 1;     // pull up an RB6 enabled (TX)
    //INTCON2bits.RBPU = 0;   // weiß nicht, ob ich das brauche

    ANSELC = 0b00000000;    // disable analog inputs on PORTC
    TRISC = 0b10010000;     // RX, TX, SDO1, SDI1, SCK1, CS1, LED_GRUEN, LED_BLAU

    LED_GRUEN = 0;
    LED_BLAU = 0;
    LED_ROT = 0;

    statusData.all = 0;
    dataBuffer.idx = 0;

    //OUT-Package
    outPck.ID.CH1=1;
    outPck.ID.CH2=1;
    outPck.ID.CH3=0;
    outPck.ID.CH4=0;
    outPck.ID.CH5=0;
    outPck.ID.MESSAGE=0;
    outPck.ID.SIZE=1;
    
    InitInterrupts();

    InitADC();

    //Ohne CCP
    //InitTimer0();
    InitTimer1();
    InitADC0();

    //Mit CCP
    //InitCCP();

    InitWT12();
    InitFlash();
    
    //kurz warten damit IMU immer erfolgreich initiiert wird!    
    InitMPU6000();
    Delay1KTCYx(100);
    InitMPU6000();
    HSUprotInit();
}
// ----------------------------------------------------------------------------
void InitInterrupts(void)
{
    //enable ADC Interrupt
    PIE1bits.ADIE = 1;

    // enable interrupt priority
    RCONbits.IPEN = 1;

    // interrupt on rising edge on RB0
    INTCON2bits.INTEDG0 = 1;
    // enable INT0 (accel. sensor)
    INTCONbits.INT0IE = 1;

    // enable interrupt on change for RB5
    IOCBbits.IOCB5 = 1;

    // enable peripheral interrupts
    INTCONbits.PEIE = 1;
    PIE1bits.RC1IE = 1;
    PIE3bits.RC2IE = 1;
    IPR1bits.RC1IP = 1;

    // enable high and low priority interrupts
    INTCONbits.GIE = 1;
}
// ----------------------------------------------------------------------------
void InitTimer0(void)
{
    timer0.lt = 65536 - (unsigned int)(FOSC/4/PRETEILER/SAMPRATE);

    TM0_HightByte = timer0.bt[1];
    TM0_LowByte = timer0.bt[0];

    TMR0H = TM0_HightByte;  		// Write high byte to Timer0
    TMR0L = TM0_LowByte;                 // Write low byte to Timer0

    OpenTimer0(TIMER_INT_ON&            // Interrupts an
                    T0_16BIT&   	// Timer im 8bit Modus (256 Werte)
                    T0_SOURCE_INT&	// Quelle intern 1Mhz (Taktzyklen)
                    PRESCALE);          // Vorteiler

    // Timer erstmal ausschalten
    T0CONbits.TMR0ON = 0;
}
// ----------------------------------------------------------------------------
void InitTimer1(void)
{
    timer1.lt = 65536 - (unsigned int)(FOSC/4/8/SAMPRATE); // - (unsigned int)(FOSC/4/8/SAMPRATE);
    //timer1.lt = 999999999;

    TM1_HightByte = timer1.bt[1];
    TM1_LowByte = timer1.bt[0];

    TMR1H = TM1_HightByte;  		// Write high byte to Timer0
    TMR1L = TM1_LowByte;                // Write low byte to Timer0


    OpenTimer1(TIMER_INT_ON           // Interrupts an
                & T1_16BIT_RW   	// Timer im 8bit Modus (256 Werte)
                & T1_SOURCE_FOSC	// Quelle intern 1Mhz (Taktzyklen)
                & T1_PS_1_8
                & T1_OSC1EN_OFF
                & T1_SYNC_EXT_OFF, 0);

    // Timer erstmal ausschalten
    T1CONbits.TMR1ON = 0;
}
// ----------------------------------------------------------------------------
void InitADC0(void)
{
    OpenADC(ADC_FOSC_64	& 	// AD Conversion Clock Select Bit
        ADC_RIGHT_JUST	& 	// Result in Least Significant Bits
        ADC_12_TAD,		// Acquisition Time 12 Tad
        ADC_CH0		&  	// Channal 0 für AD Wandlung
        ADC_INT_ON	,	// Interrupt enabled
        ADC_REF_VDD_VSS);

}
// ----------------------------------------------------------------------------
void InitCCP(void)
{
  OpenTimer1(  TIMER_INT_OFF &
            T1_16BIT_RW &
            T1_SOURCE_FOSC_4 &
            T1_PS_1_2 &
            T1_OSC1EN_OFF &
            T1_SYNC_EXT_OFF,
            TIMER_GATE_OFF);
      
    OpenADC(    ADC_FOSC_64 & 	// AD Conversion Clock Select Bit
            ADC_RIGHT_JUST  & 	// Result in Least Significant Bits
            ADC_12_TAD      ,	// Acquisition Time 12 Tad
            ADC_CH0         &  	// Channal 0 für AD Wandlung
            ADC_INT_ON      &	// Interrupt enabled
            ADC_REF_VDD_VSS ,    	// Referenz = Betriebsspannung 0 bis 3,3V;
            ADC_TRIG_CCP5         // Trigger
            );              //Port configuration (all digital)


//Enable Interrupt
T1CONbits.TMR1ON = 1;
CCPTMRS1bits.C5TSEL0 = 0; // timer <-> ccp module (CCP5 / TMR1)
CCPR5 = 65536 - (unsigned int)(FOSC/4/PRETEILER/SAMPRATE); // Fosc/4 / prescaler / Fadc = 1MHz /2 /10Hz
CCP5CONbits.CCP5M = 0b1011; // Compare Mode with Special Event Trigger



}
// ----------------------------------------------------------------------------
void InitWT12(void)
{
    spbrg = 16;
    //spbrg = 34;
    rx_count = 0;
    connected = 0;

    Open1USART( USART_TX_INT_OFF
                    & USART_RX_INT_OFF
                    & USART_ASYNCH_MODE
                    & USART_EIGHT_BIT
                    & USART_CONT_RX
                    & USART_BRGH_HIGH,
                    spbrg);

    BAUDCON1bits.BRG16 = 1;

    Delay1KTCYx(100);

    // dummy AT command
    // das muss ich machen, weil das erste command nach Öffnen der Schnittstelle
    // meist vom BT-Modul nicht verstandne wird und ein sytax error zurückkommt
    strcpypgm2ram(tx_buffer, "AT");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);
/*
    // reset WT12 to default values
    strcpypgm2ram(tx_buffer, "SET RESET");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);

        // reset WT12 to default values
    strcpypgm2ram(tx_buffer, "SET BT PAIR *");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);
*/

    
    // setze den PIN CODE auf 1234
    strcpypgm2ram(tx_buffer, "SET BT AUTH * 1234");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';


    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);

    // mache PIO2 zum carrier detect signal pin (connection)
    // first parameter in HEX: 0x4 = 0b00000100 for POI5
    // sec parameter:   0 -> POI2 = high if there is a connection
    //                  1 -> POI2 = high only in data mode
    strcpypgm2ram(tx_buffer, "SET CONTROL CD 4 0");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);

    //Einstellen der Baudrate
    strcpypgm2ram(tx_buffer, "SET CONTROL BAUD 230400,8N1");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(100);


    // display WT12 values
    strcpypgm2ram(tx_buffer, "SET");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(200);

    Close1USART();
}
// ----------------------------------------------------------------------------
void InitMPU6000(void)
{
    CS_SPI1 = 1;    // chip select CS_SPI1 = 1;

    // max. 1MHz SPI freq
    OpenSPI1(SPI_FOSC_16, MODE_00, SMPMID);
    Delay1KTCYx(10);

    // SPI-mode einschalten -----------------------------------------------
    // USER_CTRL 0x6A + 0x00(write) = 0x6A
    // I2C_IF_DIS = 1 -> SPI-Mode enabled
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x6A);
    Delay1KTCYx(10);
    WriteSPI1(0x10);
    CS_SPI1 = 1;

    Delay1KTCYx(10);

    // Raus aus dem sleep mode --------------------------------------------
    // PWR_MGMT_1 0x6B + 0x00(write) = 0x6B
    // SLEEP = 0 -> raus aus dem sleep mode
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x6B);
    Delay1KTCYx(10);
    WriteSPI1(0x00);
    CS_SPI1 = 1;

    Delay1KTCYx(10);

    // Gib deine ID zurück
    // WHO_AM_I 0x75 + 0x80 (read) = 0xF5
    // gibt I²C-Adresse zurück, default = 104 dec.
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0xF5);
    while(!DataRdySPI1());
    dev_id = ReadSPI1();
    CS_SPI1 = 1;

    Delay1KTCYx(10);

    // Gyrosensor auf full scale +-1000°/s setzen -------------------------
    // GYRO_CONFIG 0x1B + 0x00(write) = 0x1B
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x1B);
    Delay1KTCYx(10);
    //FS_SEL = 0; //-> +- 250°/s
    //FS_SEL = 1; //-> +- 500°/s
    FS_SEL = 2; //-> +-1000°/s  set
    //FS_SEL = 3; //-> +-2000°/s
    WriteSPI1(FS_SEL << 3);
    CS_SPI1 = 1;

    Delay1KTCYx(10);

    // Beschleunigungssensor auf full scale +-8g setzen -------------------
    // ACCEL_CONFIG 0x1C  + 0x00(write) = 0x1C
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x1C);
    Delay1KTCYx(10);
    //AFS_SEL = 0; // -> +- 2g
    //AFS_SEL = 1; // -> +- 4g
    AFS_SEL = 2; // -> +- 8g  set
    //AFS_SEL = 3; // -> +-16g
    WriteSPI1(AFS_SEL << 3);
    CS_SPI1 = 1;

    Delay1KTCYx(10);
    FS_SEL = 0;
    AFS_SEL = 0;
    // zum Test, ob das Setzen der full scale Werte erfolgreich war
    // Gyrosensor full scale auslesen -------------------------
    // GYRO_CONFIG 0x1B + 0x80(read) = 0x1B
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x9B);
    while(!DataRdySPI1());
    FS_SEL = (ReadSPI1() & 0x18) >> 3;
    CS_SPI1 = 1;

    Delay1KTCYx(10);

    // Beschleunigungssensor full scale auslesen -------------------
    // ACCEL_CONFIG 0x1C  + 0x80(read) = 0x1C
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x9C);
    while(!DataRdySPI1());
    AFS_SEL = (ReadSPI1() & 0x18) >> 3;
    CS_SPI1 = 1;

    Delay1KTCYx(100);
/*
//---- INT-TEST ------------------------------------------------------------
    // PWR_MGMT_2 0x6C + 0x00(write) = 0x6C
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x6C);
    Delay1KTCYx(10);
    WriteSPI1(0x00);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // ACCEL_CONFIG 0x1C + 0x00(write) = 0x1C
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x1C);
    Delay1KTCYx(10);
    WriteSPI1(0x00);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // CONFIG 0x1A + 0x00(write) = 0x1A
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x1A);
    Delay1KTCYx(10);
    WriteSPI1(0x00);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // INT_ENABLE 0x38 + 0x00(write) = 0x38
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x38);
    Delay1KTCYx(10);
    WriteSPI1(0x40);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // MOT_DUR 0x20 + 0x00(write) = 0x20
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x20);
    Delay1KTCYx(10);
    WriteSPI1(0x1);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // MOT_THR 0x1F + 0x00(write) = 0x1F
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x1F);
    Delay1KTCYx(10);
    WriteSPI1(0x10);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // MOT_DETECT_CTRL 0x69 + 0x00(write) = 0x69
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x69);
    Delay1KTCYx(10);
    WriteSPI1(0x10);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // CONFIG 0x1A + 0x00(write) = 0x1A
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x1A);
    Delay1KTCYx(10);
    WriteSPI1(0x7);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // PWR_MGMT_2 0x6C + 0x00(write) = 0x6C
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x6C);
    Delay1KTCYx(10);
    WriteSPI1(0x40);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;

    // PWR_MGMT_1 0x6B + 0x00(write) = 0x6B
    CS_SPI1 = 0; //SPI_CS = 0 -> chip enabled;
    WriteSPI1(0x6B);
    Delay1KTCYx(10);
    WriteSPI1(0x20);
    CS_SPI1 = 1; //SPI_CS = 0 -> chip enabled;
// --- ENDE INT-TEST -----------------------------------------------------------
*/
    Delay1KTCYx(10);
}
// ----------------------------------------------------------------------------
void InitFlash(void)
{
    CS_SPI2 = 1;    // chip select CS_SPI2 = 1;

    // max. 1MHz SPI freq
    OpenSPI2(SPI_FOSC_16, MODE_00, SMPMID);
    Delay1KTCYx(10);

    Manuf = 0;
    Dev1 = 0;
    Dev2 = 0;

    // FLASH read identification -------------------
    // RDID 0x9F
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0x9F);
    while(!DataRdySPI2());
    Manuf = ReadSPI2();
    Dev1 = ReadSPI2();
    Dev2 = ReadSPI2();
    CS_SPI2 = 1;

    Delay1KTCYx(10);

    // FLASH read status register -------------------
    // RDSR 0x05
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0x5);
    while(!DataRdySPI2());
    status = ReadSPI2();
    CS_SPI2 = 1;

    Delay1KTCYx(10);
}
// ----------------------------------------------------------------------------
void WriteToFlash(unsigned long int addr, unsigned char* buffer, unsigned char length)
{
    // continously program mode CP
    // Hier gibt es Probleme beim Übergang zur nächsten page
    // Die Adresse muss beim Schreibzyklus innerhalb von 0xFF (0x1FF, 0x2FF etc.) bleiben
    // Die automatische Adressincrementierung kann nich darüber hinaus
    unsigned char dummy;

    while(FlashIsBusy() == 0x01);

    // write enable instruction
    // WREN 0x06
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0x06);
    CS_SPI2 = 1;

    Delay1KTCYx(1);

    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    // continously program mode
    // PP 0xAD
    WriteSPI2(0xAD);

    // write 3 address bytes
    dummy = (unsigned char)(addr >> 16);
    WriteSPI2(dummy);  
    dummy = (unsigned char)(addr >> 8);
    WriteSPI2(dummy);
    dummy = (unsigned char)addr;
    WriteSPI2(dummy);
    
    WriteSPI2(*(buffer++));    
    WriteSPI2(*(buffer++));
    
    for(i=2; i < length; i+=2)
    {
        CS_SPI2 = 1;
        Delay1KTCYx(1);
        CS_SPI2 = 0;    
        WriteSPI2(0xAD);        
        WriteSPI2(*(buffer++));
        WriteSPI2(*(buffer++));
    }
     
    CS_SPI2 = 1;
    Delay1KTCYx(1);
    
     // write diable instruction
    // WREN 0x04
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0x04);
    CS_SPI2 = 1;   

    while(FlashIsBusy() == 0x01);

    CS_SPI2 = 1;

    while(FlashIsBusy() == 0x01);
}
// ----------------------------------------------------------------------------
void ReadFromFlash(unsigned long int addr, unsigned char* buffer, unsigned char length)
{
    unsigned char dummy;

    // write read command
    // READ 0x03
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;

    WriteSPI2(0x03);
    // write 3 address bytes
    dummy = (unsigned char)(addr >> 16);
    WriteSPI2(dummy);
    dummy = (unsigned char)(addr >> 8);
    WriteSPI2(dummy);
    dummy = (unsigned char)addr;
    WriteSPI2(dummy);

    // read length data bytes
    for(i=0; i < length; i++)
    {
        *(buffer + i) = ReadSPI2();
    }

    CS_SPI2 = 1;
}
// ----------------------------------------------------------------------------
void Erase4kSector(unsigned long int addr)
{
    unsigned char dummy;

    while(FlashIsBusy() == 0x01);

    // write enable instruction
    // WREN 0x06
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0x06);
    CS_SPI2 = 1;

    Delay1KTCYx(10);

    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    // erase command
    // CE 0x20
    WriteSPI2(0x20);
    // write 3 address bytes
    dummy = (unsigned char)(addr >> 16);
    WriteSPI2(dummy);
    dummy = (unsigned char)(addr >> 8);
    WriteSPI2(dummy);
    dummy = (unsigned char)addr;
    WriteSPI2(dummy);

    CS_SPI2 = 1;

    while(FlashIsBusy() == 0x01);
}
// ----------------------------------------------------------------------------
unsigned char FlashIsBusy(void)
{
    // FLASH read status register -------------------
    // RDSR 0x05

    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;

    WriteSPI2(0x05);
    // warte hier mal, bis daten zum Lesen vorhanden sind
    while(!DataRdySPI2());
    status = ReadSPI2();

    CS_SPI2 = 1;

    // Abfrage auf WIP-Bit im status-Register (write in progress)
    if((status & 0x01) == 0x01)
        return(0x01);  //flash is busy
    else
        return(0x00);
}
// ----------------------------------------------------------------------------
void ReadSensorData(void)
{
       dataBuffer.bytes[dataBuffer.idx++]=ADRESH;
       dataBuffer.bytes[dataBuffer.idx++]=ADRESL;
}

void ReadSensorDataTest(void)
{
        dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
        dataBuffer.bytes[dataBuffer.idx++]=Testbyte;

}

void ReadIMUDataTest(void)
{
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;
    dataBuffer.bytes[dataBuffer.idx++]=Testbyte;

    //Testbyte++;
    if(Testbyte>=255)
    Testbyte=0;
}


void ReadIMUData(void)
{
    // Beschleunigungssensor lesen: X-Achse
    // ACCEL_XOUT_H 0x3B + 0x80(read) = 0xBB
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBB);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_XOUT_L 0x3C + 0x80(read) = 0xBC
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBC);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Beschleunigungssensor lesen: Y-Achse
    // ACCEL_YOUT_H 0x3D + 0x80(read) = 0xBD
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBD);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_YOUT_L 0x3E + 0x80(read) = 0xBE
    CS_SPI1 = 0;
    Delay10TCYx(1);
    WriteSPI1(0xBE);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Beschleunigungssensor lesen: Z-Achse
    // ACCEL_ZOUT_H 0x3F + 0x80(read) = 0xBF
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBF);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_ZOUT_L 0x40 + 0x80(read) = 0xC0
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC0);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: X-Achse
    // GYRO_XOUT_H 0x43 + 0x80(read) = 0xC3
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC3);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_XOUT_L 0x44 + 0x80(read) = 0xC4
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC4);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: Y-Achse
    // GYRO_YOUT_H 0x45 + 0x80(read) = 0xC5
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC5);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_YOUT_L 0x46 + 0x80(read) = 0xC6
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC6);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: Z-Achse
    // GYRO_ZOUT_H 0x47 + 0x80(read) = 0xC7
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC7);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_ZOUT_L 0x48 + 0x80(read) = 0xC8
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC8);
    while(!DataRdySPI1());
    dataBuffer.bytes[dataBuffer.idx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // aktuelle Temperatur auslesen
    // TEMP_OUT_H 0x41 + 0x80 (read) = 0xC1
//    CS_SPI1 = 0;
//    Delay10TCYx(1);
//    WriteSPI1(0xC1);
//    while(!DataRdySPI1());
//    *(buffer++) = ReadSPI1();
//    CS_SPI1 = 1;
//    Delay10TCYx(1);
//    // TEMP_OUT_L 0x42 + 0x80 (read) = 0xC2
//    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
//    Delay10TCYx(1);
//    WriteSPI1(0xC2);
//    while(!DataRdySPI1());
//    *(buffer) = ReadSPI1();
//    CS_SPI1 = 1;
//    Delay10TCYx(1);
}
// ----------------------------------------------------------------------------
void TransmitDataViaBT(void)
{
    statusData.newData = 0;

    while(!TXSTA1bits.TRMT1);
    TXREG1 = STARTFLAG;                      // STARTFLAG
    while(!TXSTA1bits.TRMT1);
    TXREG1 = outPck->ID.all;                // ID (must not be a FLAG !!!)
    while(!TXSTA1bits.TRMT1);
    TXREG1 = PCK_SIZE+4; //SIZE +START +ID +SIZE +STOP
    //sendPck->ChkSum = sendPck->ID.all;

    if (dataBuffer.idx < MAX_DATA){
        for(i = 0;i<MAX_DATA;i++){

       while(!TXSTA1bits.TRMT1);
       TXREG1 = dataBuffer.bytes[i + MAX_DATA];
        }
         while(!TXSTA1bits.TRMT1);
         TXREG1 = STOPFLAG;
    }
    else{
        for(i = 0;i<MAX_DATA;i++){

       while(!TXSTA1bits.TRMT1);
       TXREG1 = dataBuffer.bytes[i];
        }
         while(!TXSTA1bits.TRMT1);
         TXREG1 = STOPFLAG;
    }
}

void SendMessage(char message)
{
    while(!TXSTA1bits.TRMT1);
    TXREG1 = STARTFLAG;                      //STARTFLAG
    while(!TXSTA1bits.TRMT1);
    TXREG1 = 0x80;                           //MESSAGE ID
    while(!TXSTA1bits.TRMT1);
    TXREG1 = message;                        //MSG
    while(!TXSTA1bits.TRMT1);
    TXREG1 = (0x80+message);                 //CHKSUM
    while(!TXSTA1bits.TRMT1);
    TXREG1 = STOPFLAG;                       //STOP
}

// ----------------------------------------------------------------------------
void TransmitDataViaUSB(unsigned char* buffer, unsigned char length)
{
    // Übertragung via USART
//    while(!TXSTA2bits.TRMT2);
//    TXREG2 = STARTBYTE;
//
//    for(i=0; i < length; i++)
//    {
//       if( (*(buffer+i) == 255)
//        || (*(buffer+i) == 254)
//        || (*(buffer+i) == 253) )
//       {
//          *(buffer+i) = *(buffer+i) - MASKBYTE;
//
//          while(!TXSTA2bits.TRMT2);
//          TXREG2 = MASKBYTE;
//       }
//       while(!TXSTA2bits.TRMT2);
//       TXREG2 = *(buffer+i);
//    }
//
//    while(!TXSTA2bits.TRMT2);
//    TXREG2 = STOPBYTE;
}
// ----------------------------------------------------------------------------
void IncrementFlashAddr(unsigned char length)
{
    addr += length;
    // wir schreiben erstmal nur bis 0xFFA = 4090...
    if(addr >= ((unsigned long int)(sector + 1) * 0x1000) )
    {
        // ...und gehen dann in den nächsten Sektor
        addr = (unsigned long int)(sector + 1) * 0x1000;
        if(addr > FlashSize) // flash size 32MB
            addr = 0x00000000;
        // diesen Sektor vor dem Schreiben erstmal löschen
        /* Erase 4K sector of flash memory
        Note: It needs to erase dirty sector before program */
        if((mode & 8) == 8)
            Erase4kSector(addr);
        sector++;
        // letzter Sektor ?
        // ...dann fang wieder von vorne an
        if(sector > 8191)
        {
           sector = 0;
           addr = 0x00000000;
        }
    }
}
// ----------------------------------------------------------------------------
void GoToSleep(void)
{
    OSCCONbits.IDLEN = 0;
    BAUDCON1bits.WUE = 1;

    GoToSleepSensor();
    GoToSleepFlash();

    //warten, bis die connection aufgelöst wurde
    while(connected == 1);
    GoToSleepWT12();

    Sleep();
}
// ----------------------------------------------------------------------------
void GoToSleepWT12(void)
{
    // setze WT12 in den sleep mode
    strcpypgm2ram(tx_buffer, "SET SLEEP");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';

    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

    Delay1KTCYx(10);
}
// ----------------------------------------------------------------------------
void GoToSleepSensor(void)
{
    // setzte Beschleunigungssensor in den speep mode
    // PWR_MGMT_1 = 0x6B + 0x00(write) = 0x6B
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x6B);
    Delay1KTCYx(10);
    // sleep bit gesetzt (Rest ist 0)
    WriteSPI1(0x40);
    CS_SPI1 = 1;
    
    Delay1KTCYx(10);
}
// ----------------------------------------------------------------------------
void GoToSleepFlash(void)
{
    // setzte Flash in den sleep mode
    // deep power down, DP 0xB9
    CS_SPI2 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI2(0xB9);
    CS_SPI2 = 1;

    //vgl. release from deep power down 0xAB
    Delay1KTCYx(10);
}
// ----------------------------------------------------------------------------

void InitADC(void)
{
    batt_voltage = 0;

    OpenADC(ADC_FOSC_32
            & ADC_RIGHT_JUST
            & ADC_12_TAD,
            ADC_CH4
            & ADC_INT_OFF
            & ADC_REF_VDD_VSS, 14);

    // wegen Fehler in der Bibliothek
    ADCON1 = 0b00001111;

    ConvertADC();
    while( BusyADC());
    batt_voltage = ReadADC();

    POWER_OK = 0;
    // Grün ist aus
    // Rot ist aus

    // Akku voll
    if(batt_voltage > 900)
    {
        // 3mal grün blinken
        for(i=0; i < 6; i++)
        {
            LED_GRUEN = !LED_GRUEN;
            Delay10KTCYx(100);
        }
        // Grün ist aus
        LED_GRUEN = 1;
        POWER_OK = 1;
    }
    else
    {
        // Akku noch o.k.
        if( (batt_voltage > 700) && (batt_voltage <= 900))
        {
            // 2mal grün blinken
            for(i=0; i < 4; i++)
            {
                LED_GRUEN = !LED_GRUEN;
                Delay10KTCYx(100);
            }
            // Grün ist aus
            // 1mal rot blinken
            for(i=0; i < 2; i++)
            {
                LED_ROT = !LED_ROT;
                Delay10KTCYx(100);
            }
            // Grün ist aus
            // Rot ist aus
            LED_GRUEN = 1;
            POWER_OK = 1;
        }
        else
        {
            // Akku schwächelt
            if( (batt_voltage > 500) && (batt_voltage <= 700))
            {
                // 1mal grün blinken
                for(i=0; i < 2; i++)
                {
                    LED_GRUEN = !LED_GRUEN;
                    Delay10KTCYx(100);
                }
                // Grün ist aus
                // 2mal rot blinken
                for(i=0; i < 4; i++)
                {
                    LED_ROT = !LED_ROT;
                    Delay10KTCYx(100);
                }
                // Grün ist aus
                // Rot ist aus
                LED_GRUEN = 1;
                POWER_OK = 1;
            }
            else
            {
                // Akku laden
                // 3mal rot blinken
                for(i=0; i < 6; i++)
                {
                    LED_ROT = !LED_ROT;
                    Delay10KTCYx(100);
                }
                // Rot ist aus
                LED_ROT = 1;
                POWER_OK = 0;
            }
        }
    }
    CloseADC();
}
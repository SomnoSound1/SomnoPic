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
unsigned int spbrg;


void __init()
{
    //OSCCON = 0b01100000;  // 8 MHz internal clock
    OSCCON = 0b01110000;    // 16 MHz internal clock, wegen USART mit 115200Baud

    ANSELA = 0b00100001;    // enable analog inputs
    TRISA = 0b00100001;     // OSC, OSC, AD_BATT, LED_ROT, PAD2 I/O, Taster, AD_1, AD_0

    ANSELB = 0b00000000;    // disable analog inputs on PORTB
    TRISB = 0b10100101;     // PGD/RX, PGC/TX, WT12_CON, CS2, SDO2, SDI2, SCK2, INT_SENSOR
    WPUBbits.WPUB7 = 1;     // pull up an RB7 enabled (RX)
    WPUBbits.WPUB6 = 1;     // pull up an RB6 enabled (TX)

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
//    InitTimer1();
//    InitADC0();

    //Mit CCP
    InitCCP();

    InitWT12();
    //InitBluetooth();
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
    timer0.lt = 65536 - (unsigned int)(FOSC_/4/PRETEILER/SAMPRATE);

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
    timer1.lt = 65536 - (unsigned int)(FOSC_/4/8/SAMPRATE); // - (unsigned int)(FOSC/4/8/SAMPRATE);

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
    OpenADC(    ADC_FOSC_64 & 	// AD Conversion Clock Select Bit
            ADC_RIGHT_JUST  & 	// Result in Least Significant Bits
            ADC_12_TAD      ,	// Acquisition Time 12 Tad
            ADC_CH0         &  	// Channal 0 für AD Wandlung
            ADC_INT_ON      &	// Interrupt enabled
            ADC_REF_VDD_VSS ,   // Referenz = Betriebsspannung 0 bis 3,3V;
            ADC_TRIG_CCP5       // Trigger
            );              //Port configuration (all digital)

      OpenTimer1(  TIMER_INT_OFF &
            T1_16BIT_RW &
            T1_SOURCE_FOSC_4 &
            T1_PS_1_8 &
            T1_OSC1EN_OFF &
            T1_SYNC_EXT_OFF,
            TIMER_GATE_OFF);

//Enable Interrupt
//T1CONbits.TMR1ON = 1;

CCPTMRS1bits.C5TSEL = 0; // timer <-> ccp module (CCP5 / TMR1)
CCPR5L = 100; // Fosc/4 / prescaler / samprate
CCPR5H = 100/256;
CCP5CONbits.CCP5M = 0b1011; // Compare Mode with Special Event Trigger
}

void InitBluetooth(void)
{
        spbrg = 16;
        //spbrg = 34;
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
}


// ----------------------------------------------------------------------------
void InitWT12(void)
{
    //spbrg = 16;
    spbrg = 34;
    rx_count = 0;
    //connected = 0;

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
    //setze neuen Namen
    strcpypgm2ram(tx_buffer, "SET BT NAME Somno3");
    buff_len = strlen(tx_buffer);
    tx_buffer[buff_len++] = '\r';
    tx_buffer[buff_len++] = '\n';


    // send command via USART
    for(i = 0; i < buff_len; i++)
    {
       TXREG1 = tx_buffer[i];
       while(!TXSTA1bits.TRMT1);
    }

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


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

#pragma udata // let linker place variables (banked RAM)

#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

char Testbyte = 3;
unsigned char Manuf, Dev1, Dev2, status, IMUidx;
int batt_voltage, triggerTimer;
unsigned short package_number = 0;
char POWER_OK;

void GetData()
{    
    ReadSensorData();
    //ReadSensorDataTest();
    if(dataBuffer.idx == MAX_AUDIO || dataBuffer.idx >=(MAX_AUDIO+MAX_DATA))
    {        
        //ReadIMUDataTest();
        //Platz für IMU Daten lassen
        dataBuffer.idx += 14;
        statusData.newValues = 1;       
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
}

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

    if(Testbyte>=255)
    Testbyte=0;
}

void AddPackageNumber(void)
{
    dataBuffer.bytes[IMUidx++] = LOWBYTE(package_number);
    dataBuffer.bytes[IMUidx++] = HIGHBYTE(package_number);
    package_number++;
}

void AddSyncTrigger(void)
{
    triggerTimer++;
    if(triggerTimer >= 250)
    {
        triggerTimer=0;
        TESTPIN = !TESTPIN;
        dataBuffer.bytes[IMUidx++] = TESTPIN;
    }
}

void ReadIMUData(void)
{

    if (dataBuffer.idx < MAX_DATA)
        IMUidx = 94;
    else
        IMUidx = 40;

    //TESTPIN = !TESTPIN;
    // Beschleunigungssensor lesen: X-Achse
    // ACCEL_XOUT_H 0x3B + 0x80(read) = 0xBB
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBB);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_XOUT_L 0x3C + 0x80(read) = 0xBC
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBC);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Beschleunigungssensor lesen: Y-Achse
    // ACCEL_YOUT_H 0x3D + 0x80(read) = 0xBD
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBD);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_YOUT_L 0x3E + 0x80(read) = 0xBE
    CS_SPI1 = 0;
    Delay10TCYx(1);
    WriteSPI1(0xBE);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Beschleunigungssensor lesen: Z-Achse
    // ACCEL_ZOUT_H 0x3F + 0x80(read) = 0xBF
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xBF);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // ACCEL_ZOUT_L 0x40 + 0x80(read) = 0xC0
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC0);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: X-Achse
    // GYRO_XOUT_H 0x43 + 0x80(read) = 0xC3
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC3);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_XOUT_L 0x44 + 0x80(read) = 0xC4
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC4);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: Y-Achse
    // GYRO_YOUT_H 0x45 + 0x80(read) = 0xC5
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC5);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_YOUT_L 0x46 + 0x80(read) = 0xC6
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC6);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);

    // Gyrosensor lesen: Z-Achse
    // GYRO_ZOUT_H 0x47 + 0x80(read) = 0xC7
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC7);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
    CS_SPI1 = 1;
    Delay10TCYx(1);
    // GYRO_ZOUT_L 0x48 + 0x80(read) = 0xC8
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    Delay10TCYx(1);
    WriteSPI1(0xC8);
    while(!DataRdySPI1());
    dataBuffer.bytes[IMUidx++] = ReadSPI1();
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
//    TESTPIN = !TESTPIN;
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

//void TransmitDataViaBT(void)
//{
//    unsigned char j;
//    struct serialPck sendPck;
//
//    if (dataBuffer.idx < MAX_DATA){
//    for(i = 0;i<MAX_DATA;i++)
//    sendPck.Bytes[i] = dataBuffer.bytes[i + MAX_DATA];}
//    else{
//    for(i = 0;i<MAX_DATA;i++)
//    sendPck.Bytes[i] = dataBuffer.bytes[i];}
//
//
//    sendPck.ID.SIZE = 0;
//    sendPck.ID.MESSAGE = 0;
//    statusData.newData = 0;
//
//    SendPck(sendPck);
//}

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
void GoToSleep(void)
{
    OSCCONbits.IDLEN = 0;
    BAUDCON1bits.WUE = 1;

    GoToSleepSensor();   

    //warten, bis die connection aufgelöst wurde
    while(WT12_CON);
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
    // setzte Beschleunigungssensor in den sleep mode
    // PWR_MGMT_1 = 0x6B + 0x00(write) = 0x6B
    CS_SPI1 = 0; //CS_SPI1 = 0 -> chip enabled;
    WriteSPI1(0x6B);
    Delay1KTCYx(10);
    // sleep bit gesetzt (Rest ist 0)
    WriteSPI1(0x40);
    CS_SPI1 = 1;

    Delay1KTCYx(10);
}


void CloseAll(void)
{
    Close1USART();
    Close2USART();
    CloseSPI1();
    CloseSPI2();
    CloseADC();
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

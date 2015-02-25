/* 
 * File:   functions.h
 * Author: Lizenznehmer
 *
 * Created on 24. Februar 2015, 10:29
 */

#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

void InitInterrupts(void);
void InitTimer1(void);
void InitTimer0(void);
void InitWT12(void);
void InitMPU6000(void);
void InitBluetooth(void);
void GoToSleep(void);
void GoToSleepWT12(void);
void GoToSleepSensor(void);
void InitADC(void);
void InitADC0(void);
void InitCCP(void);

void GetData(void);
void CloseAll(void);
void SendMessage(char message);
void ReadIMUDataTest(void);
void ReadIMUData(void);
void ReadSensorData(void);
void ReadSensorDataTest(void);
void TransmitDataViaBT(void);
void TransmitDataViaUSB(unsigned char* buffer, unsigned char length);

#endif	/* FUNCTIONS_H */


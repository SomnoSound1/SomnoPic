/* 
 * File:   global_def.h
 * Author: Lizenznehmer
 *
 * Created on 24. Februar 2015, 13:49
 */
#include "uC_main.h"
#include <timers.h>

#ifndef GLOBAL_DEF_H
#define	GLOBAL_DEF_H

#define    FOSC_       16000000  // internal clock
#define    BAUDRATE    230400
#define    SAMPRATE    5000
//#define    SAMPRATE    1385
//#define    SAMPRATE    750
#define    PRESCALE    T0_PS_1_32
#define    PRETEILER   32


// G L O B A L E   P R O T O T Y P E N  ////////////////////////////////////////
extern char mode;
extern union u_status statusData;     // (siehe oben)
extern unsigned char TM0_LowByte, TM1_LowByte;
extern unsigned char TM0_HightByte, TM1_HightByte;
extern union Timers timer0, timer1;
extern unsigned int spbrg;
extern struct s_dataBuffer  dataBuffer;     // daten fuer PC
extern unsigned char buff_len;
extern unsigned char tx_buffer[30];
extern unsigned char rx_count;
extern unsigned char rx_buffer[50];
extern unsigned char i, tick_counts;
extern unsigned char dev_id;
extern char FS_SEL, AFS_SEL, MOT_EN, MOT_THR, MOT_DUR, MOT_INT;
#endif	/* GLOBAL_DEF_H */


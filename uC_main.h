#ifndef UC_MAIN_H
#define	UC_MAIN_H

#define CS_SPI1     LATCbits.LATC2
#define CS_SPI2     LATBbits.LATB4

#define WT12_CON    PORTBbits.RB5
#define TESTPIN     PORTAbits.RA1

#define LED_GRUEN   PORTCbits.RC1
#define LED_BLAU    PORTCbits.RC0
#define LED_ROT     PORTAbits.RA4

#define MAX_DATA        52
#define MAX_AUDIO       40
#define MAX_RX_BUFFER   10
#define STANDBY_TIMEOUT 2000


union u_status {
  char all;
  struct {
    unsigned newData        : 1;
    unsigned dataOVF        : 1;
    unsigned newRX          : 1;
    unsigned rxOVF          : 1;
    unsigned newValues      : 1;
    unsigned startADC       : 1;
    unsigned bit6           : 1;
    unsigned bit7           : 1;
  };
};

struct s_dataBuffer{
    unsigned char idx;
    unsigned char bytes[MAX_DATA*2];
};

struct s_rxBuffer{
    unsigned char idx;
    unsigned char bytes[MAX_RX_BUFFER];
};


//#define    CID_INIT        = 0x00,
//#define    CID_STOP        = 0x01,     // stop sending data
////#define    CID_CLOSE,
//#define    CID_START       = 0x03,     // send data (continuous)
//#define    CID_DEVICE      = 0x10,     // send device identification string
////#define    CID_VERSION,                // send version info-string
////#define    CID_DATE,                   // send date info-string
////#define    CID_MANU,                   // send developper info-string
////#define    CID_MISC,                   // send miscellanous info-string
//#define    CID_LED2_OFF    = 0x20,     // LED_2 einschalten
//#define    CID_LED2_ON     = 0x21,     // LED_2 einschalten
//#define    CID_LED2_TOG    = 0x23,
//#define    CID_TIME        = 0x30,     // set sampling time (in ms)

#endif	/* UC_MAIN_H */


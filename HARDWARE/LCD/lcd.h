#ifndef _lcd_h_
#define _lcd_h_
#include<stm32f4xx.h>
typedef unsigned char uchar;
typedef unsigned int uint;

void gpiolcd_init(void);
void delay_xms(u16 time);
void SendByteLCD(uchar WLCDData);
void SPIWR(uchar Wdata,uchar WRS);
void lcd_wcmd(uchar CMD);
void lcd_wdat(uchar Data);
void LCDInit(void);
void ShowQQChar(char addr,char *english,char count);

#endif

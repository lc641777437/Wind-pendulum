#include "lcd.h"
#include <math.h>
#include<stm32f4xx_gpio.h>
#include<stm32f4xx_rcc.h>
/*
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,//第一行汉字位置
0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,//第二行汉字位置
0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,//第三行汉字位置
0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,//第四行汉字位置
*/
void gpiolcd_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOC时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//GPIOC
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
}

/*void delay_xms(u16 time)
{    
   u16 i;  
   while(time--)
   {
      i=12000;  
      while(i--);                     
   }
}*/
void SendByteLCD(uchar WLCDData)
{
    uchar i;
    gpiolcd_init();
	for(i=0;i<8;i++)
	{
		if((WLCDData<<i)&0x80)
			 GPIO_SetBits(GPIOB,GPIO_Pin_15);//rw
		else 
			GPIO_ResetBits (GPIOB, GPIO_Pin_15); //rw
		GPIO_ResetBits (GPIOB, GPIO_Pin_13);//en
                GPIO_SetBits(GPIOB,GPIO_Pin_13);//en
	}
}

void SPIWR(uchar Wdata,uchar WRS)
{
	SendByteLCD(0xf8+(WRS<<1));
        delay_xms(10);
	SendByteLCD(Wdata&0xf0);
        delay_xms(5);
	SendByteLCD((Wdata<<4)&0xf0);
        delay_xms(5);
}

void lcd_wcmd(uchar CMD)
{
        GPIO_ResetBits (GPIOB, GPIO_Pin_14); //rs
	GPIO_SetBits(GPIOB,GPIO_Pin_14);//rs
	SPIWR(CMD,0);
	delay_xms(1);
}

void lcd_wdat(uchar Data)
{
        GPIO_ResetBits (GPIOB, GPIO_Pin_14); //rs
	GPIO_SetBits(GPIOB,GPIO_Pin_14);//rs
	SPIWR(Data,1);
	delay_xms(1);
}

void LCDInit(void)
{
  lcd_wcmd(0x30); //基本指令集
	lcd_wcmd(0x06); //启始点设定：光标右移
	lcd_wcmd(0x01); //清除显示DDRAM
	lcd_wcmd(0x0C); //显示状态开关：整体显示开，光标显示关，光标显示反白关
	lcd_wcmd(0x02); //地址归零
}

void ShowQQChar(char addr,char *english,char count)
{
	uchar i;
	lcd_wcmd(addr); //设定DDRAM地址
	for(i=0;i<count;)
	{
		lcd_wdat(english[i*2]);
		lcd_wdat(english[i*2+1]);
		i++;
	}
}

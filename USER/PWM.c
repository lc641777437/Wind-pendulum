#include<stm32f4xx.h>
#include"stm32f4xx_tim.h"
#include<stm32f4xx_gpio.h>
#include<stm32f4xx_rcc.h>
#include<stm32f4xx_exti.h>
#include<misc.h>
#include<pwm.h>
#include<stdio.h>

#include<stm32f4xx.h>
#include "stm32f4xx_tim.h"
#include<stm32f4xx_gpio.h>
#include<stm32f4xx_rcc.h>
#include<stm32f4xx_exti.h>
#include<misc.h>
#include <stdio.h>



void TIM3_PWM_Init(unsigned int arr,unsigned int psc)
{              
      GPIO_InitTypeDef GPIO_InitStructure;
      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
      TIM_OCInitTypeDef  TIM_OCInitStructure;     
      
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //TIM3时钟使能
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能gpioC时钟
       
       GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);//GA6复用为TIM3
       GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);//GA6复用为TIM3
       GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);//GA6复用为TIM3
       GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);//GA6复用为TIM3
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
       GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PA6
       
       TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
       TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
       TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
       TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
       TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
       
       TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM调制模式1
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性低
       TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_Cmd(TIM3, ENABLE);  //使能TIM3
       TIM_SetCompare1(TIM3,0);
       TIM_SetCompare2(TIM3,0);
       TIM_SetCompare3(TIM3,0);
       TIM_SetCompare4(TIM3,0);
}

void TIM4_PWM_Init(unsigned int arr,unsigned int psc)              ////   	用法：TIM3_PWM_Init(psc_max-1,0);     TIM_SetCompare1(TIM8,psc);TIM_SetCompare2(TIM8,0);
{                                              
       GPIO_InitTypeDef GPIO_InitStructure;
       TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
       TIM_OCInitTypeDef  TIM_OCInitStructure;
       
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); //TIM3时钟使能
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能gpioa时钟
       
       GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);//PA7复用为TIM3
       GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);//PA6复用为TIM3
       
      
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
       GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PA6、PA7
       
       TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
       TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
       TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
       TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
       TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器3
       
       TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM调制模式1
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性低
       TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //初始化外设TIM3OC2
       TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_Cmd(TIM4, ENABLE);  //使能TIM3
       
}
void TIM5_PWM_Init(unsigned int arr,unsigned int psc)              ////   	用法：TIM3_PWM_Init(psc_max-1,0);     TIM_SetCompare1(TIM8,psc);TIM_SetCompare2(TIM8,0);
{                                              
       GPIO_InitTypeDef GPIO_InitStructure;
       TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
       TIM_OCInitTypeDef  TIM_OCInitStructure;
       
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE); //TIM3时钟使能
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能gpioa时钟
       
       
       GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);//PA6复用为TIM3    
      // GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);//PA7复用为TIM3
      // GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);//PA6复用为TIM3
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//GPIO_Pin_0|GPIO_Pin_1;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
       GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PA6、PA7
       
       TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
       TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
       TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
       TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
       TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器3
       
       TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM调制模式1
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性低
       TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //初始化外设TIM3OC1
       TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //初始化外设TIM3OC2
       TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能预装载寄存器
       TIM_Cmd(TIM5, ENABLE);  //使能TIM3
       
}


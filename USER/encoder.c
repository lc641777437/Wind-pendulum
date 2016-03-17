#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "catch.h"
#include "pid.h"
#include "lcd.h"
#include "math.h"
#include "remote.h"
#include "string.h"
#include "inv_mpu.h"
#include "stm32f4xx_exti.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "mpu6050.h"
#include "pwm.h"
#include "stdlib.h"
#include "TIM_Encoder.h"
 extern GPIO_InitTypeDef GPIO_InitStructure;  
 extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 extern TIM_ICInitTypeDef TIM_ICInitStructure; 
 extern NVIC_InitTypeDef NVIC_InitStructure;
 extern EXTI_InitTypeDef   EXTI_InitStructure;
void ENC1_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
    
    /*NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);*/
    
    /* Timer configuration in Encoder mode */ 
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
    TIM_TimeBaseStructure.TIM_Period = 40000-1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xe;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
    // Clear all pending interrupts
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM2,0);
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);
}

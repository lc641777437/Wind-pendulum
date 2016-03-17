#ifndef _pwm_h_
#define _pwm_h_
#include<stm32f4xx.h>
void TIM5_PWM_Init(unsigned int arr,unsigned int psc);
void TIM3_PWM_Init(unsigned int arr,unsigned int psc);
void TIM4_PWM_Init(unsigned int arr,unsigned int psc);
#endif

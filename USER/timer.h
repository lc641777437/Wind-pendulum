
#include "stm32f4xx.h"
#include "delay.h"
#include "sys.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "math.h"
//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms
//TIM4_Int_Init(10-1,8400-1);

extern int32_t ms;
extern int64_t us;
extern float pitch_6050,roll_6050,yaw_6050;  
extern float pitch_max,roll_max,yaw_max; 
extern float pitch_v,roll_v; 
extern float theata;
extern float theata_max;
extern int wdir;
extern int wdir_pre;
extern int highpoint;
extern int pwm_change;
extern int pwm_clr;
extern int16_t pwm_a,pwm_b;
extern int en;
extern int period;
void TIM5_Int_Init(u16 arr,u16 psc);

void TIM2_Cap_Init(u32 arr,u16 psc);
void TIM10_Int_Init(u16 arr,u16 psc);





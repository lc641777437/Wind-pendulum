#ifndef _TIM_Encoder_h_
#define _TIM_Encoder_h_
extern int16_t TIM3_Encoder_Cnt;
extern int TIM2_Encoder_Cnt;
extern int a;
extern float Motor_Angle,Motor_Angle_Pre,Motor_Angle_Speed;
extern int16_t Tim3_Pre_Cnt;
extern int Tim2_Pre_Cnt;
extern float Sensor_Angle,Senor_Angle_Pre,Sensor_Angle_Speed;
extern int16_t Sensor_Buf[10];
void ENC_Init(void);
void time7_init(void);
int TIM7_IRQHandler(void);
void ENC1_Init(void);
#endif

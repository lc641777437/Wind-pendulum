
#include "timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
extern  short gyrox,gyroy,gyroz,gyox_1,gyoy_1,gyoz_1;
extern float pitch_v,roll_v,yaw_v,err_v,roll_v_now;
extern short aacx,aacy,aacz;
extern float pitch,roll,yaw;  //yaw ����� roll �����  pitch  ������

int32_t ms=0;
int64_t us=0;
int flag_a,flag_b;



void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM5,ENABLE); //ʹ�ܶ�ʱ��3
	
	TIM5->CNT=0;
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
void TIM4_Int_Init(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); //??TIM3??

    TIM_TimeBaseInitStructure.TIM_Period = 999; //?????
    TIM_TimeBaseInitStructure.TIM_Prescaler=839;  //????
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //?????
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);// ???????TIM3
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //?????3????

    NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //??3??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //????1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //????3
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);// ????NVIC

    TIM_Cmd(TIM4,ENABLE); //?????3
}

void TIM4_IRQHandler(void)
{
       if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
       {
				 	if(flag_b<30000)
	     {
				 flag_b++;
	      }
	      else flag_b=0;
			 }
       TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //??????
}


void TIM5_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //����ж�
	{ 
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz); 
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0);
	  get_gyo(gyrox,gyroy,&pitch_v,&roll_v);
			
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //����жϱ�־λ
}


//void TIM10_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  ///ʹ��TIM3ʱ��
//	
//  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
//	
//	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
//	TIM_Cmd(TIM10,ENABLE); //ʹ�ܶ�ʱ��3
//	
//	TIM10->CNT=0;
//	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //��ʱ��3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//}


//void TIM1_UP_TIM10_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //����ж�
//	{
//		us++;
// 
//	}
//	TIM_ClearITPendingBit(TIM10,TIM_IT_Update);  //����жϱ�־λ
//}


//TIM_ICInitTypeDef  TIM2_ICInitStructure;
////��ʱ��2ͨ��1/2/3���벶������
////arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
////psc��ʱ��Ԥ��Ƶ��
//void TIM2_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); //PA0����λ��ʱ��5
//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
//  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
//	

//	//��ʼ��TIM2���벶�����
//	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2 | TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
//  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
//  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
//  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
//  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
//		
//	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
//	
//  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	
//}

//void TIM2_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
//	{
////		us++;
////		us=0;
////		led_s=!led_s;
////		LED1(led_s);
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
//	}
//	
//	if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)
//	{
//		flag =((flag+1)%3);
//		time1=us;
//		TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
//		
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); 
//	}
//	
//	if(TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)
//	{
//		flag =((flag+1)%3);
//		time2=us;
//		TIM_ITConfig(TIM2,TIM_IT_CC2,DISABLE);
//		
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); 
//	}
//	
//	if(TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)
//	{
//		flag =((flag+1)%3);
//		time3=us;
//		TIM_ITConfig(TIM2,TIM_IT_CC3,DISABLE);
//		
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); 
//	}
//		
//}




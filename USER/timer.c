
#include "timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
extern  short gyrox,gyroy,gyroz,gyox_1,gyoy_1,gyoz_1;
extern float pitch_v,roll_v,yaw_v,err_v,roll_v_now;
extern short aacx,aacy,aacz;
extern float pitch,roll,yaw;  //yaw 航向角 roll 横滚角  pitch  俯仰角

int32_t ms=0;
int64_t us=0;
int flag_a,flag_b;



void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	TIM5->CNT=0;
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
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
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{ 
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz); 
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0);
	  get_gyo(gyrox,gyroy,&pitch_v,&roll_v);
			
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
}


//void TIM10_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  ///使能TIM3时钟
//	
//  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM10,ENABLE); //使能定时器3
//	
//	TIM10->CNT=0;
//	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//}


//void TIM1_UP_TIM10_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //溢出中断
//	{
//		us++;
// 
//	}
//	TIM_ClearITPendingBit(TIM10,TIM_IT_Update);  //清除中断标志位
//}


//TIM_ICInitTypeDef  TIM2_ICInitStructure;
////定时器2通道1/2/3输入捕获配置
////arr：自动重装值(TIM2,TIM5是32位的!!)
////psc：时钟预分频数
//void TIM2_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); //PA0复用位定时器5
//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
//  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
//	

//	//初始化TIM2输入捕获参数
//	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2 | TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
//  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
//  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
//  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
//  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
//  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
//		
//	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
//	
//  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
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




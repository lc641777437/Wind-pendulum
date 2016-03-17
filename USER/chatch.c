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
#include "timer.h"
#include "TIM_Encoder.h"
 GPIO_InitTypeDef GPIO_InitStructure;  
 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 TIM_ICInitTypeDef TIM_ICInitStructure; 
 NVIC_InitTypeDef NVIC_InitStructure;
 EXTI_InitTypeDef   EXTI_InitStructure;
 extern float data;
extern  short gyrox,gyroy,gyroz,gyox_1,gyoy_1,gyoz_1;
extern float pitch_v,roll_v,yaw_v,err_v,roll_v_now;
extern short aacx,aacy,aacz;
extern int x;
extern float x_v,y_v;
extern int flag,flag_v;
extern double alfa;
extern arm_pid_instance_f32 S;	
extern float PWM_1,PWM_2;
extern char sp[10],yj[10],hj[10];
extern float pitch,roll,yaw;  //yaw 航向角 roll 横滚角  pitch  俯仰角
extern int str_i;
extern float roll_max,aim;
extern int i;
extern int theta[2][10];
extern int circle;
void EXTIX_Init(void)

{
     GPIO_InitTypeDef GPIO_InitStructure;  
	   NVIC_InitTypeDef   NVIC_InitStructure;
     EXTI_InitTypeDef   EXTI_InitStructure;
	
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟 
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//下拉
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_2|GPIO_Pin_3;
     GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);       //使能SYSCFG时钟  
     SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE2连接线2     B点
	   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE2连接线2     B点
     SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2连接线2     B点
     SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);//PE3连接线3     A点
     SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);//PE4连接线4     C点

/* 配置EXTI_Line2,3,4 */

  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //上升沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置   

  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置NVIC

  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断3.0000
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置NVIC 

  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置 NVIC 
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置 NVIC 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置 NVIC 
	
}
/*                  控幅摆                           */
void EXTI0_IRQHandler(void)

{
	
  if( EXTI_GetITStatus(EXTI_Line0)!=RESET)
  {
				/*key_scan();
		ShowQQChar(0x80,str,strlen(str)%2?(strlen(str)/2+1):strlen(str)/2);
		delay_ms(800);*/
		/****************************************************************************/
/*                                 控幅摆                                   */
/****************************************************************************/
		while(1)
		{
      key_scan();
      x=data;
			for(i=0;i<10000;i++);
			if(data!=0)
				break;
		}
	TIM5_Int_Init(10000-1,42-1);
  aim=atan(x/88.3)*57.3*1.03;
	S.Kp=400;
  S.Ki=30;
  S.Kd=2;
while(1)
	{		
	  if(fabs((float)roll)>roll_max)
	  roll_max=abs((float)roll);
		if(roll_max<aim)
		{	
			if(pitch_v>0)
		{
			TIM_SetCompare4(TIM3,2000);
			TIM_SetCompare3(TIM3,0);
		}
		else
		{
			TIM_SetCompare3(TIM3,2000);
			TIM_SetCompare4(TIM3,0);
			
		}
				
			if(roll_v>0)
		  {
			  TIM_SetCompare2(TIM3,2000);
			  TIM_SetCompare1(TIM3,0);
		  }
		  else
		  {
			  TIM_SetCompare1(TIM3,2000);
			  TIM_SetCompare2(TIM3,0);	
		  }
		}
		else 
		{
			while(1)
			{
				if(fabs(roll_v)<0.8)
				{
					roll_max=fabs(roll);
					err_v=aim-fabs(roll);
					PWM_1=arm_pid_speed(&S,err_v);
					//PWM_1=err_v*200;
					delay_ms(50);
				}
							if(pitch_v>0)
		{
			TIM_SetCompare4(TIM3,2000);
			TIM_SetCompare3(TIM3,0);
		}
		else
		{
			TIM_SetCompare3(TIM3,2000);
			TIM_SetCompare4(TIM3,0);
			
		}
					if(roll_v>0)
		      {
						if(PWM_1>0)
						{
			        TIM_SetCompare2(TIM3,1000+PWM_1);
			        TIM_SetCompare1(TIM3,1000);
						}
						else
						{
						  TIM_SetCompare1(TIM3,1000-PWM_1);
			        TIM_SetCompare2(TIM3,1000);
						}
		      }
					else
					{
						if(PWM_1>0)
						{
						   TIM_SetCompare1(TIM3,1000+PWM_1);
			         TIM_SetCompare2(TIM3,1000);
						}
						else
						{
						   TIM_SetCompare2(TIM3,1000-PWM_1);
			         TIM_SetCompare1(TIM3,1000);
						}
						
					}
					
				}	
			}	
		}
  
	}
  EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位     
}
/*                  控角摆                           */		
  /**************************************************************************/
	/*                            任意角度摆                                  */
	/**************************************************************************/
void EXTI1_IRQHandler(void)
{
	
  if( EXTI_GetITStatus(EXTI_Line1)!=RESET)
  {
		TIM5_Int_Init(10000-1,42-1);
	   while(1)
	   {	
		   for(i=0;i<=9;i++)
		   {
			   if(theta[1][i]==alfa)
			   {
				   alfa=theta[2][i];	      
				   break;
			   }
		   }
		   //x_v = cos(alfa/57.3)*31108/10.36;
       //y_v = sin(alfa/57.3)*31108/10.36;
		   x_v=(cos(alfa/57.3)*42320+413)/14.22f;
		   y_v=(sin(alfa/57.3)*42320+413)/14.22f;
	     //x_v=(cos(alfa/57.3)*(42315.73))/10.5789f;
       //y_v=(sin(alfa/57.3)*(42315.73)+202.825*cos(alfa/57.3))/10.5789f;
		   while((fabs(roll)<15.0f )&&( fabs(pitch)<15.0f))
		  {
			   if(roll_v>0||pitch_v<0)
         {
			     TIM_SetCompare2(TIM3,x_v);
			     TIM_SetCompare1(TIM3,0);
			     TIM_SetCompare4(TIM3,y_v);
			     TIM_SetCompare3(TIM3,0);
			   }
			   else
			   {
			      TIM_SetCompare1(TIM3,x_v);
			      TIM_SetCompare2(TIM3,0);
			      TIM_SetCompare3(TIM3,y_v);
			      TIM_SetCompare4(TIM3,0);		
			   }	
		  }
			TIM_SetCompare1(TIM3,0);
      TIM_SetCompare2(TIM3,0);
      TIM_SetCompare3(TIM3,0);
      TIM_SetCompare4(TIM3,0);
	  }
	}
  EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE0上的中断标志位      
}

/*                  制动停                           */		
  /****************************************************************/
	/*                        制动************************************/
	/****************************************************************/
void EXTI2_IRQHandler(void)

{
							MPU_Init();
	          while(mpu_dmp_init()==1);
	TIM5_Int_Init(10000-1,42-1);
  if( EXTI_GetITStatus(EXTI_Line2)!=RESET)
  {   

	while(1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_14);
	  if(pitch_v>0)
		{
			TIM_SetCompare4(TIM3,5000);
			TIM_SetCompare3(TIM3,0);
			
		}
		else
		{
			TIM_SetCompare3(TIM3,5000);
			TIM_SetCompare4(TIM3,0);
		}
	  if(roll_v>0)
		{
			TIM_SetCompare1(TIM3,5000);
			TIM_SetCompare2(TIM3,0);
			
		}
		else
		{
			TIM_SetCompare2(TIM3,5000);
			TIM_SetCompare1(TIM3,00);
		}
	
	}
		
  }
  EXTI_ClearITPendingBit(EXTI_Line2); //清除LINE0上的中断标志位
}      


//外部中断2服务程序
/*                  画圆摆                           */
void EXTI3_IRQHandler(void)

{  
	TIM5_Int_Init(10000-1,42-1);
  if( EXTI_GetITStatus(EXTI_Line3)!=RESET)
  {   
		while(1)
{
	while(fabs(roll)<10)
	{
	if(roll_v>0)
		{
			TIM_SetCompare2(TIM3,3000);
			TIM_SetCompare1(TIM3,0);
		}
		else
		{
			TIM_SetCompare1(TIM3,3000);
			TIM_SetCompare2(TIM3,00);
		}
	}
	while(1)
	{
		if(roll_v>0)
		{
			TIM_SetCompare2(TIM3,circle);
			TIM_SetCompare1(TIM3,0);
		}
		else
		{
			TIM_SetCompare1(TIM3,circle);
			TIM_SetCompare2(TIM3,0);
		}
		if(fabs(roll_v)<1)
		{
			while(fabs(pitch)<11)
			{
				if(pitch_v>0)
		    {
			     TIM_SetCompare3(TIM3,3000);
			     TIM_SetCompare4(TIM3,0);			
	      }
		    else
		    {
			    TIM_SetCompare4(TIM3,3000);
			    TIM_SetCompare3(TIM3,0);	
	      } 				
			}
			while(1)
			{ 
				if(roll_v>0)
		    {
			    TIM_SetCompare2(TIM3,circle);
			    TIM_SetCompare1(TIM3,0);
		    }
		    else
		    {
			     TIM_SetCompare1(TIM3,circle);
			     TIM_SetCompare2(TIM3,0);
		    }
	      if(pitch_v>0)
		    {
			     TIM_SetCompare3(TIM3,circle);
			     TIM_SetCompare4(TIM3,0);			
		    }
		    else
		    {
			     TIM_SetCompare4(TIM3,circle);
			     TIM_SetCompare3(TIM3,0);	
	      } 
		  }	
		}
	}
}
	}
  EXTI_ClearITPendingBit(EXTI_Line3);//清除LINE2上的中断标志位

}

//外部中断3服务程序
/*                  抗扰摆                           */
void EXTI4_IRQHandler(void)

{  
	TIM5_Int_Init(10000-1,42-1);
  if( EXTI_GetITStatus(EXTI_Line4)!=RESET)
  {
    
  }
  EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE4上的中断标志位 
}


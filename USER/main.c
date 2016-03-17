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
#include "timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
void TIM4_Int_Init(void);


void System_Init(void);
extern char str[10];
int theta[2][10]={0,10,20,30,40,50,60,70,80,90,0,20,30,35,45,50,55,63,75,90};
short gyrox,gyroy,gyroz,gyox_1,gyoy_1,gyoz_1;
float pitch_v,roll_v,yaw_v,err_v,roll_v_now=0;
short aacx,aacy,aacz;
int x=25,i;
float x_v,y_v,d,b,c;
int flag=0,flag_v=0;
double alfa=30;
arm_pid_instance_f32 S;	
float PWM_1,PWM_2,gama;
char sp[10],yj[10],hj[10];
float pitch,roll,yaw;  //yaw º½Ïò½Ç roll ºá¹ö½Ç  pitch  ¸©Ñö½Ç
extern int str_i;
float d_v,b_v;
extern float data;
float roll_max,aim,pitch_max,err_v_2;
	float theta_1;
	int circle=1000;
	extern int flag_b;
/*aim=atan(x/88.3)*57.3;*/
/*                             ÏµÍ³¸´Î»º¯Êý                                   */
/*	
  __set_FAULTMASK(1);
	NVIC_SystemReset();
*/

   int main(void)
{ 
	/****************************************************************************/
	/*                           ÏµÍ³º¯Êý³õÊ¼»¯                                 */
	/****************************************************************************/

	System_Init();
	/*****************************************************************************/
	/*                           ¶¥²ãÄ£Ê½Ñ¡ÔñÄ£                                  */
	/*****************************************************************************/
	while(1)    
	{
		key_scan();
	  if(str[0]=='1')
		{
			str_i=0;	
			ShowQQChar(0x80,"¿Ø·ù°Ú£º",4);
			memset(str, 0, sizeof(str));
			data=0;
			LCDInit();
			
			while(1)
			{
				key_scan();
				delay_ms(500);
				if(data!=0)
				{
						MPU_Init();
	          while(mpu_dmp_init()==1);
					x=data;
					TIM5_Int_Init(10000-1,42-1);
  aim=atan(x/88.3)*57.3*1.03;
	S.Kp=400;
  S.Ki=30;
  S.Kd=2;
while(1)
	{		
	  if(fabs((float)roll)>roll_max)
	  roll_max=abs((float)roll);
		GPIO_SetBits(GPIOA,GPIO_Pin_14);
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
			GPIO_SetBits(GPIOA,GPIO_Pin_15);
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
			}
		}
		else if(str[0]=='2')
		{
			str_i=0;
			ShowQQChar(0x80,"¿Ø½Ç°Ú£º",4);
			memset(str, 0, sizeof(str));
			data=0;
			LCDInit();
			while(1)
			{
				key_scan();
				delay_ms(500);
				if(data!=0)
				{
					if(data<=90)
					{
					alfa=data;
												MPU_Init();
	          while(mpu_dmp_init()==1);
					TIM5_Int_Init(10000-1,42-1);
					GPIO_SetBits(GPIOA,GPIO_Pin_14);
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
	 
				else if(data>90&&data<=180)
				{
					while(1)
	      {	
	       alfa=180-data;
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
		while((fabs(roll)<15.0f)&&(fabs(pitch)<15.0f))
		{
			//if(fabs(tan(roll)/tan(pitch))>fabs(theta_1))
			//{
		//		TIM_SetCompare3(TIM3,5000);
		//	}
			//else
			//{
		//		TIM_SetCompare1(TIM3,5000);
		///	}
			if(roll_v>0||pitch_v>0)
      {
			   TIM_SetCompare2(TIM3,x_v);
			   TIM_SetCompare1(TIM3,0);
			   TIM_SetCompare3(TIM3,y_v);
			   TIM_SetCompare4(TIM3,0);
			}
			else
			{
			   TIM_SetCompare1(TIM3,x_v);
			   TIM_SetCompare2(TIM3,0);
			   TIM_SetCompare4(TIM3,y_v);
			   TIM_SetCompare3(TIM3,0);		
			}
			
		}
			    TIM_SetCompare1(TIM3,0);
          TIM_SetCompare2(TIM3,0);
          TIM_SetCompare3(TIM3,0);
          TIM_SetCompare4(TIM3,0);
	  }
					
				}
		}
	}
	}
		else if(str[0]=='3')
		{
			str_i=0;
			ShowQQChar(0x80,"ÖÆ¶¯Í££º",4);
			memset(str, 0, sizeof(str));
			EXTI_GenerateSWInterrupt(EXTI_Line2);
		}
		else if(str[0]=='4')
		{
			str_i=0;
			ShowQQChar(0x80,"»­Ô²°Ú£º",4);
			memset(str, 0, sizeof(str));
			data=0;
			LCDInit();
			while(1)
			{	
				key_scan();
				delay_ms(500);
				if(data!=0)
				{
			    x=data;
											MPU_Init();
	          while(mpu_dmp_init()==1);
					GPIO_SetBits(GPIOA,GPIO_Pin_14);
	        aim=atan(x/88.3)*57.3*1.03;
			    TIM5_Int_Init(10000-1,42-1);
					if(x>32.5)
					{
						d_v=1500;
						b_v=1530;
					}
					else if(x>27.5)
					{
						d_v=1200;
						b_v=1200;
					}
					else if(x>22.5)
					{
						d_v=920;
						b_v=920;
					}
					else if(x>17.5)
					{
						d_v=825;
						b_v=800;
					}
					else
					{
						
					}
	
	      while(1)
	      {
	       c=flag_b;
	       d=2550*sin(c*0.041);
	       b=2550*cos(c*0.041);
	       if(d>=0)
	       {
            TIM_SetCompare1(TIM3,(int)d);
			      TIM_SetCompare2(TIM3,0);
	       }
	       else
	       {
		        TIM_SetCompare2(TIM3,(int)(-d));
			      TIM_SetCompare1(TIM3,0);
	       }
	       if(b>=0)
	       {
	          TIM_SetCompare4(TIM3,(int)b);
	          TIM_SetCompare3(TIM3,0);
	       }
	       else
	       {
	          TIM_SetCompare3(TIM3,(int)(-b));
	          TIM_SetCompare4(TIM3,0);
	       }
	       if(fabs(pitch)>aim||fabs(roll)>aim)
	       {
		        while(1)
		        {
				       c=flag_b;
	             d=d_v*sin(c*0.041);
	             b=b_v*cos(c*0.041);
	             if(d>=0)
	             {
                 TIM_SetCompare1(TIM3,(int)d);
			           TIM_SetCompare2(TIM3,0);
	             }
	             else
	             {
		              TIM_SetCompare2(TIM3,(int)(-d));
			            TIM_SetCompare1(TIM3,0);
	             }
	             if(b>=0)
	             {
	                TIM_SetCompare4(TIM3,(int)b);
	                TIM_SetCompare3(TIM3,0);
	             }
	             else
	             {
	                TIM_SetCompare3(TIM3,(int)(-b));
	                TIM_SetCompare4(TIM3,0);
	             }
			
		        }
	      }
     }
	 }
 }
}
		else if(str[0]=='5')
		{
			str_i=0;
			ShowQQChar(0x80,"¿¹ÈÅ°Ú£º",4); 
			memset(str, 0, sizeof(str));
			EXTI_GenerateSWInterrupt(EXTI_Line4);
		}
	}
	/**************************************************************/
	/*                        Æð°Ú1                                */
	/**************************************************************/
	/*while(1)
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
	 } */
	/**************************************************************/
	/*                        Æð°Ú2                                */
	/**************************************************************/
	/*while(1)
	{
		while(fabs(pitch<30))
		{
	  
		if(roll_v>0)
		{
			TIM_SetCompare1(TIM3,1000);
			TIM_SetCompare2(TIM3,0);
		}
		else 
		{
			TIM_SetCompare2(TIM3,2000);
			TIM_SetCompare1(TIM3,0);
		}
		if(pitch_v>0)
		{
			TIM_SetCompare3(TIM3,2000);
			TIM_SetCompare4(TIM3,0);
			
		}
		else
		{
			TIM_SetCompare4(TIM3,2000);
			TIM_SetCompare3(TIM3,0);
		
	 } 
 }
		
 TIM_SetCompare1(TIM3,0);
 TIM_SetCompare2(TIM3,0);
 TIM_SetCompare3(TIM3,0);
 TIM_SetCompare4(TIM3,0);
		while(1)
		{
		if(roll_v>0)
		{
			TIM_SetCompare1(TIM3,1000);
			TIM_SetCompare2(TIM3,0);
		}
		else 
		{
			TIM_SetCompare2(TIM3,2000);
			TIM_SetCompare1(TIM3,0);
		}
	}
	}*/
/***********************************************************/
/*                        270-360¿Ø½Ç°Ú                    */
/***********************************************************/
/*	while(1)
	{	
	  alfa=180-gama;
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
		while(fabs(roll)<18.0f||fabs(pitch)<18.0f)
		{
			//if(fabs(tan(roll)/tan(pitch))>fabs(theta_1))
			//{
		//		TIM_SetCompare3(TIM3,5000);
		//	}
			//else
			//{
		//		TIM_SetCompare1(TIM3,5000);
		///	}
			if(roll_v>0||pitch_v>0)
      {
			   TIM_SetCompare2(TIM3,x_v);
			   TIM_SetCompare1(TIM3,0);
			   TIM_SetCompare3(TIM3,y_v);
			   TIM_SetCompare4(TIM3,0);
			}
			else
			{
			   TIM_SetCompare1(TIM3,x_v);
			   TIM_SetCompare2(TIM3,0);
			   TIM_SetCompare4(TIM3,y_v);
			   TIM_SetCompare3(TIM3,0);		
			}
			
		}
			    TIM_SetCompare1(TIM3,0);
          TIM_SetCompare2(TIM3,0);
          TIM_SetCompare3(TIM3,0);
          TIM_SetCompare4(TIM3,0);
		while(1)
		{
			
			;
		}
	}*/
		
/****************************************************************/
/*                         »­Ô²                                 */
/****************************************************************/
/*while(1)
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
}*/

/****************************************************************/
/*                         »­Ô² -2                                */
/****************************************************************/
/*while(1)
{
	while(fabs(roll)<8)
	{
	if(roll_v>0)
		{
			TIM_SetCompare2(TIM3,circle);
			TIM_SetCompare1(TIM3,0);
		}
		else
		{
			TIM_SetCompare1(TIM3,circle);
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
			TIM_SetCompare2(TIM3,00);
		}
		if(fabs(roll_v)<2)
		{
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
	  if(pitch_v<0)
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
}*/


/****************************************************************/
/*                         »­Ô²-3                                 */
/****************************************************************/
/*aim=atan(x/88.3)*57.3;
	S.Kp=500;
  S.Ki=0;
  S.Kd=0;
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
			while(fabs(pitch)<10)
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
				if(fabs(roll_v)<0.5&&fabs(roll)>5)
				{
					roll_max=fabs(roll);
					err_v=aim-fabs(roll_max);
					PWM_1=arm_pid_speed(&S,err_v);
					//PWM_1=err_v*200;
		      delay_ms(50);
				}
					if(roll_v>0)
		      {
						if(PWM_1>0)
						{
			        TIM_SetCompare2(TIM3,PWM_1);
			        TIM_SetCompare1(TIM3,0);
						}
						else
						{
						  TIM_SetCompare1(TIM3,-PWM_1);
			        TIM_SetCompare2(TIM3,0);
						}
		      }
					else
					{
						if(PWM_1>0)
						{
						   TIM_SetCompare1(TIM3,+PWM_1);
			         TIM_SetCompare2(TIM3,0);
						}
						else
						{
						   TIM_SetCompare2(TIM3,-PWM_1);
			         TIM_SetCompare1(TIM3,0);
						}
						
					}
	     if(fabs(pitch_v)<0.5&&fabs(pitch)>5)
				{
					pitch_max=fabs(pitch);
					err_v_2=aim-fabs(pitch_max);
					PWM_2=arm_pid_speed(&S,err_v_2);
					//PWM_1=err_v*200;
          delay_ms(50);
				}
					if(pitch_v>0)
		      {
						if(PWM_2>0)
						{
			        TIM_SetCompare3(TIM3,PWM_2);
			        TIM_SetCompare4(TIM3,0);
						}
						else
						{
						  TIM_SetCompare4(TIM3,-PWM_2);
			        TIM_SetCompare3(TIM3,0);
						}
		      }
					else
					{
						if(PWM_2>0)
						{
						   TIM_SetCompare4(TIM3,+PWM_2);
			         TIM_SetCompare3(TIM3,0);
						}
						else
						{
						   TIM_SetCompare3(TIM3,-PWM_2);
			         TIM_SetCompare4(TIM3,0);
						}				
					}
		  }	
		}
	}
}*/

}

/**********************************************************/
/*                 ¶¨·ù»­Ô²                               */
/**********************************************************/
/*aim=atan(x/88.3)*57.3;

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
				if(fabs(roll_v)<0.5)
				{
					roll_max=fabs(roll);
					err_v=aim-fabs(roll);
					PWM_1=arm_pid_speed(&S,err_v)*1.2;
					//PWM_1=err_v*200;
					delay_ms(50);
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
				}	
			}	
		}

}*/



void System_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
  LCDInit();
	MPU_Init();
	while(mpu_dmp_init()==1);
	Remote_Init();		//ºìÍâ½ÓÊÕ³õÊ¼»¯	
	TIM3_PWM_Init(5000,0);
	EXTIX_Init();
	ShowQQChar(0x92,"SYS OK",3);
	lcd_wcmd(0x01); //Çå³ýÏÔÊ¾DDRAM
	S.Kp=500;
  S.Ki=0;
  S.Kd=0;
	arm_pid_init_f32(&S,0);
	TIM4_Int_Init();
	delay_ms(100);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//??PORTA??	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
	GPIO_Init(GPIOA,&GPIO_InitStructure); //???PA0
	GPIO_ResetBits(GPIOA,GPIO_Pin_14);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	
	
	

	}
		
  
	




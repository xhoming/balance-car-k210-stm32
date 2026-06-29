#include "bsp_ultrasonic.h"

u32 g_distance = 0;//超声波距离  Ultrasonic distance

//这是仅仅避障 This is just obstacle avoidance
void APP_avoid(void)
{
	if(g_distance<250) 
	{
			Move_X = -10;
			my_delay(1); 
		
			Move_X = 0;
			Move_Z = 450;
			my_delay(1);
			Move_Z = 0;
	}
	else
	{
			Move_X = 15;
			Move_Z = 0;
	}


}

//超声波距离控制小车 放到定时器6服务里
//这是避障和跟随相结合
//Ultrasonic distance control car placed in timer 6 service
//This is a combination of obstacle avoidance and following
void App_Change_Car(void)
{
	
	//小车的状态不处于静止状态或者超声波状态
	//The state of the car is not in a stationary state or ultrasonic state
	if(g_newcarstate != enSTOP && g_newcarstate != enAvoid && g_newcarstate != enFollow)
	{
		return;
	}
	
	
	//如果超声波没插或者异常
	//If the ultrasound is not inserted or abnormal
	if(g_distance == 0)
	{
		return;
	}
	
	if(g_distance >20 && g_distance<100)
	{
		g_newcarstate = enAvoid;//避障  Obstacle avoidance
	}
	else if(g_distance >240 && g_distance<300)
	{
		g_newcarstate = enFollow;//跟随  follow
	}
	else
	{
		g_newcarstate = enSTOP;//停止状态  stop
	}
	
	

}


void ultrasonic_init(void)
{
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);//打开定时器2通道2的输入捕获中断 Enable the input capture interrupt of timer 2 channel 2
	HAL_TIM_Base_Start_IT(&htim2);            //打开定时器2通道2的更新中断 Enable the update interrupt of timer 2 channel 2


}




//获取超声波的距离
//Distance to obtain ultrasonic waves
u16 TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL;
void Get_Distane(void)        
{   
	 TRIG_SIG = 1;         
	 delay_us(15);  
	 TRIG_SIG = 0;	
	 if(TIM2CH2_CAPTURE_STA&0X80)//成功捕获到了一次高电平 //Successfully captured a high level once
	 {
		 g_distance=TIM2CH2_CAPTURE_STA&0X3F; 
		 g_distance*=65536;					        //溢出时间总和 Overflow time sum
		 g_distance+=TIM2CH2_CAPTURE_VAL;		//得到总的高电平时间 Get the total high level time
		 g_distance=g_distance*170/1000;      //时间*声速/2（来回） 一个计数0.001ms  Time * speed of sound/2 (round trip), one count 0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//开启下一次捕获 Start the next capture
	 }				
}

//超声波回波脉宽读取中断 Ultrasonic echo pulse width reading interruption
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行 Execute when a capture interrupt occurs
{ 		    		  			    
	if(htim==&htim2)
	{
	if((TIM2CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	Not captured yet
		{
			if(TIM2CH2_CAPTURE_STA&0X40)  //捕获到一个下降沿 Capture a falling edge
			{
				 TIM2CH2_CAPTURE_STA|=0X80;  //标记成功捕获到一次高电平脉宽 The marker successfully captures a high level pulse width
				 TIM2CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//获取当前的捕获值. Get the current capture value.
				 __HAL_TIM_DISABLE(&htim2);
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2);   //一定要先清除原来的设置！！ Be sure to clear the original settings first!!
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM2通道2上升沿捕获 Configure TIM2 channel 2 rising edge capture
				 __HAL_TIM_ENABLE(&htim2);//使能定时器2 Enable timer 2
			}
			else  								     //还未开始,第一次捕获上升沿  Not started yet, first capture rising edge
			{
				 TIM2CH2_CAPTURE_STA=0;	 //清空 Clear
				 TIM2CH2_CAPTURE_VAL=0;
				 TIM2CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿  The marker captures the rising edge
				 //配置tim前一定要先关闭tim，配置完以后再使能 Before configuring tim, be sure to disable tim first, and then enable it after configuration.
				 __HAL_TIM_DISABLE(&htim2);        //关闭定时器2 Turn off timer 2
				 __HAL_TIM_SET_COUNTER(&htim2,0);  //计数器CNT置0 Counter CNT is set to 0
				 TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2);   //一定要先清除原来的设置！！ Be sure to clear the original settings first!!
				 TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时器3通道3设置为下降沿捕获 Timer 3 channel 3 is set to capture the falling edge
				 __HAL_TIM_ENABLE(&htim2);//使能定时器2 Enable timer 2
			}
		}
	}
}






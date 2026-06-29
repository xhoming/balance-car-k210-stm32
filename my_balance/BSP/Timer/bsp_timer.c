#include "bsp_timer.h"

static float battery_All;
static uint8_t battery_count=0,battery_flag=0;

static u16 stop_time = 0;//延迟时间  delay time

u16 led_flag = 0; //1:进入闪烁状�?0:等待闪烁 //1: Entering flashing state 0: waiting for flashing
u16 led_twinkle_count = 0;// 闪烁计数  //Flashing Count

u16 led_count = 0; //开始计�? //Start counting

u8 lower_power_flag = 0; //低电压标�? 0:电压正常 1：低�? //Low Voltage Flag 0: Normal Voltage 1: Low Voltage


//定时�?做延�?10ms的延�?此方法比delay准确
//Timer 6 has a delay of 10ms. This method is more accurate than delay
void delay_time(u16 time)
{
	stop_time = time;
	while(stop_time);//死等 Wait
}

//延迟1s  Unit second
void my_delay(u16 s)//s
{
	for(int i = 0;i<s;i++)
	{
		delay_time(100);
	}
}


/**************************************************************************
Function function: TIM6 initialization, timed for 10 milliseconds
Entrance parameters: None
Return value: None
函数功能：TIM6初始化，定时10毫秒
入口参数：无
返回  值：�?
**************************************************************************/
void TIM6_Init(void)
{
	// 打开定时器中�?
	// Turn on timer interrupt
	HAL_TIM_Base_Start_IT(&htim6);
}

u8 bulettohflag = 0;

// TIM6中断
// 此回调函数可放多个定时器处理
// 传入参数：定时器结构�?
// This callback function can handle multiple timers
// Incoming parameter: Timer structure
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		led_count++;  //led服务显示标志 LED service display logo
		battery_flag ++;		//电量显示标志	 Electricity display sign
		
		if(stop_time>0)
		{
			stop_time --;
		}
		
		if(mode == Bluetooth_Mode || mode == ChaseLine_Mode)
			Get_Distane();//获取距离  Get distance
		
		if(mode == Bluetooth_Mode)
		{
			bulettohflag = 1;
		}
		
		
////////电压检测流�?	 Voltage detection process
		if(battery_flag > 2)//20ms
		{		
			battery_flag = 0;
			battery_All += Get_Battery_Volotage();//获取电源电量 Obtain the power level of the power supply
			battery_count++;
			if(battery_count == 50)//1000ms
			{
				battery = battery_All/50; //平均�?average value
				battery_All = 0; 
				battery_count = 0;
				power_decect();//电压处理  Voltage processing
			}
			
		}
///////////
		
		cotrol_led();//灯服�? led service
		
				
		
	}
}


void power_decect(void)
{
	static u8 normal_power_flag = 1; //电压恢复标志 0：没恢复 1:恢复 //Voltage recovery flag 0: not restored 1: restored
	if(battery < 9.6) //小于9.6V报警 //Alarm below 9.6V
	{
		lower_power_flag = 1;
		normal_power_flag = 0;
	}
	else
	{
		if(normal_power_flag == 0)
		{
			lower_power_flag = 0;
			normal_power_flag = 1;
			BEEP_BEEP = 0;
		}
		
	}
}

void cotrol_led(void)
{
	//灯的效果和蜂鸣器的效�?低压报警 //The effect of the lamp and buzzer is low voltage alarm
		if(!led_flag)
		{
			if(led_count>300)//3S
			{
				led_count = 0;
				led_flag = 1;
			}
		}
		else
		{
			if(led_count>20)//200ms
			{
				led_count = 0;
				
				if(lower_power_flag == 0)
				{
					LED = !LED;//状态反�?//State reversal
				}
				else
				{
					BEEP_BEEP = !BEEP_BEEP;
					LED = 1;//低压蓝灯常亮 //Low voltage blue light is always on
				}
				
				led_twinkle_count++;
				if(led_twinkle_count == 6)
				{
					if(lower_power_flag == 0)
					{
						LED = 0;
					}
					else
					{
						BEEP_BEEP = 0;
					}
					
					led_twinkle_count = 0;
					led_flag = 0;
				}
				
			}
		}

}


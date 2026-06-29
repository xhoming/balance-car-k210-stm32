#include "app.h"

extern uint8_t k210_new;

extern u8 newLineReceived;//蓝牙接收 Bluetooth reception
extern int int9num;
extern u8 bulettohflag;

extern char showbuf[32];


void app_user(void)
{
	if(mode == Bluetooth_Mode)
	{
		if (newLineReceived)
		{
			if (int9num == 3)
				ProcessCarProtocol();
			else if (int9num == 9)
				ProcessPIDFrame();
			CarHexForward();
			newLineReceived = 0;
		}
		CarUltrasonicCheck();
		CarTimeoutCheck();
		CarTelemSend();

		sprintf(showbuf,"%s  hdg:% 6.1f",
		        Car_Diff_IsLocked() ? "locked" : "unlock",
		        Car_Diff_Heading());
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"S:%d T:%d gy:%.1f",
		        (int)Car_Target_Velocity, (int)Move_Z, Gyro_Turn);
		OLED_Draw_Line(showbuf, 3, false, true);
	}
	else if(mode == ChaseLine_Mode)
	{
		CarUltrasonicCheck();
		CarTimeoutCheck();
		CarTelemSend();

		sprintf(showbuf,"E:%d S:%d C:%d A:%d",
		        (int)g_vision_input.error,
		        (int)g_vision_input.slope,
		        (int)g_vision_input.confidence,
		        (int)(HAL_GetTick() - g_vision_input.last_update_ms));
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"DV:%d DT:%d SP:%d",
		        (int)g_vision_debug_v,
		        (int)g_vision_debug_t,
		        (int)g_vision_input.base_speed);
		OLED_Draw_Line(showbuf, 3, false, true);
	}
}

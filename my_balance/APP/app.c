#include "app.h"


extern u8 newLineReceived;
extern int int9num;

extern char showbuf[32];


void app_user(void)
{
	if(mode == Bluetooth_Mode)
	{
		if (newLineReceived)
		{
			if (int9num == 3)
				ProcessCarProtocol();
			newLineReceived = 0;
		}
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
		CarTelemSend();

		sprintf(showbuf,"E:%d S:%d C:%d A:%d",
		        (int)g_vision_input.error,
		        (int)g_vision_input.slope,
		        (int)g_vision_input.confidence,
		        (int)(HAL_GetTick() - g_vision_input.last_update_ms));
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"V:%d PWM:%d SP:%d",
		        (int)g_vision_debug_v,
		        (int)g_vision_debug_t,
		        (int)g_vision_input.base_speed);
		OLED_Draw_Line(showbuf, 3, false, true);
	}
	else if(mode == KickBall_Mode)
	{
		CarTelemSend();

		sprintf(showbuf,"B:%d E:%d N:%d Y:%d",
		        (int)g_ball_debug_state,
		        (int)g_ball_input.error,
		        (int)g_ball_input.area,
		        (int)g_ball_input.speed);
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"V%02d A%03u F%03u R%03u",
		        (int)g_ball_debug_v,
		        (unsigned int)(((HAL_GetTick() - g_ball_input.last_update_ms) > 999) ?
		                       999 : (HAL_GetTick() - g_ball_input.last_update_ms)),
		        (unsigned int)(g_ball_rx_frames % 1000),
		        (unsigned int)(g_ball_rx_bytes % 1000));
		OLED_Draw_Line(showbuf, 3, false, true);
	}
}

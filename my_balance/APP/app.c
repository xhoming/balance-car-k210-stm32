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

		sprintf(showbuf,"B%d F%d E%d A%d.%d",
		        (int)g_ball_debug_state,
		        (int)g_ball_input.flags,
		        (int)g_ball_input.error,
		        (int)(g_ball_debug_area_x10 / 10),
		        (int)(g_ball_debug_area_x10 % 10));
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"V%d.%d T%d A%03u",
		        (int)(g_ball_debug_v / 10),
		        (int)(g_ball_debug_v % 10),
		        (int)g_ball_debug_t,
		        (unsigned int)g_ball_debug_missed);
		OLED_Draw_Line(showbuf, 3, false, true);
	}
	else if(mode == Goalkeeper_Mode)
	{
		CarTelemSend();

		/* Fixed-width fields erase stale OLED digits; H is input age in ms. */
		sprintf(showbuf,"G%d L%+d S%03u P%+04d ",
		        (int)g_goalkeeper_debug_state,
		        (int)g_goalkeeper_debug_lane,
		        (unsigned int)g_goalkeeper_debug_encoder_speed,
		        (int)g_goalkeeper_debug_turn);
		OLED_Draw_Line(showbuf, 2, false, false);
		sprintf(showbuf,"F%d E%+04d A%03u V%03d",
		        (int)g_goalkeeper_input.flags,
		        (int)g_goalkeeper_input.error,
		        (unsigned int)g_goalkeeper_input.area_x10,
		        (int)g_goalkeeper_debug_speed_x10);
		OLED_Draw_Line(showbuf, 3, false, true);
	}
}

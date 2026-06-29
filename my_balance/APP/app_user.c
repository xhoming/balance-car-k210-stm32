#include "app_user.h"

extern u8 newLineReceived;
extern u8 inputString[80];
extern int int9num;
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;
extern enCarState g_newcarstate;
extern float Move_X, Move_Z;
extern float Car_Target_Velocity, Car_Turn_Amplitude_speed;

// ---- 车辆状�?----
static int16_t car_speed = 0;       // 当前巡航速度 0~CAR_MAX_SPEED
static int8_t  car_dir = 1;         // 1=前进, -1=后退
static int16_t car_turn = 0;        // 累加转向�?-TURN_MAX ~ +TURN_MAX
static uint8_t car_raw[4];          // 最近一帧原始字�?

// ---- K210 接收缓冲 (6字节: A5 speed turn dir chk 5A) ----
static uint8_t  k210_buf[6];
static uint8_t  k210_idx = 0;
uint8_t  k210_new = 0;

// ---- 遥测 ----
static uint32_t car_last_fwd_ms  = 0;
static uint32_t car_last_telem_ms = 0;

// ---- PID 调参 (蓝牙可改�? ----
float Car_Turn_Kp = 64.0f;
float Car_Turn_Ki = 0.11f;
float Car_Turn_Kd = 0.7f;
static uint8_t pid_init_cnt = 0;


// =============================================================================
// 两级模式选择: System / User �?具体模式
// =============================================================================
void Mode_select_v2(void)
{
	// 直接选模�? 1.ChaseLine  2.Bluetooth
	int16_t mode_cnt = 0;
	mode = ChaseLine_Mode;
	OLED_Draw_Line("1.ChaseLine Mode", 1, true, true);

	while (!Key1_State(1))
	{
		mode_cnt += Read_Encoder(MOTOR_ID_ML);
		mode_cnt += -Read_Encoder(MOTOR_ID_MR);
		car_mode_range(mode_cnt, ChaseLine_Mode, Bluetooth_Mode);
		show_mode_oled();
	}
	while (Key1_State(1));

	Set_Mid_Angle();
	Set_angle();
	Set_control_speed();
	Set_PID();

	// Bluetooth/ChaseLine 模式确认�? 陀螺闭环重新校�?+ PID初始�?
	if (mode == Bluetooth_Mode || mode == ChaseLine_Mode)
	{
		Car_Diff_Turn_Reset();
		VisionTurn_Reset();
		if (mode == Bluetooth_Mode)
			StartPIDInit();
	}
}


// =============================================================================
// 限制范围的模式切�?
// =============================================================================
void car_mode_range(int16_t cnt, Car_Mode min_mode, Car_Mode max_mode)
{
	static int16_t cnt_old;
	if (myabs(myabs(cnt) - myabs(cnt_old)) > 250)
	{
		if (cnt < cnt_old)
		{
			mode = (mode == min_mode) ? max_mode : (Car_Mode)(mode - 1);
		}
		else
		{
			mode = (mode == max_mode) ? min_mode : (Car_Mode)(mode + 1);
		}
		cnt_old = cnt;
	}
}


// =============================================================================
// 新版汽车协议 �?解析 A5 <flags> <chksm> 5A �?
// =============================================================================
void ProcessCarProtocol(void)
{
	// 必须恰好 4 字节: 索引 0=A5, 1=flags, 2=chksm, 3=5A
	if (int9num != 3) return;

	uint8_t hdr   = inputString[0];
	uint8_t flags = inputString[1];
	uint8_t chk   = inputString[2];
	uint8_t ftr   = inputString[3];

	// 校验
	if (hdr != BT_HEADER || ftr != BT_FOOTER) return;
	if (chk != flags) return;   // 校验�?= flags 自身

	// 保存原始�?
	memcpy(car_raw, inputString, 4);

	// ---- 解析 ----
	uint8_t left     = flags & BT_LEFT;
	uint8_t right    = flags & BT_RIGHT;
	uint8_t speed_up = flags & BT_SPEED_UP;
	uint8_t speed_dn = flags & BT_SPEED_DN;
	uint8_t forward  = flags & BT_FORWARD;
	uint8_t backward = flags & BT_BACKWARD;

	// 方向 (只有显式指令才切�?
	if (backward) car_dir = -1;
	if (forward)  car_dir =  1;

	// 油门
	if (speed_up)
	{
		car_speed += CAR_ACCEL_STEP;
		if (car_speed > CAR_MAX_SPEED) car_speed = CAR_MAX_SPEED;
	}
	if (speed_dn)
	{
		car_speed -= CAR_DECEL_STEP;
		if (car_speed < 0) car_speed = 0;
	}

	// 转向: 累加目标航向, 永不自动归零
	if (left && !right)
		car_turn -= CAR_TURN_STEP;
	else if (right && !left)
		car_turn += CAR_TURN_STEP;

	if (car_turn >  CAR_TURN_MAX) car_turn =  CAR_TURN_MAX;
	if (car_turn < -CAR_TURN_MAX) car_turn = -CAR_TURN_MAX;

	// ---- 施加�?PID ----
	g_newcarstate = enSTOP;
	Move_X = car_dir * car_speed;
	Move_Z = car_turn;
	Car_Target_Velocity = car_speed;
	Car_Turn_Amplitude_speed = CAR_TURN_MAX;
}


// =============================================================================
// 蓝牙原始帧转发到 USART1 (100ms 限频, 超频帧直接丢�?
// =============================================================================
void CarHexForward(void)
{
	uint32_t now = HAL_GetTick();
	if (now - car_last_fwd_ms < 100) return;  // 100ms 内不再转�?
	car_last_fwd_ms = now;

	printf("%02X %02X %02X %02X\r\n",
	       car_raw[0], car_raw[1], car_raw[2], car_raw[3]);
}


// =============================================================================
// K210 接收: USART2 中断逐字�? 6字节�?A5 speed turn dir chk 5A
// =============================================================================
void Deal_K210_Car(uint8_t rx)
{
	if (rx == BT_HEADER)
	{
		k210_idx = 0;
		k210_buf[k210_idx++] = rx;
	}
	else if (k210_idx > 0 && k210_idx < 6)
	{
		k210_buf[k210_idx++] = rx;
		if (k210_idx == 6 && k210_buf[5] == BT_FOOTER)
			k210_new = 1;
	}
}

// =============================================================================
// K210 帧处�? 直接赋�?(不累�?, 6字节 A5 speed turn dir chk 5A
// =============================================================================
void Deal_K210_Vision(uint8_t rx)
{
	VisionTurn_ParseByte(rx);
}

void ProcessK210Frame(void)
{
	if (!k210_new) return;
	k210_new = 0;

	uint8_t hdr   = k210_buf[0];
	uint8_t speed = k210_buf[1];
	int8_t  turn  = (int8_t)k210_buf[2];
	uint8_t dir   = k210_buf[3];
	uint8_t chk   = k210_buf[4];
	uint8_t ftr   = k210_buf[5];

	if (hdr != BT_HEADER || ftr != BT_FOOTER) return;
	if ((uint8_t)(speed + (uint8_t)turn + dir) != chk) return;
	memcpy(car_raw, k210_buf, 6);

	// speed直接赋�? turn增量累加
	car_speed = speed;
	if (car_speed > CAR_MAX_SPEED) car_speed = CAR_MAX_SPEED;
	car_turn = turn;                                  // 增量!
	if (car_turn >  CAR_TURN_MAX)  car_turn =  CAR_TURN_MAX;
	if (car_turn < -CAR_TURN_MAX)  car_turn = -CAR_TURN_MAX;
	car_dir   = (dir == 2) ? -1 : 1;

	g_newcarstate = enSTOP;
	Move_X = car_dir * car_speed;
	Move_Z = car_turn;
	Car_Target_Velocity = car_speed;
	Car_Turn_Amplitude_speed = CAR_TURN_MAX;
}


// =============================================================================
// 紧急停�?(蓝牙断连时安全刹�?
// =============================================================================
void CarEmergencyStop(void)
{
	car_speed = 0;
	car_turn = 0;
	Move_X = 0;
	Move_Z = 0;
	Car_Target_Velocity = 0;
	Car_Turn_Amplitude_speed = 0;
	g_newcarstate = enSTOP;
}


// =============================================================================
// 遥测发�? �?00ms通过蓝牙回传状�? A5 speed turn dir chk 5A
// =============================================================================
void CarTelemSend(void)
{
	uint32_t now = HAL_GetTick();
	if (now - car_last_telem_ms < 100) return;
	car_last_telem_ms = now;

	// 初始化阶�? �?00ms发一次PID参数, �?0�?
	if (pid_init_cnt > 0)
	{
		SendPIDParams();
		pid_init_cnt--;
	}

	// 发�? A5 speed turn target dir chk 5A (7字节)
	uint8_t speed   = (uint8_t)car_speed;

	float hdg = Car_Diff_Heading();
	int8_t  turn_s  = (int8_t)((int)hdg);       // 实际航向
	int8_t  tgt_s   = (int8_t)car_turn;         // 目标航向
	uint8_t turn_u  = (uint8_t)turn_s;
	uint8_t tgt_u   = (uint8_t)tgt_s;
	uint8_t dir     = (car_dir > 0) ? 1 : 2;
	uint8_t chk     = speed + turn_u + dir + tgt_u;

	uint8_t telem[7] = {BT_HEADER, speed, turn_u, dir, tgt_u, chk, BT_FOOTER};

	UART5_DataByte(telem[0]);
	UART5_DataByte(telem[1]);
	UART5_DataByte(telem[2]);
	UART5_DataByte(telem[3]);
	UART5_DataByte(telem[4]);
	UART5_DataByte(telem[5]);
	UART5_DataByte(telem[6]);

	if (mode == ChaseLine_Mode)
		USART2_Send_ArrayU8(telem, 7);

	printf("[T] %02X %02X %02X %02X %02X %02X %02X\r\n",
	       telem[0], telem[1], telem[2], telem[3], telem[4], telem[5], telem[6]);
}



// =============================================================================
// 超声波限�? sigmoid 平滑过渡  <30cm�?  >100cm�?0  中间平滑
// =============================================================================
void CarUltrasonicCheck(void)
{
	extern u32 g_distance;
	if (g_distance == 0) return;  // 无效读数

	int16_t limit = CAR_MAX_SPEED;

	if (g_distance <= US_DIST_STOP)
	{
		limit = 0;
	}
	else if (g_distance < US_DIST_FULL)
	{
		// quadratic ease-in-out (sigmoid-like)
		float t = (float)(g_distance - US_DIST_STOP)
		        / (US_DIST_FULL - US_DIST_STOP);  // 0~1
		float eased;
		if (t < 0.5f)
			eased = 2.0f * t * t;
		else
			eased = -1.0f + (4.0f - 2.0f * t) * t;
		limit = (int16_t)(CAR_MAX_SPEED * eased);
	}

	if (car_speed > limit)
	{
		car_speed = limit;
		Move_X = car_dir * car_speed;
		Car_Target_Velocity = car_speed;
	}
}


// =============================================================================
// 差速闭�? 编码器差�?feedback �?叠加在原 PWM 之上, 不动直立和速度
// =============================================================================
// 陀螺闭环内部状�?
static float  heading       = 0;    // 累积航向�?°
static float  heading_zero   = 0;   // 校准航向零点
static float  turn_int      = 0;    // PI积分
static uint8_t calib_done    = 0;
static uint8_t stable_cnt    = 0;
static float  last_gyro      = 0;

void Car_Diff_Turn_Reset(void)
{
	heading        = 0;
	heading_zero   = 0;
	turn_int = 0;
	calib_done     = 0;
	stable_cnt     = 0;
	last_gyro      = 0;
}

uint8_t Car_Diff_IsLocked(void)  { return calib_done; }
float   Car_Diff_Heading(void)   { return heading - heading_zero; }

// 手动重校�? 把当前位置作为新0° (同时清目标角�?
// 手动重校�? 把当前位置作为新0° (仅复位航向目�? 不动速度)
void Car_Diff_Recalibrate(void)
{
	heading_zero = heading;
	turn_int     = 0;
	car_turn     = 0;
	Move_Z       = 0;
}

void Car_Diff_Turn(float gyro_turn, int enc_left, int enc_right)
{
	extern float Turn_Kd;

	// 积分实际航向
	heading += gyro_turn * 0.005f;
	while (heading >  180.0f) heading -= 360.0f;
	while (heading < -180.0f) heading += 360.0f;

	// === 校准: 等稳�? 记录航向零点 ===
	if (!calib_done)
	{
		float diff = gyro_turn - last_gyro;
		if (diff < 0) diff = -diff;
		last_gyro = gyro_turn;

		if (diff < 2.0f)
		{
			stable_cnt++;
			if (stable_cnt >= 20)
			{
				heading_zero   = heading;
				turn_int = 0;
				calib_done     = 1;
				open_beep(30);
			}
		}
		else
			stable_cnt = 0;
		return;
	}

	// === 位置式航向闭�?===
	// car_turn = 目标航向 ° (不缩�? 永不自动归零)
	float target = (float)car_turn;
	float actual = heading - heading_zero;
	while (actual >  180.0f) actual -= 360.0f;
	while (actual < -180.0f) actual += 360.0f;
	float error  = actual - target;
	while (error >  180.0f) error -= 360.0f;
	while (error < -180.0f) error += 360.0f;

	turn_int += error;
	if (turn_int >  2000) turn_int =  2000;
	if (turn_int < -2000) turn_int = -2000;

	// I 项防过冲: error 跨零时清积分
	static float last_error = 0;
	if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0))
		turn_int = 0;
	last_error = error;

	extern float Car_Turn_Kp, Car_Turn_Ki, Car_Turn_Kd;
	int turn_pwm = (int)(error * Car_Turn_Kp)
	             + (int)(turn_int * Car_Turn_Ki)
	             - (int)(gyro_turn * Car_Turn_Kd);      // D: 角速度阻尼

	Motor_Left  = PWM_Limit(Motor_Left  + turn_pwm, 2600, -2600);
	Motor_Right = PWM_Limit(Motor_Right - turn_pwm, 2600, -2600);
}


// =============================================================================
// PID 调参: 9字节�?A5 BL_Kp BL_Kd V_Kp V_Ki T_Kp T_Ki chk 5A
// =============================================================================

void StartPIDInit(void)
{
	pid_init_cnt = 10;   // 启动时发10�?
}

void ProcessPIDFrame(void)
{
	// 帧长必须�?0字节 (A5 + 7 param + chk + 5A)
	if (int9num != 9) return;

	uint8_t hdr = inputString[0];
	uint8_t ftr = inputString[9];
	if (hdr != BT_HEADER || ftr != BT_FOOTER) return;

	int8_t d_bl_kp = (int8_t)inputString[1];
	int8_t d_bl_kd = (int8_t)inputString[2];
	int8_t d_v_kp  = (int8_t)inputString[3];
	int8_t d_v_ki  = (int8_t)inputString[4];
	int8_t d_t_kp  = (int8_t)inputString[5];
	int8_t d_t_ki  = (int8_t)inputString[6];
	int8_t d_t_kd  = (int8_t)inputString[7];
	uint8_t chk    = inputString[8];

	if ((uint8_t)(d_bl_kp + d_bl_kd + d_v_kp + d_v_ki + d_t_kp + d_t_ki + d_t_kd) != chk) return;

	extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;
	extern float Car_Turn_Kp, Car_Turn_Ki, Car_Turn_Kd;
	Balance_Kp  += d_bl_kp * 100.0f;
	Balance_Kd  += d_bl_kd;
	Velocity_Kp += d_v_kp  * 100.0f;
	Velocity_Ki += d_v_ki;
	Car_Turn_Kp += d_t_kp;
	Car_Turn_Ki += d_t_ki / 100.0f;
	Car_Turn_Kd += d_t_kd / 10.0f;

	if (Balance_Kp  < 0)    Balance_Kp  = 0;
	if (Balance_Kd  < 0)    Balance_Kd  = 0;
	if (Velocity_Kp < 0)    Velocity_Kp = 0;
	if (Velocity_Ki < 0)    Velocity_Ki = 0;
	if (Car_Turn_Kp < 0)    Car_Turn_Kp = 0;
	if (Car_Turn_Ki < 0)    Car_Turn_Ki = 0;
	if (Car_Turn_Kd < 0)    Car_Turn_Kd = 0;

	// 回传新参�?
	SendPIDParams();
}

// 蓝牙回传当前 PID 参数
void SendPIDParams(void)
{
	extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;
	extern float Car_Turn_Kp, Car_Turn_Ki, Car_Turn_Kd;

	uint8_t bl_kp = (uint8_t)(Balance_Kp / 100.0f);
	uint8_t bl_kd = (uint8_t)Balance_Kd;
	uint8_t  v_kp = (uint8_t)(Velocity_Kp / 100.0f);
	uint8_t  v_ki = (uint8_t)Velocity_Ki;
	uint8_t  t_kp = (uint8_t)Car_Turn_Kp;
	uint8_t  t_ki = (uint8_t)(Car_Turn_Ki * 100.0f);
	uint8_t  t_kd = (uint8_t)(Car_Turn_Kd * 10.0f);
	uint8_t  chk  = bl_kp + bl_kd + v_kp + v_ki + t_kp + t_ki + t_kd;

	UART5_DataByte(BT_HEADER);
	UART5_DataByte(bl_kp);
	UART5_DataByte(bl_kd);
	UART5_DataByte(v_kp);
	UART5_DataByte(v_ki);
	UART5_DataByte(t_kp);
	UART5_DataByte(t_ki);
	UART5_DataByte(t_kd);
	UART5_DataByte(chk);
	UART5_DataByte(BT_FOOTER);
}


// =============================================================================
// 超时检�? 已禁�?
// =============================================================================
void CarTimeoutCheck(void)
{
	// 用户要求取消超时刹车
}

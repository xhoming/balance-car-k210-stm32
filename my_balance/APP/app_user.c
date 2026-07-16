#include "app_user.h"

extern u8 inputString[80];
extern int int9num;
extern float Move_X, Move_Z;
extern float Car_Target_Velocity;

static int16_t car_speed = 0;
static int8_t car_dir = 1;
static int16_t car_turn = 0;
static uint32_t car_last_telem_ms = 0;

float Car_Turn_Kp = 64.0f;
float Car_Turn_Ki = 0.11f;
float Car_Turn_Kd = 0.7f;

static float heading = 0.0f;
static float heading_zero = 0.0f;
static float turn_int = 0.0f;
static uint8_t calib_done = 0;
static uint8_t stable_cnt = 0;
static float last_gyro = 0.0f;

static void car_mode_range(int16_t cnt, Car_Mode min_mode, Car_Mode max_mode)
{
    static int16_t cnt_old = 0;

    if (myabs(myabs(cnt) - myabs(cnt_old)) <= 250) {
        return;
    }

    if (cnt < cnt_old) {
        mode = (mode == min_mode) ? max_mode : (Car_Mode)(mode - 1);
    } else {
        mode = (mode == max_mode) ? min_mode : (Car_Mode)(mode + 1);
    }
    cnt_old = cnt;
}

void Mode_select_v2(void)
{
    int16_t mode_cnt = 0;

    mode = Goalkeeper_Mode;
    OLED_Draw_Line("4.Goalkeeper Mode", 1, true, true);

    while (!Key1_State(1)) {
        mode_cnt += Read_Encoder(MOTOR_ID_ML);
        mode_cnt += -Read_Encoder(MOTOR_ID_MR);
        car_mode_range(mode_cnt, Bluetooth_Mode, Goalkeeper_Mode);
        show_mode_oled();
    }
    while (Key1_State(1));

    Set_Mid_Angle();
    Set_angle();
    Set_control_speed();
    Set_PID();

    Car_Diff_Turn_Reset();
    VisionTurn_Reset();
    BallKick_Reset();
    Goalkeeper_Reset();
}

void ProcessCarProtocol(void)
{
    uint8_t hdr;
    uint8_t flags;
    uint8_t chk;
    uint8_t ftr;
    uint8_t left;
    uint8_t right;
    uint8_t speed_up;
    uint8_t speed_dn;
    uint8_t forward;
    uint8_t backward;

    if (int9num != 3) {
        return;
    }

    hdr = inputString[0];
    flags = inputString[1];
    chk = inputString[2];
    ftr = inputString[3];

    if (hdr != BT_HEADER || ftr != BT_FOOTER || chk != flags) {
        return;
    }

    left = flags & BT_LEFT;
    right = flags & BT_RIGHT;
    speed_up = flags & BT_SPEED_UP;
    speed_dn = flags & BT_SPEED_DN;
    forward = flags & BT_FORWARD;
    backward = flags & BT_BACKWARD;

    if (backward) {
        car_dir = -1;
    }
    if (forward) {
        car_dir = 1;
    }

    if (speed_up) {
        car_speed += CAR_ACCEL_STEP;
        if (car_speed > CAR_MAX_SPEED) {
            car_speed = CAR_MAX_SPEED;
        }
    }
    if (speed_dn) {
        car_speed -= CAR_DECEL_STEP;
        if (car_speed < 0) {
            car_speed = 0;
        }
    }

    if (left && !right) {
        car_turn -= CAR_TURN_STEP;
    } else if (right && !left) {
        car_turn += CAR_TURN_STEP;
    }

    if (car_turn > CAR_TURN_MAX) {
        car_turn = CAR_TURN_MAX;
    }
    if (car_turn < -CAR_TURN_MAX) {
        car_turn = -CAR_TURN_MAX;
    }

    Move_X = car_dir * car_speed;
    Move_Z = car_turn;
    Car_Target_Velocity = car_speed;
}

void Deal_K210_Vision(uint8_t rx)
{
    VisionTurn_ParseByte(rx);
}

void CarTelemSend(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t speed;
    int8_t turn_s;
    int8_t target_s;
    uint8_t turn_u;
    uint8_t target_u;
    uint8_t dir;
    uint8_t chk;
    uint8_t telem[7];

    if (now - car_last_telem_ms < 100) {
        return;
    }
    car_last_telem_ms = now;

    speed = (uint8_t)car_speed;
    turn_s = (int8_t)((int)Car_Diff_Heading());
    target_s = (int8_t)car_turn;
    turn_u = (uint8_t)turn_s;
    target_u = (uint8_t)target_s;
    dir = (car_dir > 0) ? 1 : 2;
    chk = speed + turn_u + dir + target_u;

    telem[0] = BT_HEADER;
    telem[1] = speed;
    telem[2] = turn_u;
    telem[3] = dir;
    telem[4] = target_u;
    telem[5] = chk;
    telem[6] = BT_FOOTER;

    UART5_DataByte(telem[0]);
    UART5_DataByte(telem[1]);
    UART5_DataByte(telem[2]);
    UART5_DataByte(telem[3]);
    UART5_DataByte(telem[4]);
    UART5_DataByte(telem[5]);
    UART5_DataByte(telem[6]);
}

void Car_Diff_Turn_Reset(void)
{
    heading = 0.0f;
    heading_zero = 0.0f;
    turn_int = 0.0f;
    calib_done = 0;
    stable_cnt = 0;
    last_gyro = 0.0f;
}

uint8_t Car_Diff_IsLocked(void)
{
    return calib_done;
}

float Car_Diff_Heading(void)
{
    return heading - heading_zero;
}

void Car_Diff_UpdateHeading(float gyro_turn)
{
    float diff;

    heading += gyro_turn * 0.005f;
    while (heading > 180.0f) {
        heading -= 360.0f;
    }
    while (heading < -180.0f) {
        heading += 360.0f;
    }

    if (calib_done) {
        return;
    }

    diff = gyro_turn - last_gyro;
    if (diff < 0.0f) {
        diff = -diff;
    }
    last_gyro = gyro_turn;

    if (diff < 2.0f) {
        stable_cnt++;
        if (stable_cnt >= 20) {
            heading_zero = heading;
            turn_int = 0.0f;
            calib_done = 1;
            open_beep(30);
        }
    } else {
        stable_cnt = 0;
    }
}

void Car_Diff_Turn(float gyro_turn)
{
    float target;
    float actual;
    float error;
    int turn_pwm;
    static float last_error = 0.0f;

    if (!calib_done) {
        return;
    }

    target = (float)car_turn;
    actual = heading - heading_zero;
    while (actual > 180.0f) {
        actual -= 360.0f;
    }
    while (actual < -180.0f) {
        actual += 360.0f;
    }

    error = actual - target;
    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }

    turn_int += error;
    if (turn_int > 2000.0f) {
        turn_int = 2000.0f;
    }
    if (turn_int < -2000.0f) {
        turn_int = -2000.0f;
    }

    if ((error > 0.0f && last_error < 0.0f) ||
        (error < 0.0f && last_error > 0.0f)) {
        turn_int = 0.0f;
    }
    last_error = error;

    turn_pwm = (int)(error * Car_Turn_Kp)
             + (int)(turn_int * Car_Turn_Ki)
             - (int)(gyro_turn * Car_Turn_Kd);

    Motor_Left = PWM_Limit(Motor_Left + turn_pwm, 2600, -2600);
    Motor_Right = PWM_Limit(Motor_Right - turn_pwm, 2600, -2600);
}

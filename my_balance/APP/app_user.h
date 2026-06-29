#ifndef __APP_USER_H_
#define __APP_USER_H_

#include "AllHeader.h"

// ========== 模式选择 ==========
void Mode_select_v2(void);
void car_mode_range(int16_t cnt, Car_Mode min_mode, Car_Mode max_mode);

// ========== 新版汽车协议 (A5...5A 二进制帧) ==========

// 协议位定义  Protocol bit definitions
#define BT_HEADER    0xA5
#define BT_FOOTER    0x5A
#define BT_LEFT      0x01   // bit 0
#define BT_RIGHT     0x02   // bit 1
#define BT_SPEED_UP  0x04   // bit 2
#define BT_SPEED_DN  0x08   // bit 3
#define BT_FORWARD   0x10   // bit 4
#define BT_BACKWARD  0x20   // bit 5

// 车辆参数  Vehicle parameters
#define CAR_MAX_SPEED    40
#define CAR_ACCEL_STEP   1
#define CAR_DECEL_STEP   8
#define CAR_TURN_MAX     127
#define CAR_TURN_STEP    2

// 超声波限速门限 (仅对前进生效)
#define US_DIST_STOP     100   // < 10cm 限速=0
#define US_DIST_FULL     200   // > 20cm 全速

// 处理控制帧 (4字节) 和 PID调参帧 (9字节)
void ProcessCarProtocol(void);
void ProcessPIDFrame(void);
void SendPIDParams(void);       // 蓝牙回传当前PID
void StartPIDInit(void);        // 模式启动发10次PID

// 新协议帧 hex 转发到 USART1
void CarHexForward(void);

// K210 巡线帧接收 (USART2 中断调用)
void Deal_K210_Car(uint8_t rx);
void Deal_K210_Vision(uint8_t rx);

// K210 帧处理 (main loop 中调用, 协议与蓝牙一致)
void ProcessK210Frame(void);

// 超时检测: 2秒无帧自动刹车 (在 main loop 中调用)
void CarTimeoutCheck(void);

// 超声波急停: 距离 < 10cm 强制停止 (最高优先级)
void CarUltrasonicCheck(void);

// 每100ms通过蓝牙发送遥测帧: A5 speed turn dir chk 5A
void CarTelemSend(void);

// 陀螺闭环转向: Z轴角速度PI + 陀螺阻尼, 校准复位
void Car_Diff_Turn_Reset(void);
void Car_Diff_Turn(float gyro_turn, int enc_left, int enc_right);
uint8_t Car_Diff_IsLocked(void);
float   Car_Diff_Heading(void);       // 当前航向角 °
void    Car_Diff_Recalibrate(void);   // KEY1 手动重校零

// 紧急停止 (通信中断时安全停)
void CarEmergencyStop(void);

#endif

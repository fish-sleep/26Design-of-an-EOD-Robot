#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

/*
 * ============================================================
 * 麦轮底盘运动控制
 *
 * 【电机布局】(已确认)
 *            车头
 *      4(左前)      1(右前)
 *      3(左后)      2(右后)
 *
 * 【本文件提供的接口】
 *   底层:  fish_Step_All_Control()  直接给四个电机设速度并同步启动
 *   核心:  Car_Mecanum()            麦轮逆运动学, 前进/横移/自转任意组合
 *   常用:  Car_Drive()              闭环下发入口(前进 + 横移 + 自转, 三量任意组合)
 *   语义:  Car_MoveForward / MoveBackward / MoveLeft / MoveRight
 *          Car_TurnLeft / Car_TurnRight   (原地转向)
 *          Car_Stop
 * ============================================================
 */

#include "Emm_V5.h"
#include "servo.h"

/* 【底层】分别设定四个电机的方向和速度, 最后广播同步启动。
 * dir  : 0 = 后退, 1 = 前进(见 Car_control.c 里的方向约定)
 * vel  : 转速 RPM, 范围 0~5000
 * 注意 : 内部有 4 个 HAL_Delay(2), 一次调用阻塞 8ms 以上,
 *        绝对不能在中断里调用。 */
void fish_Step_All_Control(uint8_t dir1, uint16_t vel1,
                           uint8_t dir2, uint16_t vel2,
                           uint8_t dir3, uint16_t vel3,
                           uint8_t dir4, uint16_t vel4);

/* 【核心】麦轮逆运动学, 三个量任意组合:
 * vx    : 前进速度(RPM), 正数前进, 负数后退
 * vy    : 左移速度(RPM), 正数左移, 负数右移
 * omega : 自转速度(RPM), 正数逆时针(原地左转), 负数顺时针
 * 例: Car_Mecanum(150, 0, 0)     直行
 *     Car_Mecanum(0, 150, 0)     原地左移(横着走)
 *     Car_Mecanum(0, 0, 100)     原地左转
 *     Car_Mecanum(150, 0, 30)    边走边往左偏 */
void Car_Mecanum(int16_t vx, int16_t vy, int16_t omega);

/* 【常用】闭环下发的统一入口: 前进 / 横移 / 自转任意组合。
 * vx    : 前进速度(RPM), >0 前进, <0 后退
 * vy    : 左移速度(RPM), >0 左移(横着往左), <0 右移; 不需要横移就传 0
 * omega : 自转速度(RPM), >0 逆时针(原地左转), <0 顺时针
 *
 * 只给 omega(vy = 0)时就是"四驱差速"那种走法:
 *     vx = 150, omega = 0    -> 直行
 *     vx = 150, omega = 20   -> 直行中往左修正
 *     vx = 0,   omega = 100  -> 原地左转
 * 三个量在运动学上互相独立、可直接叠加, 所以也能三者同时非 0:
 *     vx = 150, vy = 100, omega = 20
 *       -> 一边前进一边整体左移, 同时把车头往左纠偏
 *
 * 说明: omega > 0 时右侧轮比左侧轮快, 车头会向左偏,
 *       正好用来抵消"车头偏右"的航向误差。
 * ⚠ 横移是开环的(车上没有横向位置反馈), 只能给速度, 不能"走到某处停下"。 */
void Car_Drive(int16_t vx, int16_t vy, int16_t omega);

/* 【语义接口】下面是常用动作的封装, 内部都是调用 Car_Mecanum */
void Car_MoveForward(uint16_t vel);     /* 直行前进 */
void Car_MoveBackward(uint16_t vel);    /* 直行后退 */
void Car_MoveLeft(uint16_t vel);        /* 左移(横着往左) */
void Car_MoveRight(uint16_t vel);       /* 右移(横着往右) */
void Car_TurnLeft(uint16_t vel);        /* 原地左转(逆时针) */
void Car_TurnRight(uint16_t vel);       /* 原地右转(顺时针) */

/* 【停车】四个电机立即停止并同步触发。
 * 注意: 内部有 4 个 HAL_Delay(2), 不能在中断里调用。 */
void Car_Stop(void);

/* ==================== 舵机应用层 ====================
 *
 * 底层的读写放在 servo.c 里:
 *     Servo_Init()                     上电初始化
 *     Servo_SetAngle(id, 角度)        直接按角度控制(瞬间到位)
 *     Servo_GetAngle(id)               读当前角度
 *     Servo_SetPulseUs(id, 脉宽)      直接按脉宽控制(标定用)
 *     Servo_GetPulseUs(id)             读当前脉宽
 *     Servo_Enable(id)                 只启动某一路(单路调试/标定用)
 *
 * 舵机 ID (按引脚顺序):
 *     SERVO_ID1 -> PD12     SERVO_ID2 -> PD13
 *     SERVO_ID3 -> PD14     SERVO_ID4 -> PD15
 *     SERVO_ID5 -> PE5
 *
 * 这里放“动作级别”的封装。 */

/* 【平滑转动】在 duration_ms 毫秒内, 把某个舵机从当前位置平滑转到
 * target_angle 度(0~180)。duration_ms 传 0 就是立即到位。
 *
 * 为什么要用它:
 *   直接 Servo_SetAngle() 是瞬间到位, 舵机会以最快速度冲过去 ——
 *   抖得厉害、瞬间抢电流、机械结构冲击也大。
 *   分步插值走完, 动作又稳又好看, 多路舵机一起动时电流也平缓。
 *
 * ⚠ 注意这是**阻塞**函数, 期间不会处理按键和菜单。
 *   跑图 / 航向闭环这类实时逻辑里不要用它。 */
void Servo_MoveTo(uint8_t id, float target_angle, uint16_t duration_ms);

#endif /* __CAR_CONTROL_H */

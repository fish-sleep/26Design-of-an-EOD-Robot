#ifndef __SERVO_H
#define __SERVO_H

/*
 * ============================================================
 * 舵机驱动层 (50Hz PWM) —— 机械臂 5 路
 *
 * 【ID 与引脚对应关系】按引脚顺序编号
 *     ID1 -> PD12   TIM4_CH1
 *     ID2 -> PD13   TIM4_CH2
 *     ID3 -> PD14   TIM4_CH3
 *     ID4 -> PD15   TIM4_CH4
 *     ID5 -> PE5    TIM9_CH1
 *
 * 【为什么可以省掉换算】
 *   两个定时器都配成"1us 分辨率 + 20ms 周期":
 *       APB1 定时器(TIM4, 84MHz) : PSC = 83,  ARR = 19999
 *       APB2 定时器(TIM9, 168MHz): PSC = 167, ARR = 19999
 *   计数频率都是 1MHz, 所以 **CCR 的值直接等于脉宽的微秒数**,
 *   代码里不需要任何换算。
 *
 * 【本文件的职责(驱动层)】
 *   只做三件事: 启动 PWM 输出、角度<->脉宽换算、限幅保护。
 *   机械臂的动作逻辑(缓动、姿态、抓取序列)放在 Car_control.c 里。
 * ============================================================
 */

#include "main.h"

/* ---------------- 舵机 ID ---------------- */
#define SERVO_ID1           0       /* PD12  TIM4_CH1 */
#define SERVO_ID2           1       /* PD13  TIM4_CH2 */
#define SERVO_ID3           2       /* PD14  TIM4_CH3 */
#define SERVO_ID4           3       /* PD15  TIM4_CH4 */
#define SERVO_ID5           4       /* PE5   TIM9_CH1 */

#define SERVO_NUM           5       /* 舵机总数 */

/* ---------------- 接口 ---------------- */

/* 【初始化】上电调用一次。
 * 把 servo.c 里 s_servo_power_on_pulse[] 表的脉宽(us)限幅后**直接**写进比较
 * 寄存器(CCR), 再启动 PWM 输出 —— 不做任何角度换算, 表里写多少 us 就输出多少。
 * 顺序不能反, 否则上电瞬间会先输出 CubeMX 里的初值(0% 占空比),
 * 舵机会先松垮一下再到位。
 * 想改上电姿态只改那张脉宽表即可, 不用动本函数的代码。 */
void Servo_Init(void);

/* 【单路启动】只启动某一路的 PWM 输出, 其余通道保持不输出(松的)。
 * 专给"单路调试/标定"用: 台架上只有这一路会动, 别的几路不会跟着乱动。
 * 调用前建议先用 Servo_SetPulseUs() 写好脉宽, 决定上电第一帧的姿态;
 * 若从未设过脉宽, 内部默认给中位(1500us), 不会冲到行程端点。 */
void Servo_Enable(uint8_t id);

/* 【按角度控制】id 传 SERVO_ID1 ~ SERVO_ID5。
 * angle_deg 范围 0 ~ 180 度, 超出会自动限幅。
 * 内部已经处理了各舵机的方向(dir)和行程范围(pulse_min/max)。 */
void Servo_SetAngle(uint8_t id, float angle_deg);

/* 【读取当前角度】返回上一次设定的角度(0~180), 供缓动/显示用。 */
float Servo_GetAngle(uint8_t id);

/* 【按脉宽控制】直接给脉宽(微秒), 自动限幅。
 * 标定行程或驱非标舵机时用这个更直接。 */
void Servo_SetPulseUs(uint8_t id, uint16_t pulse_us);

/* 【读取当前脉宽】(微秒), 供显示/调试用。 */
uint16_t Servo_GetPulseUs(uint8_t id);

/* 【读取硬件比较寄存器(CCR)实际值】单位微秒。
 * 和 Servo_GetPulseUs() 的区别: 那个读软件缓存, 这个直接读定时器寄存器;
 * 两者一致才说明写的值真的进了硬件。排"脉宽在变但舵机不动"时用得上。 */
uint32_t Servo_GetCcr(uint8_t id);

#endif /* __SERVO_H */

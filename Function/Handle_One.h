#ifndef __HANDLE_ONE_H
#define __HANDLE_ONE_H

/* ============================================================
 * Handle_One —— 航向角 PD 闭环(取姿态数据 + PD 运算 + 电机下发)
 *
 * 【职责】整个 yaw 闭环都收在这个文件里:
 *   - TIM3 中断(20ms): 读一帧 JY901S 姿态数据, 紧接着用最新航向角做 PD;
 *   - 主循环 Handle_One_Run(): 处理启动/停车请求 + 把结果下发给电机。
 *   main.c 里只留 Handle_One_Init() 和 Handle_One_Run() 两处调用,
 *   不出现任何 PD 细节。
 *
 * 【为什么中断里只"算"不"发"】
 *   电机命令 fish_Step_All_Control() 一次要 8ms 以上(里面有 4 个 HAL_Delay),
 *   在中断里跑会拖死整个系统, 所以中断只把结果存进 g_yaw_output,
 *   再由主循环下发。
 *
 * 【中断里读 I2C 的前提条件】
 *   中断里的姿态读取是阻塞式的, 靠 HAL_GetTick() 判超时, 而 HAL_GetTick()
 *   只在 SysTick 中断里累加 —— 所以必须让 SysTick 优先级高于 TIM3(5)。
 *   这件事已经由 Handle_One_Init() 内部做完
 *   (HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0)), 调用方不用管。
 *
 * 【★ 在 Keil 里必须做的两件事】
 *   1. 把 Function\Handle_One.c 加进工程分组(否则中断回调不参与编译);
 *   2. Options for Target -> C/C++ -> Include Paths 加上 ..\Function。
 * ============================================================ */

#include "main.h"

/* 【初始化】上电调用一次(放 main 的 USER CODE 2):
 * 复位闭环状态 + 把 SysTick 优先级提到 TIM3 之上 + 启动 TIM3 更新中断。
 * 调用后闭环仍处于"停车"状态, 车不会自己动, 要等启动请求。 */
void Handle_One_Init(void);

/* 【周期任务】放在 while(1) 里反复调用:
 *   1. 处理菜单发来的启动/停车请求(标志变化时才动作);
 *   2. VOFA+ 波形上报(与闭环开关无关, 波形一直有);
 *   3. 把中断算出的 (基准速度, 横移量, 修正量) 下发给电机(内部限频 30ms,
 *      与上次相同则不重发)。
 * ⚠ 不能放在中断里调用(内部含阻塞的电机命令)。 */
void Handle_One_Run(void);

/* ==================== 与外部共享的全局量 ====================
 * 定义都在 Handle_One.c 里, 这里只声明。别的模块按需使用:
 *   Menu.c          : 翻转 g_mode_run_request 请求启动/停车
 *   wireless_uart.c : 上报用 g_yaw_target, 在线整定写 g_yaw_kp / g_yaw_kd
 * 想手动试横移(不改菜单)时直接改 g_drive_side, 闭环会在 ≤30ms 内带上。 */

/* 闭环总开关: 1 = 中断开始运算(上电默认 0, 车不会自己动) */
extern volatile uint8_t g_yaw_loop_enable;

/* 启动/停车请求: 0 = 停车(默认), 1 = 启动。由 Menu.c 翻转, 闭环自己消化 */
extern volatile uint8_t g_mode_run_request;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
extern volatile float   g_yaw_target;

/* 可在线整定的 PD 参数 */
extern volatile float   g_yaw_kp;
extern volatile float   g_yaw_kd;

/* 前进基准速度(RPM): 0 = 原地转向, 非 0 = 边走边修 */
extern volatile int16_t g_drive_base;

/* 横移速度(RPM): > 0 左移, < 0 右移, 0 = 不横移(默认); 不参与 PD */
extern volatile int16_t g_drive_side;

#endif /* __HANDLE_ONE_H */

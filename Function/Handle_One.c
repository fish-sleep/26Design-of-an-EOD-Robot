/* ============================================================
 * Handle_One —— 航向角 PD 闭环(取姿态数据 + PD 运算 + 电机下发)
 *
 * 【数据流】
 *   TIM3 中断(20ms): JY901S_ReadOnce() 取最新姿态 -> PD 运算 -> g_yaw_output
 *   主循环 Handle_One_Run(): 处理启动/停车请求 -> 把 (基准速度, 横移量,
 *                            修正量) 下发给电机
 *
 * 【三种走法(由 g_drive_base / g_drive_side 组合)】
 *   base = 0, side = 0 : 原地转向(输出有下限, 起步就转得动)
 *   base ≠ 0           : 边走边修(差速修正, 输出无下限)
 *   side ≠ 0           : 叠加横移(开环, 不参与 PD)
 * ============================================================ */

#include "main.h"
#include "tim.h"
#include "Handle_One.h"
#include "Car_control.h"
#include "jy901s.h"
#include "wireless_uart.h"

/* ==================== PD 参数 ====================
 * 宏集中在这里, 紧挨下面的 PD 算法, 方便对照。
 * ★ 宏必须定义在"用到处"之前, 不要挪到文件末尾。 */

/* PD 参数默认值。整定顺序: 先加 Kp 到"够快但微晃", 再加 Kd 压晃;
 * Kd 直接用陀螺仪角速度, 不用自己差分。
 * 运行时可用 VOFA+ 经 USART2 发 "kp,kd\n" 在线修改。 */
#define YAW_PID_KP_DEFAULT      (4.0f)      /* 比例: 航向误差 1 度给多少转速 */
#define YAW_PID_KD_DEFAULT      (0.35f)     /* 微分: 阻尼项, 压住过冲 */

/* 原地转向的输出限幅(RPM): 下限保证步进电机转得动, 上限防冲过头 */
#define YAW_PID_TURN_OUT_MIN    (20.0f)
#define YAW_PID_TURN_OUT_MAX    (80.0f)

/* 边走边修的差速修正限幅(RPM), 刻意不设下限, 走起来才平顺 */
#define YAW_PID_DRIVE_OUT_MAX   (60.0f)

/* 到位判定: 航向误差小于此角度即认为到位, 输出归零 */
#define YAW_PID_TOLERANCE       (1.0f)

/* 姿态数据连续读取失败的次数上限: 达到后把修正量清零, 交主循环停车,
 * 避免拿旧角度一直修、越修越偏 */
#define YAW_IMU_FAIL_LIMIT      (5u)

/* 电机命令下发间隔(ms): fish_Step_All_Control() 一次要 8ms 以上, 必须限频 */
#define YAW_CMD_INTERVAL_MS     (30u)

/* 菜单按 PE0 启动闭环时的目标航向(度, 相对 JY901S_ZeroYaw() 清零点)。
 * 取 90° 而非 0°: 0° 时误差为 0, 在容差内输出直接归零, 像"按了没反应" */
#define MODE_START_YAW_TARGET   (90.0f)

/* 启动后的前进基准速度(RPM):
 *   0    = 原地保持航向(最安全, 先验证航向保持)
 *   非 0 = 边走边修(★ 不会自动停, 除非按 PE0) */
#define MODE_START_DRIVE_BASE   (0)

/* ==================== 运行状态 ==================== */

/* 闭环总开关: 1 = 中断开始运算(上电默认 0, 车不会自己动) */
volatile uint8_t g_yaw_loop_enable = 0;

/* 启动/停车请求(Menu 与闭环之间通信): Menu 按 PE0 时翻转,
 * 主循环看到变化后执行启动/停车。
 * 0 = 请求停车(默认), 1 = 请求启动 */
volatile uint8_t g_mode_run_request = 0;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的 PD 参数(初值 = 上面两个默认值宏) */
volatile float   g_yaw_kp = YAW_PID_KP_DEFAULT;
volatile float   g_yaw_kd = YAW_PID_KD_DEFAULT;

/* 前进基准速度(RPM): 0 = 原地转向, 非 0 = 边走边修 */
volatile int16_t g_drive_base = 0;

/* 横移速度(RPM): > 0 整体左移, < 0 整体右移, 0 = 不横移(默认)。
 * ★ 不参与 PD, 中断里的闭环只负责车头指向(omega); 需要横移时由外部写它,
 *   闭环会在下一次下发时(≤30ms)自动带上 —— 三个量是叠加的。 */
volatile int16_t g_drive_side = 0;

/* 转向修正量(RPM): > 0 往左修, < 0 往右修, 绝对值 = 转速。
 * 中断算完写这里, 主循环读完下发; 只在本文件用, 所以不外放。 */
static volatile float g_yaw_output = 0.0f;

/* 上次下发的 (基准速度, 横移速度, 修正量), 主循环专用 */
static int16_t s_last_base = 0;
static int16_t s_last_side = 0;
static int16_t s_last_turn = 0;

/* 姿态数据连续读取失败次数(中断里累加, 成功即清零) */
static uint8_t s_yaw_read_fail = 0;

/* ---- 内部函数(只在本文件用) ---- */
static void heading_update_run_state(void);   /* 执行启动/停车请求 */
static void heading_do_start(void);
static void heading_do_stop(void);

/* ==================== 对外接口 ==================== */

void Handle_One_Init(void)
{
    /* ★ SysTick 优先级必须高于 TIM3(5):
     * TIM3 中断里要读 I2C, 而 HAL 的阻塞式 I2C 靠 HAL_GetTick() 判超时;
     * HAL_GetTick() 又只在 SysTick 中断里累加。CubeMX 默认把 SysTick 放在
     * 最低优先级(hal_conf.h 里 TICK_INT_PRIORITY = 15), 这样 SysTick 进不来,
     * 一旦 I2C 读失败就会在 TIM3 中断里死等, 整个程序卡死。
     * 放在 USER CODE 外面也没关系 —— 本文件是人写的, 不会被 CubeMX 覆盖。 */
    HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

    /* 复位闭环状态: 上电默认停车, 参数回默认值, 车不会自己动 */
    g_yaw_loop_enable  = 0;
    g_mode_run_request = 0;
    g_yaw_target       = 0.0f;
    g_yaw_kp           = YAW_PID_KP_DEFAULT;
    g_yaw_kd           = YAW_PID_KD_DEFAULT;
    g_drive_base       = 0;
    g_drive_side       = 0;
    g_yaw_output       = 0.0f;
    s_last_base        = 0;
    s_last_side        = 0;
    s_last_turn        = 0;
    s_yaw_read_fail    = 0;

    /* 启动 TIM3 更新中断(20ms 一次)。CubeMX 不会自动生成这一句,
     * 不加的话中断永远不会进来, 姿态数据也不会刷新。 */
    HAL_TIM_Base_Start_IT(&htim3);
}

/**
  * @brief  主循环周期任务: 下发闭环输出(反复调用)
  * @note   1. 先处理菜单发来的启动/停车请求(标志变化时才动作);
  *         2. VOFA+ 波形上报(与闭环开关解耦, 波形一直有);
  *         3. 限频 30ms, 把 (基准速度, 横移速度, 修正量) 下发给电机;
  *            三个量都和上次一样就不重发(省掉一次 8ms 以上的电机命令);
  *            三个量全 0 -> 停车。
  *         ⚠ 内部含阻塞的 Car_Drive / Car_Stop, 只能在主循环调用。
  */
void Handle_One_Run(void)
{
    static uint32_t last_cmd_tick = 0;
    uint32_t now;
    int16_t  base;
    int16_t  side;
    int16_t  turn;

    /* 1. 先处理菜单发来的启动/停车请求 */
    heading_update_run_state();

    /* 2. VOFA+ 波形上报(自带限频) */
    Wireless_ReportYaw();

    /* 3. 限频 */
    now = HAL_GetTick();
    if ((now - last_cmd_tick) < YAW_CMD_INTERVAL_MS)
    {
        return;
    }
    last_cmd_tick = now;

    base = g_drive_base;
    side = g_drive_side;
    turn = (int16_t)g_yaw_output;   /* 取整, 顺带滤掉不足 1 转的抖动 */

    /* 与上次相同则不重发 */
    if ((base == s_last_base) && (side == s_last_side) && (turn == s_last_turn))
    {
        return;
    }
    s_last_base = base;
    s_last_side = side;
    s_last_turn = turn;

    if ((base == 0) && (side == 0) && (turn == 0))
    {
        /* 已到位 -> 停车 */
        Car_Stop();
    }
    else
    {
        Car_Drive(base, side, turn);
    }
}

/* ==================== 内部实现 ==================== */

/**
  * @brief  启动航向闭环(只能在主循环里调用)
  * @note   先把参数备好再开中断, 避免中断读到半套状态;
  *         末尾先发一次初始速度, 让车轮/电机立刻按基准速度动起来。
  */
static void heading_do_start(void)
{
    g_yaw_loop_enable = 0;                     /* 先关中断再改参数 */
    g_yaw_target      = MODE_START_YAW_TARGET;
    g_drive_base      = MODE_START_DRIVE_BASE;
    g_yaw_output      = 0.0f;
    s_last_base       = 0;
    s_last_side       = 0;
    s_last_turn       = 0;
    g_yaw_loop_enable = 1;                     /* 开闸, 中断开始运算 */

    Car_Drive(g_drive_base, g_drive_side, 0);
}

/**
  * @brief  停止航向闭环并停车(只能在主循环里调用)
  * @note   横移量 g_drive_side 也一起清零: 否则停车后主循环看到它非 0,
  *         又会把车"横着开走", 那就不是停车了。
  */
static void heading_do_stop(void)
{
    g_yaw_loop_enable = 0;
    g_drive_base      = 0;
    g_drive_side      = 0;
    g_yaw_output      = 0.0f;
    s_last_base       = 0;
    s_last_side       = 0;
    s_last_turn       = 0;

    Car_Stop();
}

/**
  * @brief  处理菜单发来的启动/停车请求(主循环里反复调用)
  * @note   只在 g_mode_run_request 变化时动作, 状态记在内部,
  *         所以反复调用不会重复启动/停车。
  */
static void heading_update_run_state(void)
{
    static uint8_t s_applied = 0;   /* 已执行过的请求值(上电默认停车) */

    if (g_mode_run_request == s_applied)
    {
        return;                     /* 没有新请求 */
    }

    s_applied = g_mode_run_request;

    if (s_applied)
    {
        heading_do_start();
    }
    else
    {
        heading_do_stop();
    }
}

/**
  * @brief  定时器中断回调: 取姿态数据 + 航向角 PD 运算(TIM3, 20ms 一次)
  * @note   每次进来先读一帧 JY901S(I2C 阻塞读, 24 字节约 2~3ms), 拿到最新
  *         航向角后紧接着做 PD, 保证"每次算的都是刚读到的新数据", 不会再出现
  *         旧角度被重复运算、响应滞后的情况。
  *         err = 目标航向 - 当前航向; u = Kp*err - Kd*gz(gz = Z 轴角速度);
  *         限幅后存入 g_yaw_output, 由 Handle_One_Run() 下发。
  *         ★ 中断里的 I2C 是阻塞式读取, 内部靠 HAL_GetTick() 判超时, 所以
  *         SysTick 优先级必须高于 TIM3(见 Handle_One_Init()), 否则读失败时
  *         会死等卡死。
  *         ★ 仍然禁止调用 HAL_Delay() 及任何内部含它的函数(Car_Drive /
  *         Car_Stop ...): 那些是给主循环用的, 在中断里跑会白白拖长中断时间。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float err;
    float u;
    float out_max;
    float out_min;

    /* 只处理 TIM3(该回调是所有定时器共用的) */
    if (htim->Instance != TIM3)
    {
        return;
    }

    /* 1. 取最新姿态数据(阻塞 I2C, 约 2~3ms) */
    if (JY901S_ReadOnce() != 0)
    {
        /* 读失败: jy901s_data 里还是上一次的旧值, 拿旧角度继续修只会越修越偏。
         * 所以连续失败够多次就把修正量清零 -> 主循环下发 (base, side, turn)
         * 全 0, 也就是停车(横移量默认 0), 车不会带着错误的角度乱跑。 */
        if (s_yaw_read_fail < 0xFFu)
        {
            s_yaw_read_fail++;
        }
        if (s_yaw_read_fail >= YAW_IMU_FAIL_LIMIT)
        {
            g_yaw_output = 0.0f;
        }
        return;
    }
    s_yaw_read_fail = 0;

    /* 2. 闭环未启动: 数据已经刷新(菜单/波形照常显示最新值), 不再往下算 */
    if (!g_yaw_loop_enable)
    {
        return;
    }

    /* 3. 航向误差(用连续角, 过 ±180° 不跳变) */
    err = g_yaw_target - jy901s_data.yaw_continuous;

    /* 4. 到位判定: 容差内输出归零, 否则会因输出下限在目标点附近来回抖 */
    if ((err <= YAW_PID_TOLERANCE) && (err >= -YAW_PID_TOLERANCE))
    {
        g_yaw_output = 0.0f;
    }
    else
    {
        /* 5. PD: 比例项纠偏; 微分项用 gz 做阻尼, 不必自己差分也不放大噪声 */
        u = (g_yaw_kp * err) - (g_yaw_kd * jy901s_data.gz);

        /* 6. 限幅: 原地转向必须有下限(太慢步进电机转不动), 边走边修不限 */
        if (g_drive_base == 0)
        {
            out_max = YAW_PID_TURN_OUT_MAX;
            out_min = YAW_PID_TURN_OUT_MIN;
        }
        else
        {
            out_max = YAW_PID_DRIVE_OUT_MAX;
            out_min = 0.0f;
        }

        /* 按误差方向分段限幅, 保证输出始终指向目标那一侧 */
        if (err > 0.0f)
        {
            if (u > out_max)
            {
                u = out_max;
            }
            else if (u < out_min)
            {
                u = out_min;
            }
        }
        else
        {
            if (u < -out_max)
            {
                u = -out_max;
            }
            else if (u > -out_min)
            {
                u = -out_min;
            }
        }

        g_yaw_output = u;
    }

    /* 结果已存进 g_yaw_output, 由 Handle_One_Run() 下发 */
}

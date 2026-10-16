/* ============================================================
 * 舵机驱动层 (50Hz PWM) —— 机械臂 5 路
 *
 * 【ID 与引脚】按引脚顺序编号
 *     ID1 -> PD12   TIM4_CH1
 *     ID2 -> PD13   TIM4_CH2
 *     ID3 -> PD14   TIM4_CH3
 *     ID4 -> PD15   TIM4_CH4
 *     ID5 -> PE5    TIM9_CH1
 *
 * 【为什么 CCR 可以直接写脉宽】
 *   两个定时器都配成了 1us 分辨率 + 20ms 周期:
 *       TIM4 (APB1, 84MHz) : PSC = 83  -> 1MHz
 *       TIM9 (APB2, 168MHz): PSC = 167 -> 1MHz
 *       ARR 都是 19999      -> 20000us = 20ms = 50Hz
 *   所以 1 个计数 = 1us, CCR 的值直接就是脉宽的微秒数。
 *
 * 【上电顺序】
 *   先给每个通道写好比较值, 再 HAL_TIM_PWM_Start()。
 *   如果反过来的话, 上电瞬间会先按 CubeMX 里的初值输出
 *   (TIM9_CH1 和 TIM4_CH4 的初值是 0, 也就是 0% 占空比、完全没有脉冲),
 *   舵机会先松垮一下再到位。
 *
 * 【上电姿态】
 *   Servo_Init() 不摆固定中位, 也不做角度换算, 而是把下面那张
 *   s_servo_power_on_pulse[] 表里的脉宽(us)直接写进 CCR(限幅后) ——
 *   想改上电第一帧的姿态, 只改那张脉宽表即可。
 * ============================================================ */

#include "main.h"
#include "tim.h"
#include "servo.h"

/* 中位角度(度) */
#define SERVO_ANGLE_MID     90.0f

/* ---- 硬件映射表: 舵机 ID -> 定时器 + 通道 ---- */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} Servo_Hw_t;

static const Servo_Hw_t s_servo_hw[SERVO_NUM] =
{
    { &htim4, TIM_CHANNEL_1 },      /* ID1 -> PD12 */
    { &htim4, TIM_CHANNEL_2 },      /* ID2 -> PD13 */
    { &htim4, TIM_CHANNEL_3 },      /* ID3 -> PD14 */
    { &htim4, TIM_CHANNEL_4 },      /* ID4 -> PD15 */
    { &htim9, TIM_CHANNEL_1 }       /* ID5 -> PE5  */
};

/* ---- 标定表: 每路舵机的行程范围和安装方向 ----
 * pulse_min : 0 度对应的脉宽(us)
 * pulse_max : 180 度对应的脉宽(us)
 * dir       : +1 = 正常, -1 = 装反了(写 0 度实际转到 180 度)
 *
 * 通用舵机是 500~2500us。如果装配后发现某一路行程对不上
 * (比如写 0 度但舵机顶住了机械限位), 就单独调这一路的值。
 * 限幅的作用是保护舵机 —— 脉宽超出行程会让它一直堵转, 很容易烧。 */
typedef struct
{
    uint16_t pulse_min;
    uint16_t pulse_max;
    int8_t   dir;
} Servo_Cal_t;

static const Servo_Cal_t s_servo_cal[SERVO_NUM] =
{
    { 500, 2500,  1 },      /* ID1 -> PD12 */
    { 500, 2500,  1 },      /* ID2 -> PD13 */
    { 500, 2500,  1 },      /* ID3 -> PD14 */
    { 500, 2500,  1 },      /* ID4 -> PD15 */
    { 500, 2500,  1 }       /* ID5 -> PE5  */
};

/* ---- 运行状态(供读取和缓动用) ---- */
static uint16_t s_servo_pulse[SERVO_NUM];   /* 当前脉宽(us) */
static float    s_servo_angle[SERVO_NUM];   /* 当前角度(度) */

/**
  * @brief  把脉宽限幅到该路舵机的合法范围
  * @param  id        舵机 ID
  * @param  pulse_us  原始脉宽
  * @retval 限幅后的脉宽
  */
static uint16_t servo_limit_pulse(uint8_t id, uint16_t pulse_us)
{
    if (pulse_us < s_servo_cal[id].pulse_min)
    {
        return s_servo_cal[id].pulse_min;
    }
    if (pulse_us > s_servo_cal[id].pulse_max)
    {
        return s_servo_cal[id].pulse_max;
    }
    return pulse_us;
}

/**
  * @brief  角度 -> 脉宽
  * @param  id         舵机 ID
  * @param  angle_deg  角度(0~180, 调用前已限幅)
  * @retval 该角度对应的脉宽(微秒)
  * @note   线性映射: pulse = min + (角度/180) * (max - min)
  *         如果这一路装反了(dir = -1), 先把角度翻成 180-角度 再映射。
  */
static uint16_t servo_angle_to_pulse(uint8_t id, float angle_deg)
{
    float a    = angle_deg;
    float span = (float)(s_servo_cal[id].pulse_max - s_servo_cal[id].pulse_min);

    if (s_servo_cal[id].dir < 0)
    {
        a = 180.0f - a;
    }

    return (uint16_t)((float)s_servo_cal[id].pulse_min
                      + (a / 180.0f) * span + 0.5f);   /* +0.5 四舍五入 */
}

/**
  * @brief  脉宽 -> 角度
  * @param  id        舵机 ID
  * @param  pulse_us  脉宽(微秒, 调用前已限幅)
  * @retval 该脉宽对应的角度(0~180)
  */
static float servo_pulse_to_angle(uint8_t id, uint16_t pulse_us)
{
    float span = (float)(s_servo_cal[id].pulse_max - s_servo_cal[id].pulse_min);
    float a    = ((float)pulse_us - (float)s_servo_cal[id].pulse_min) / span * 180.0f;

    if (s_servo_cal[id].dir < 0)
    {
        a = 180.0f - a;
    }

    return a;
}

/**
  * @brief  内部: 把脉宽真正写到硬件(同时更新缓存)
  * @param  id        舵机 ID
  * @param  pulse_us  已经限幅过的脉宽
  */
static void servo_write(uint8_t id, uint16_t pulse_us)
{
    s_servo_pulse[id] = pulse_us;

    __HAL_TIM_SET_COMPARE(s_servo_hw[id].htim,
                          s_servo_hw[id].channel,
                          (uint32_t)pulse_us);
}

/* ---- 上电初始脉宽表(单位: 微秒 us) ----
 * 每次上电/复位后, Servo_Init() 把这张表里的脉宽**直接**写进定时器比较寄存器
 * (CCR) —— 不经过任何角度换算, 给多少就是多少: 屏幕上/示波器上量到的高电平
 * 时间就等于这里的值。想改上电第一帧的机械臂姿态, 只改这张表。
 *
 * 数组下标就是舵机 ID (SERVO_ID1 ~ SERVO_ID5):
 *     [0] ID1 -> PD12  夹爪
 *     [1] ID2 -> PD13  腕旋转
 *     [2] ID3 -> PD14  折叠①(小臂)
 *     [3] ID4 -> PD15  折叠②(大臂)
 *     [4] ID5 -> PE5   底座回转
 *
 * 常用值(500~2500us 的 180° 舵机, 50Hz 时):
 *      500us =   0° =  2.5% 占空比
 *     1500us =  90° =  7.5% 占空比   <- 中位, 也是复位后的默认值
 *     2500us = 180° = 12.5% 占空比
 *
 * 说明:
 *   - 超出该路 s_servo_cal[] 的 pulse_min/pulse_max 会被限幅(保护舵机),
 *     除此之外不做任何转换;
 *   - 这里给的是"装到机械臂上以后"的脉宽: 该路 dir = -1 也不会被翻转,
 *     表里写多少就输出多少(要多走一点就自己把数加大);
 *   - 需要按角度来控制时用 Servo_SetAngle()/Servo_SetPulseUs(), 两套并存,
 *     本表只负责上电那一下。 */
static const uint16_t s_servo_power_on_pulse[SERVO_NUM] =
{
    2500u,      /* ID1 -> PD12  夹爪     (180° / 12.5%) */
    2500u,      /* ID2 -> PD13  腕旋转   (180° / 12.5%) */
    1500u,      /* ID3 -> PD14  折叠①    ( 90° /  7.5%) */
    1500u,      /* ID4 -> PD15  折叠②    ( 90° /  7.5%) */
    1500u       /* ID5 -> PE5   底座回转 ( 90° /  7.5%) */
};

/**
  * @brief  初始化舵机
  * @note   上电调用一次。
  *         把 s_servo_power_on_pulse[] 里的脉宽限幅后直接写进比较寄存器(CCR),
  *         再启动 PWM —— 不经过角度换算, 给多少 us 就输出多少 us。
  *         ★ 顺序不能反: 先写 CCR 再启动, 否则会先输出一帧 CubeMX 里的初值
  *           (0% 占空比), 舵机会先松垮一下再到位。
  *         写入后顺带更新角度缓存, 保证 Servo_GetAngle() 读到的值和当前
  *         脉宽一致(只是显示/读取用, 不参与这次的输出)。
  */
void Servo_Init(void)
{
    uint8_t  i;
    uint16_t pulse;

    for (i = 0; i < SERVO_NUM; i++)
    {
        /* 1. 脉宽限幅后直接写进 CCR(servo_write 内部就是 __HAL_TIM_SET_COMPARE) */
        pulse = servo_limit_pulse(i, s_servo_power_on_pulse[i]);
        servo_write(i, pulse);

        /* 角度缓存跟着对一下, 免得 Servo_GetAngle() 读到和脉宽对不上的旧值 */
        s_servo_angle[i] = servo_pulse_to_angle(i, pulse);

        /* 2. 再启动 PWM 输出 —— 顺序不能反, 否则会先输出一帧 CubeMX 初值 */
        (void)HAL_TIM_PWM_Start(s_servo_hw[i].htim, s_servo_hw[i].channel);
    }
}

/**
  * @brief  只启动某一路舵机的 PWM 输出(其余通道保持不输出)
  * @param  id  舵机 ID (SERVO_ID1 ~ SERVO_ID5)
  * @note   和 Servo_Init() 的区别: Servo_Init() 一次把 5 路全拉起来并按上电
  *         脉宽表输出; 本函数只拉一路, 专门给"单路调试/标定"用 —— 只有被拉起来
  *         的这一路会收到脉冲并保持住, 其它几路仍然是松的, 不会跟着乱动,
  *         台架上最安全, 观察到的动作也只属于这一路。
  *
  *         如果这一路从来没设过脉宽, 这里默认给中位(1500us), 免得一上电
  *         就冲到行程端点。想控制上电第一帧的姿态, 就先用 Servo_SetPulseUs()
  *         把脉宽写好再调本函数。
  */
void Servo_Enable(uint8_t id)
{
    if (id >= SERVO_NUM)
    {
        return;
    }

    if (s_servo_pulse[id] == 0u)    /* 0 不可能是合法脉宽, 说明从没设过 */
    {
        s_servo_pulse[id] = servo_limit_pulse(id,
                              servo_angle_to_pulse(id, SERVO_ANGLE_MID));
        s_servo_angle[id] = SERVO_ANGLE_MID;
    }

    /* 先写 CCR 再启动输出, 顺序和 Servo_Init() 一致
     * (TIM9_CH1 的 CubeMX 初值是 0, 反过来的话会先输出一帧无脉冲) */
    __HAL_TIM_SET_COMPARE(s_servo_hw[id].htim,
                          s_servo_hw[id].channel,
                          (uint32_t)s_servo_pulse[id]);

    (void)HAL_TIM_PWM_Start(s_servo_hw[id].htim, s_servo_hw[id].channel);
}

/**
  * @brief  按角度控制舵机
  * @param  id        舵机 ID (SERVO_ID1 ~ SERVO_ID5)
  * @param  angle_deg 目标角度, 0~180 度, 超出自动限幅
  */
void Servo_SetAngle(uint8_t id, float angle_deg)
{
    uint16_t pulse;

    if (id >= SERVO_NUM)
    {
        return;
    }

    /* 角度限幅 */
    if (angle_deg < 0.0f)
    {
        angle_deg = 0.0f;
    }
    else if (angle_deg > 180.0f)
    {
        angle_deg = 180.0f;
    }

    pulse = servo_angle_to_pulse(id, angle_deg);
    servo_write(id, pulse);

    s_servo_angle[id] = angle_deg;      /* 存原始角度, 避免来回换算累积误差 */
}

/**
  * @brief  读取某个舵机当前的角度
  * @param  id  舵机 ID
  * @retval 上一次设定的角度(0~180); ID 非法时返回 0
  */
float Servo_GetAngle(uint8_t id)
{
    if (id >= SERVO_NUM)
    {
        return 0.0f;
    }

    return s_servo_angle[id];
}

/**
  * @brief  读取某一路舵机的硬件比较寄存器(CCR)当前值
  * @param  id  舵机 ID
  * @retval CCR 寄存器里的实际值(= 脉宽微秒数); ID 非法时返回 0
  * @note   和 Servo_GetPulseUs() 的区别: 后者读的是软件缓存, 本函数直接读
  *         定时器寄存器。两者一致才说明"软件写的值确实进了硬件" ——
  *         排查"脉宽在变但舵机不动"时很有用。
  */
uint32_t Servo_GetCcr(uint8_t id)
{
    if (id >= SERVO_NUM)
    {
        return 0u;
    }

    return __HAL_TIM_GET_COMPARE(s_servo_hw[id].htim, s_servo_hw[id].channel);
}

/**
  * @brief  按脉宽控制舵机
  * @param  id        舵机 ID
  * @param  pulse_us  脉宽, 单位微秒
  * @note   标定行程时用这个更直接: 手动给 500 / 1500 / 2500,
  *         看舵机实际转到哪里, 再把 s_servo_cal[] 改准。
  */
void Servo_SetPulseUs(uint8_t id, uint16_t pulse_us)
{
    if (id >= SERVO_NUM)
    {
        return;
    }

    pulse_us = servo_limit_pulse(id, pulse_us);
    servo_write(id, pulse_us);

    s_servo_angle[id] = servo_pulse_to_angle(id, pulse_us);
}

/**
  * @brief  读取某个舵机当前的脉宽
  * @param  id  舵机 ID
  * @retval 当前脉宽(微秒); ID 非法时返回 0
  */
uint16_t Servo_GetPulseUs(uint8_t id)
{
    if (id >= SERVO_NUM)
    {
        return 0;
    }

    return s_servo_pulse[id];
}

/* ============================================================
 * Handle_Second —— ① 机械臂舵机调试 ② USART3 接收(占位, 暂未启用)
 *
 * 【本文件有两块内容】
 *   ① 机械臂舵机调试 —— 见文件末尾两个调试层, 由 Handle_Second.h 里的
 *      ARM_TEST_MODE 选择:
 *        Arm_Test_BaseSweep()  上电只让最下面那一路舵机(底座回转)来回扫描,
 *                              用来肉眼确定"零度"对应的机械姿态;
 *        Arm_OutputCheck()      不动舵机, 只把 5 路的 mem/ccr/引脚实测波形
 *                              显示出来, 用来对账"示波器读数 vs 代码"。
 *   ② USART3 字节接收 —— 占位代码, 暂未启用, 说明见下。
 *
 * ------------------------------------------------------------
 * 【② USART3 硬件】USART3: PB10 = TX, PB11 = RX, 115200 8N1
 *         (MX_USART3_UART_Init() 已经在 main.c 里调用; 目前没有配 DMA)
 *
 * 【接通后的数据流】
 *   外部设备 -> USART3_RX -> 每收到 1 字节进一次
 *   HAL_UART_RxCpltCallback() -> 存缓冲/组帧 -> 置标志 -> 主循环消费
 *
 * 【★ 现在这个回调不会被执行 —— 要用时必须补齐下面三步 TODO】
 *   1. 使能 NVIC 并补中断函数:
 *      - CubeMX 里勾上 "USART3 global interrupt" 最省事;
 *      - 手动做就是在 usart.c 的 HAL_UART_MspInit() 里加
 *          HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
 *          HAL_NVIC_EnableIRQ(USART3_IRQn);
 *      - 注意 stm32f4xx_it.c 里目前只生成了 USART1/USART2 的中断函数,
 *        没有 USART3_IRQHandler。光开 NVIC 而不补这个函数的话,
 *        中断会掉进 startup 文件里的 Default_Handler(死循环)。
 *   2. 在 main.c 的 USER CODE 2 里调一次 Handle_Second_Init() 挂起接收;
 *   3. 把 Function\Handle_Second.c 加进 Keil 的 Function 分组参与编译。
 *
 * 【另一种更省事的收法】
 *   如果以后要收不定长帧, 可以像无线串口(wireless_uart.c)那样用
 *   HAL_UARTEx_ReceiveToIdle_IT/DMA(), 走 HAL_UARTEx_RxEventCallback()
 *   回调, 不用一个字节一个字节地收。
 * ============================================================ */

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "servo.h"
#include "OLED.h"
#include "Handle_Second.h"

/* 单字节接收缓冲: HAL_UART_Receive_IT() 每次搬 1 个字节到这里 */
static uint8_t s_rx_byte = 0;

/* 已收到的字节数(调试用: 也能证明中断确实进来了) */
static volatile uint32_t s_rx_count = 0;

void Handle_Second_Init(void)
{
    /* 挂起第一次接收: 之后每收到 1 个字节进一次 HAL_UART_RxCpltCallback(),
     * 回调里再挂起下一次, 这样才能连续接收。
     * ⚠ 前提是 USART3 的 NVIC 中断已经使能并且补了 USART3_IRQHandler
     *   (见文件头 TODO 第 1 条); 否则这里虽然返回 HAL_OK, 但中断永远不会来。 */
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

uint32_t Handle_Second_GetRxCount(void)
{
    return s_rx_count;
}

/**
  * @brief  串口接收完成回调(HAL 弱函数, 这里是重定义)
  * @param  huart 触发回调的串口句柄
  * @note   ★ 这是"所有串口共用"的回调, 所以必须先判断是不是 USART3,
  *         否则会抢走 USART1(步进电机)/USART2(无线)的数据。
  *         本工程其它模块覆盖的 HAL_UARTEx_RxEventCallback /
  *         HAL_UART_ErrorCallback 也各自做了同样的判断, 互不干扰。
  *
  * 【TODO: 收到一个字节后要做的事】
  *   1. 把 s_rx_byte 存进接收缓冲, 或直接喂给帧解析状态机;
  *   2. 组好一帧(固定帧头+长度 / 固定长度 / 结束符)后置个标志,
  *      交给主循环去执行动作 —— 中断里只做搬运, 别做耗时的事;
  *   3. 通信出错/丢帧要能恢复: 加错误计数, 在 HAL_UART_ErrorCallback()
  *      里重新挂起接收(可参考 wireless_uart.c 的写法);
  *   4. 单字节中断收法效率很低, 协议定下来后建议换成
  *      HAL_UARTEx_ReceiveToIdle_IT/DMA。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;                     /* 不是串口3, 不归本模块管 */
    }

    s_rx_count++;                   /* 先记个数, 调试时能确认中断进来了 */

    /* TODO(1): 解析/缓存 s_rx_byte */
    (void)s_rx_byte;

    /* TODO(2): 重新挂起下一次接收 —— 不写这句就只能收到一个字节。
     * 帧解析写好后把下面这行注释打开:
     *   (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
     */
}

/* ============================================================
 * 机械臂舵机调试层
 *
 * 【这一步要干什么】
 *   先只让最下面那一路舵机 —— ID5 底座回转(PE5 / TIM9_CH1) —— 上电自动
 *   来回扫描, 眼睛盯着看, 把"零度"对应的机械姿态确定下来:
 *
 *       脉宽  500us = 软件   0°   <- 行程的一端, 这就是要找的"零度姿态"
 *       脉宽 1500us = 中位  90°   <- Servo_Init() 的上电脉宽表里填的值
 *       脉宽 2500us = 软件 180°   <- 行程的另一端
 *
 *   (MG996R 是 180° 舵机, 行程 500~2500us, 1° = 11.11us, 换算见 servo.c)
 *
 * 【要记录的三件事】
 *   ① 脉宽 500us 时这个关节朝哪边 —— 这就是"零度"的实际姿态;
 *   ② 脉宽从 500 涨到 2500 时它往哪边转 —— 对应 servo.c 里这一路的 dir
 *      (现在填的是 +1, 实测转反了就改成 -1);
 *   ③ 500us / 2500us 会不会顶住 —— 听到持续"嗡嗡"、看到抖动或转不动就是
 *      顶住了, 说明真实行程没这么大, 要把 servo.c 的 pulse_min/max 往回收。
 *
 * 【为什么一次只动一路】
 *   其余 4 路保持"不输出", 舵机是松的, 不会跟着乱动、不会互相干涉,
 *   台架上最安全, 也保证你看到的动作只属于这一路。
 *
 * 【怎么重复看】
 *   不需要按键: 想看一遍重新上电或按复位键就行, 流程会自动重跑。
 *   一轮约 12 秒(中位停 1s -> 走到 0° -> 停 -> 走到 180° -> 停 -> 来回)。
 *
 * 【参数在哪改】
 *   下面这几个 ARM_TEST_* 宏; 开关在 Handle_Second.h 的 ARM_TEST_MODE。
 *
 * ⚠ 安全提醒
 *   - 扫描时手虚扶着机械臂, 电源开关放在随手能按到的位置;
 *   - MG996R 堵转电流可达 ~2.5A, 必须独立 5V/6V 供电并与 MCU 共地;
 *   - 听到持续"嗡嗡"而关节不动 = 堵转, 立刻断电。
 * ============================================================ */

/* ---- 扫描参数 ---- */

/* 要测哪一路: 填 SERVO_ID1 ~ SERVO_ID5。
 * 改这一行就够了 —— 屏幕上的关节名/信号脚、CCR 读回、引脚实测占空比
 * 全部自动跟着变, 别的地方都不用动。
 *
 *     SERVO_ID1  PD12  TIM4_CH1  夹爪
 *     SERVO_ID2  PD13  TIM4_CH2  腕旋转(爪子转动)
 *     SERVO_ID3  PD14  TIM4_CH3  折叠①(小臂那节)
 *     SERVO_ID4  PD15  TIM4_CH4  折叠②(大臂那节)
 *     SERVO_ID5  PE5   TIM9_CH1  底座回转(整臂 180°)
 *                                 ★ 已实测: 500us 就是想要的姿态
 *
 * ⚠ 换一路之前: 先把机械臂摆到安全姿态、手扶住; 建议先把下面
 *   ARM_TEST_PULSE_MIN/MAX 收窄成 1000/2000 跑一轮确认不顶, 再放开。
 * ⚠ 夹爪(ID1)要注意: 如果是"合到底"的那一端, 会一直顶住堵转, 别久留。 */
#define ARM_TEST_ID             SERVO_ID5

#define ARM_TEST_PULSE_MID      1500u       /* 中位(软件 90°) */
#define ARM_TEST_PULSE_MIN      500u        /* 扫描下限(软件 0°) */
#define ARM_TEST_PULSE_MAX      2500u       /* 扫描上限(软件 180°) */
#define ARM_TEST_STEP_US        20u         /* 每步 20us, 约 2.2°, 走得慢好看 */
#define ARM_TEST_STEP_MS        20u         /* 每步 20ms, 和舵机 PWM 周期一致 */
#define ARM_TEST_END_DWELL_MS   3000u       /* 到两端各停 3 秒, 够你看清姿态和屏幕读数 */
#define ARM_TEST_DISPLAY_EVERY  8u          /* 每 8 步刷一次屏(每次刷屏还要实测占空比, 有开销) */

/* 各路舵机的名字 + 信号脚(只为显示, 让人一眼看出正在测哪一路) */
static const char *const s_arm_joint_name[SERVO_NUM] =
{
    "ID1 GRIP  PD12",
    "ID2 WRIST PD13",
    "ID3 FOLD1 PD14",
    "ID4 FOLD2 PD15",
    "ID5 BASE  PE5"
};

/* 各路信号脚对应的 GPIO 端口/引脚 —— "引脚实测占空比"要用 */
static GPIO_TypeDef *const s_arm_sig_port[SERVO_NUM] =
{
    GPIOD, GPIOD, GPIOD, GPIOD, GPIOE
};

static const uint16_t s_arm_sig_pin[SERVO_NUM] =
{
    GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_5
};

/**
  * @brief  内部: 采集某一路舵机信号脚的高电平占比(%), 证明 PWM 到底有没有
  *         真的从引脚输出出去
  * @param  id  舵机 ID (SERVO_ID1 ~ SERVO_ID5)
  * @retval 高电平占整个采样窗口的百分比(0~100)
  * @note   原理: 信号脚配成定时器复用输出后, 读它的输入寄存器(如
  *         GPIOE->IDR)拿到的仍然是引脚上的真实电平。用一个紧密循环连续
  *         采样, 统计高电平占的比例 —— 等于拿示波器直接量信号线。
  *
  *         50Hz / 20ms 周期下的理论值:
  *             脉宽  500us ->  2%
  *             脉宽 1500us ->  7%
  *             脉宽 2500us -> 13%
  *         三个值差异明显, 足够判断硬件有没有在输出。
  *
  *         读回 0%   -> 引脚恒低(定时器没跑 / 通道没使能 / 引脚没配成复用);
  *         读回 100% -> 引脚恒高;
  *         读回 2% / 7% / 13% 且随脉宽变化 -> PWM 正常, 问题在舵机/供电/机械。
  */
static uint8_t arm_pin_duty(uint8_t id)
{
    uint32_t i;
    uint32_t hi   = 0u;
    /* 采样窗口取足够大, 保证即使循环跑得最快也远超一个 20ms 的 PWM 周期,
     * 否则窗口可能整个落在脉冲之外, 读出假的 0%。 */
    uint32_t all  = 2000000u;
    GPIO_TypeDef *port = s_arm_sig_port[id];
    uint16_t      pin  = s_arm_sig_pin[id];

    /* 采样期间只屏蔽 TIM3 中断: 它每 20ms 进一次(里面要读 I2C, 耗时长且
     * 不固定), 和 20ms 的 PWM 周期同频, 不屏蔽就会在固定相位上漏采样,
     * 把占空比读歪。这里不用 __disable_irq() 关全局中断, 好让 SysTick
     * 继续走 —— 否则每次测量都会让 HAL_GetTick() 累积滞后。 */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

    for (i = 0u; i < all; i++)
    {
        if ((port->IDR & pin) != 0u)
        {
            hi++;
        }
    }

    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    return (uint8_t)((hi * 100u) / all);
}

/**
  * @brief  内部: 把当前脉宽/角度/硬件寄存器/引脚实测值画到 OLED
  * @param  pulse_us   当前脉宽(微秒)
  * @param  angle_deg  当前角度(度)
  * @param  state      第 6 行的状态提示(只能 ASCII, 见 OLED.h)
  * @note   每次调用都会实测一次引脚占空比, 所以屏幕上的 Pin 永远是刚测的
  *         真值, 不会是"--"也不是旧值。
  */
static void arm_test_show(uint16_t pulse_us, float angle_deg, const char *state)
{
    uint8_t duty = arm_pin_duty(ARM_TEST_ID);   /* 先测, 再画 */

    OLED_ClearBuffer();     /* 只清显存, 不刷屏; 下面统一 Refresh() 一次 */

    OLED_ShowString(0, 0, "ARM SELFTEST");
    OLED_ShowString(0, 1, s_arm_joint_name[ARM_TEST_ID]);

    OLED_ShowString(0, 2, "Pulse:");
    OLED_ShowNum(42, 2, (int32_t)pulse_us, 4);
    OLED_ShowString(66, 2, "us");

    OLED_ShowString(0, 3, "Angle:");
    OLED_ShowFloat(42, 3, angle_deg, 1);

    /* 硬件侧证据: 比较寄存器的实际值 —— 和上面 Pulse 一致才说明软件
     * 确实把脉宽写进了硬件寄存器 */
    OLED_ShowString(0, 4, "CCR:");
    OLED_ShowNum(30, 4, (int32_t)Servo_GetCcr(ARM_TEST_ID), 4);

    /* 引脚实测占空比: 没有这一行就分不清"压根没输出 PWM"和"舵机/供电的问题" */
    OLED_ShowString(0, 5, "Pin:");
    OLED_ShowNum(30, 5, (int32_t)duty, 2);
    OLED_ShowString(48, 5, "%");

    OLED_ShowString(0, 6, state);
    OLED_ShowString(0, 7, "exp 2/7/13%");

    OLED_Refresh();
}

/**
  * @brief  内部: 把这一路舵机从 from_us 慢慢走到 to_us(一步一步, 不是一下冲过去)
  * @param  from_us 起始脉宽(微秒)
  * @param  to_us   结束脉宽(微秒)
  * @note   每步只挪 ARM_TEST_STEP_US, 中间插 ARM_TEST_STEP_MS 延时 ——
  *         这样舵机是匀速慢慢转, 眼睛跟得上, 也不会猛甩。
  */
static void arm_test_ramp(uint16_t from_us, uint16_t to_us)
{
    uint16_t p   = from_us;
    uint8_t  cnt = 0u;

    for (;;)
    {
        Servo_SetPulseUs(ARM_TEST_ID, p);

        if (cnt == 0u)      /* 限频刷屏, 别把扫描拖慢 */
        {
            arm_test_show(p, Servo_GetAngle(ARM_TEST_ID), "Sweeping...");
        }
        cnt = (uint8_t)((cnt + 1u) % ARM_TEST_DISPLAY_EVERY);

        if (p == to_us)
        {
            break;          /* 到目标点, 退出 */
        }

        /* 往目标靠近一步; 用差值判断避免 uint16 下溢/跨过头 */
        if (p < to_us)
        {
            p = (uint16_t)(((to_us - p) > ARM_TEST_STEP_US)
                           ? (p + ARM_TEST_STEP_US) : to_us);
        }
        else
        {
            p = (uint16_t)(((p - to_us) > ARM_TEST_STEP_US)
                           ? (p - ARM_TEST_STEP_US) : to_us);
        }

        HAL_Delay(ARM_TEST_STEP_MS);
    }
}

/**
  * @brief  舵机单路自检: 只让底座那一路(ID5)来回扫描, 用来确定"零度"
  * @note   ★ 内部是死循环, 调用后不会返回, 主循环(菜单/按键/跑车)都不会跑。
  *         ★ 只有 ID5 会得到脉冲, 另外 4 路保持不输出(松的)。
  *         看的时候重点记: 500us 时朝哪边、脉宽变大时往哪边转、
  *         两端会不会顶住 —— 这三条决定 servo.c 里的 dir 和 pulse_min/max。
  */
void Arm_Test_BaseSweep(void)
{
    /* 先把脉宽写进 CCR, 再启动这一路的 PWM: 上电第一帧就是中位,
     * 不会像"先启动后写值"那样先松一下再到位(理由见 servo.c 文件头)。 */
    Servo_SetPulseUs(ARM_TEST_ID, ARM_TEST_PULSE_MID);
    Servo_Enable(ARM_TEST_ID);

    /* 先在中位停 1 秒, 让你看清基准姿态(半展开), 并实测一次引脚占空比 */
    arm_test_show(ARM_TEST_PULSE_MID, Servo_GetAngle(ARM_TEST_ID), "Hold mid 1s");
    HAL_Delay(1000u);

    /* 先往"软件 0°"(500us)那一端走 —— 这就是要找的零度姿态 */
    arm_test_ramp(ARM_TEST_PULSE_MID, ARM_TEST_PULSE_MIN);

    /* 之后就是 500 <-> 2500 三角波来回, 到两端各停一下(顺便实测占空比) */
    for (;;)
    {
        arm_test_show(ARM_TEST_PULSE_MIN, Servo_GetAngle(ARM_TEST_ID), "ZERO pose!");
        HAL_Delay(ARM_TEST_END_DWELL_MS);

        arm_test_ramp(ARM_TEST_PULSE_MIN, ARM_TEST_PULSE_MAX);

        arm_test_show(ARM_TEST_PULSE_MAX, Servo_GetAngle(ARM_TEST_ID), "180deg end");
        HAL_Delay(ARM_TEST_END_DWELL_MS);

        arm_test_ramp(ARM_TEST_PULSE_MAX, ARM_TEST_PULSE_MIN);
    }
}

/* ============================================================
 * 舵机输出对账层 —— 给"示波器量到的占空比/周期和代码算的对不上"用
 *
 * 【为什么要单独做这一层】
 *   外部仪器量 50Hz / 1.5ms 这种窄脉冲时很容易受触发、阈值、探头接地的影响,
 *   而"舵机到底被命令到了哪个角度"只由引脚上真实的高电平时间决定。
 *   这里一次给出三份证据, 三者一对照就知道断在哪一环:
 *       ① 软件侧: 缓存脉宽 mem、比较寄存器 ccr、通道使能位 CCxE
 *       ② 引脚侧: 实测高电平时间 H、周期 T、高电平占比(片上示波器,
 *                 用内核 DWT 周期计数器当基准, 与定时器无关)
 *       ③ 定时器侧: ARR / PSC 以及由实测周期换算出的"1 个计数 = 多少 us"
 *
 *   判读:
 *     - "NO PULSE ON PIN" 或 sam 0%  -> 这一路根本没输出 PWM
 *       (CCxE = NO / Servo_Init() 没跑到 / 引脚没配成复用);
 *     - duty 和 set 对不上  -> "写的"和"输出的"不是一回事(引脚没动/被外部拉死);
 *     - tick 不是 1.00us、T 不是 20000  -> 定时器时钟或预分频和预期不符,
 *       舵机收到的脉宽(us)会跟着变, 占空比自然也变;
 *     - duty 与 set 一致  -> 硬件输出正常, 问题在舵机/供电/机械。
 *
 * 【安全】本层只读不写: 不碰任何舵机寄存器, 机械臂保持 Servo_Init() 摆好的
 *   姿态不动, 所以接在 Servo_Init() 后面看是安全的。
 * ============================================================ */

/* 采样超时(ms): 20ms 的周期留 5 倍余量, 引脚上没有跳变时也能及时退出 */
#define ARM_PULSE_TIMEOUT_MS    100u

/* 每一路的停留时间(ms): 单次占空比采样本身要跑 200 万次循环(约 0.1s) */
#define ARM_CHECK_DWELL_MS      2000u

/* 等电平跳变的"循环次数"保险: 5,000,000 次约 240ms, 详见 arm_wait_level() */
#define ARM_WAIT_GUARD_MAX      5000000u

/**
  * @brief  内部: 启动 DWT 周期计数器(Cortex-M4 内核自带)
  * @note   168MHz 主频下 1 个计数 = 1/168us, 量 1.5ms / 20ms 这种量级绰绰有余。
  *         它和定时器无关, 所以即使定时器时钟配错, 它给出的仍是真实时间 ——
  *         这正是用来判断"定时器 1 个计数是不是真的 1us"的基准。
  */
static void arm_dwt_start(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* 打开跟踪单元 */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                /* 周期计数开始跑 */
}

/**
  * @brief  内部: 等引脚变成指定电平, 返回跳变时刻的 DWT 计数值
  * @param  port         GPIO 端口
  * @param  pin          引脚位
  * @param  want_high    1 = 等高电平, 0 = 等低电平
  * @param  timeout_cyc  超时(内核周期数)
  * @param  t_edge       输出: 电平变成 want_high 时的 DWT->CYCCNT
  * @retval 1 = 等到了; 0 = 超时(引脚没有这个跳变)
  * @note   除 DWT 超时外还加了一道"循环次数"保险: 万一 DWT 周期计数器没跑
  *         (计数值恒为 0, 时间差永远是 0, DWT 超时永远不会触发), 只靠 DWT
  *         判断会在这里死等、把调试界面卡住。等最长的合法情况是要等近
  *         20ms 的下一个上升沿, 约 42 万次循环, 所以 500 万次留了 12 倍余量。
  */
static uint8_t arm_wait_level(GPIO_TypeDef *port, uint16_t pin,
                              uint32_t want_high, uint32_t timeout_cyc,
                              uint32_t *t_edge)
{
    uint32_t t0    = DWT->CYCCNT;
    uint32_t guard = 0u;

    while (((port->IDR & pin) != 0u) != (want_high != 0u))
    {
        if (((DWT->CYCCNT - t0) > timeout_cyc) || (++guard > ARM_WAIT_GUARD_MAX))
        {
            return 0u;
        }
    }

    *t_edge = DWT->CYCCNT;
    return 1u;
}

/**
  * @brief  内部: 直接在信号脚上量"高电平时间"和"周期"(片上示波器)
  * @param  id         舵机 ID
  * @param  high_us    输出: 高电平时间(微秒)
  * @param  period_us  输出: 周期(微秒)
  * @retval 1 = 量到了; 0 = 引脚上没有跳变(恒低 / 恒高 / 无输出)
  * @note   信号脚配成定时器复用输出后, 读它的输入寄存器拿到的仍是引脚真实
  *         电平, 所以不用拆线、不用外接示波器就能知道舵机实际收到什么。
  *         ★ 采样期间屏蔽 TIM3 中断: 它 20ms 进一次(里面要读 I2C, 耗时且不
  *           固定), 和 PWM 周期同频, 不屏蔽会漏采样、把时间量歪。这里不关
  *           全局中断, SysTick 照常走, HAL_GetTick() 不受影响。
  */
static uint8_t arm_pin_pulse(uint8_t id, uint32_t *high_us, uint32_t *period_us)
{
    GPIO_TypeDef *port = s_arm_sig_port[id];
    uint16_t      pin  = s_arm_sig_pin[id];
    uint32_t      cyc_per_us = SystemCoreClock / 1000000u;
    uint32_t      cyc_per_ms = SystemCoreClock / 1000u;
    /* 超时的单位是"毫秒 -> 内核周期数"。★ 别漏掉 1000: 低电平要一直持续到
     * 下个周期的上升沿(50Hz 下最坏近 20ms), 只给 100us 会必然超时,
     * 把"明明有 PWM"误判成 NO PULSE ON PIN。 */
    uint32_t      timeout    = ARM_PULSE_TIMEOUT_MS * cyc_per_ms;
    uint32_t      t0 = 0u, t1 = 0u, t2 = 0u;
    uint8_t       ok;

    HAL_NVIC_DisableIRQ(TIM3_IRQn);

    ok  = arm_wait_level(port, pin, 0u, timeout, &t0);  /* 先落在低电平上 */
    ok &= arm_wait_level(port, pin, 1u, timeout, &t0);  /* 上升沿 = 脉冲起点 */
    ok &= arm_wait_level(port, pin, 0u, timeout, &t1);  /* 下降沿 = 脉冲终点 */
    ok &= arm_wait_level(port, pin, 1u, timeout, &t2);  /* 下一个上升沿 = 周期终点 */

    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    if (ok == 0u)
    {
        return 0u;
    }

    *high_us   = (t1 - t0) / cyc_per_us;    /* 无符号差值, 计数器回绕也正确 */
    *period_us = (t2 - t0) / cyc_per_us;
    return 1u;
}

/**
  * @brief  内部: 取该路通道使能位(CCxE)在 CCER 里的掩码
  * @param  id  舵机 ID
  * @retval CCER 中的掩码
  * @note   ID1~ID4 = TIM4_CH1~CH4 -> CC1E/CC2E/CC3E/CC4E; ID5 = TIM9_CH1 -> CC1E。
  *         CH1 的掩码是 0x0001, 每往后一个通道左移 4 位, 正好 (id & 3) * 4 位
  *         —— ID5(=4) & 3 = 0, 落在 CC1E 上, 两种定时器都算对。
  */
static uint32_t arm_ccer_mask(uint8_t id)
{
    return (uint32_t)(1u << (((uint32_t)id & 3u) * 4u));
}

/**
  * @brief  舵机输出对账: 5 路轮流显示"软件写了什么"和"引脚上实际是什么"
  * @note   ★ 只读不写, 不动任何舵机; 机械臂保持 Servo_Init() 摆好的姿态。
  *         ★ 内部是死循环, 调用后不会返回 —— 放在 main() 初始化段的最后;
  *           看完把 Handle_Second.h 里的 ARM_TEST_MODE 改回 OFF 再重新编译。
  *
  *         屏幕判读(每一路一屏, 每 2 秒换下一路):
  *           mem        软件缓存的脉宽(us)        -> Servo_GetPulseUs()
  *           ccr        比较寄存器里的真值(us)    -> 与 mem 一致才说明写进了硬件
  *           EN / NO    通道使能位(CCxE)开没开    -> NO = 这一路根本没启动
  *           H / T      引脚实测高电平时间 / 周期 -> 舵机真正收到的波形
  *           tick       实测周期 /(ARR+1), 即 1 个计数等于多少 us
  *           sam        200 万次引脚电平采样出的占空比(%), 不依赖 DWT, 可互为佐证
  *           duty       引脚实测占空比(%) = H/T, 两位小数, 直接和示波器读数对比
  *           set        软件设定占空比(%) = ccr/(ARR+1), 两位小数
  *           ch X/5     进度提示
  *
  *         判读:
  *           duty 与 set 一致           -> 硬件输出正常, 问题在舵机/供电/机械;
  *           duty≈0 但 set 正常         -> 引脚根本没动(没配成复用 / 被外部拉死);
  *           NO PULSE ON PIN 且 sam 0%  -> 这一路没有 PWM 输出;
  *           tick 不是 1.00us、T 不是 20000 -> 定时器时钟或预分频与预期不符,
  *           此时舵机收到的脉宽(us)也跟着变, 占空比自然对不上。
  */
void Arm_OutputCheck(void)
{
    uint8_t  id;
    uint8_t  duty;
    uint16_t mem;
    uint32_t ccr, arr, psc, en, sig;
    uint32_t high_us, period_us, tick100;
    float    duty_pin, duty_set;

    arm_dwt_start();

    for (;;)
    {
        for (id = 0u; id < SERVO_NUM; id++)
        {
            TIM_TypeDef *tim = (id == SERVO_ID5) ? htim9.Instance : htim4.Instance;

            mem = Servo_GetPulseUs(id);
            ccr = Servo_GetCcr(id);
            arr = tim->ARR;
            psc = tim->PSC;
            en  = ((tim->CCER & arm_ccer_mask(id)) != 0u) ? 1u : 0u;

            high_us   = 0u;
            period_us = 0u;
            tick100   = 0u;
            duty_pin  = 0.0f;
            duty_set  = 0.0f;

            sig  = (uint32_t)arm_pin_pulse(id, &high_us, &period_us);
            duty = arm_pin_duty(id);

            /* 设定占空比: 比较值 / 周期, 单位 %(定时器本身就是按这个比值输出的) */
            if ((arr + 1u) != 0u)
            {
                duty_set = (float)ccr * 100.0f / (float)(arr + 1u);
            }

            if (sig != 0u)
            {
                /* 1 个定时器计数 = 多少 us(乘 100 保留两位小数) */
                tick100 = (period_us * 100u) / (arr + 1u);

                if (period_us != 0u)
                {
                    /* 实测占空比: 引脚上量到的高电平时间 / 周期 */
                    duty_pin = (float)high_us * 100.0f / (float)period_us;
                }
            }

            OLED_ClearBuffer();
            OLED_ShowString(0, 0, "SERVO OUT CHECK");
            OLED_ShowString(0, 1, s_arm_joint_name[id]);

            OLED_ShowString(0, 2, "mem");
            OLED_ShowNum(18, 2, (int32_t)mem, 4);
            OLED_ShowString(42, 2, "ccr");
            OLED_ShowNum(60, 2, (int32_t)ccr, 4);
            OLED_ShowString(84, 2, (en != 0u) ? "EN" : "NO");

            if (sig != 0u)
            {
                OLED_ShowString(0, 3, "H");
                OLED_ShowNum(6, 3, (int32_t)high_us, 5);
                OLED_ShowString(36, 3, "us T");
                OLED_ShowNum(60, 3, (int32_t)period_us, 5);
                OLED_ShowString(90, 3, "us");

                OLED_ShowString(0, 4, "tick");
                OLED_ShowFloat(24, 4, (float)tick100 / 100.0f, 2);
                OLED_ShowString(60, 4, "us sam");
                OLED_ShowNum(96, 4, (int32_t)duty, 2);
                OLED_ShowString(108, 4, "%");
            }
            else
            {
                OLED_ShowString(0, 3, "NO PULSE ON PIN");
                OLED_ShowString(6, 4, "sam");
                OLED_ShowNum(24, 4, (int32_t)duty, 2);
                OLED_ShowString(36, 4, "%  no edge");
            }

            OLED_ShowString(0, 5, "ARR");
            OLED_ShowNum(18, 5, (int32_t)arr, 5);
            OLED_ShowString(48, 5, "PSC");
            OLED_ShowNum(66, 5, (int32_t)psc, 3);

            /* 占空比大小: duty = 引脚实测(H/T 算出来的); set = 软件设定
             * (ccr/ARR 算出来的)。两个数应该一致 —— 不一致就说明"写的"和
             * "输出的"不是一回事, 对着示波器的读数一眼就能看出差在哪。 */
            OLED_ShowString(0, 6, "duty");
            OLED_ShowFloat(24, 6, duty_pin, 2);
            OLED_ShowString(60, 6, "%");
            OLED_ShowString(78, 6, "ch");
            OLED_ShowNum(90, 6, (int32_t)(id + 1u), 1);
            OLED_ShowString(96, 6, "/5");

            OLED_ShowString(0, 7, "set");
            OLED_ShowFloat(18, 7, duty_set, 2);
            OLED_ShowString(54, 7, "%");

            OLED_Refresh();

            HAL_Delay(ARM_CHECK_DWELL_MS);
        }
    }
}

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "Emm_V5.h"
#include "servo.h"
#include "Car_control.h"

/* ---- 轮子方向定义: 实测反了就把 FORWARD 改成 0, BACKWARD 改成 1 ---- */
#define WHEEL_DIR_FORWARD       1
#define WHEEL_DIR_BACKWARD      0

/* 电机允许的最高转速(RPM), 由 Emm_V5 驱动器限制 */
#define CAR_MAX_RPM             5000

/* 加减速档位, 传给 Emm_V5 的 acc 参数(0 表示直接换向, 越大加速越急) */
#define CAR_ACC                 5

/**
  * @brief  底层: 分别设定四个电机的方向和速度, 最后广播同步启动
  * @param  dir1~dir4  各电机方向: 0 = 后退, 1 = 前进
  * @param  vel1~vel4  各电机转速 RPM(0~5000)
  * @note   1. 先用 snF=true 把速度"缓存"在各驱动器里, 不立即执行;
  *         2. 最后用广播地址 0x00 发同步触发命令, 四个轮子同时启动,
  *            这样才不会出现"前两个轮子先动、后两个慢一步"的跑偏。
  *         ⚠ 内部有 4 个 HAL_Delay(2), 一次调用阻塞 8ms 以上,
  *           绝对不能在任何中断里调用。
  */
void fish_Step_All_Control(uint8_t dir1, uint16_t vel1,
                           uint8_t dir2, uint16_t vel2,
                           uint8_t dir3, uint16_t vel3,
                           uint8_t dir4, uint16_t vel4)
{
    /* 1. 分别设定四个电机速度, snF=true: 命令先缓存, 等待同步触发 */
    Emm_V5_Vel_Control(1, dir1, vel1, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(2, dir2, vel2, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(3, dir3, vel3, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(4, dir4, vel4, CAR_ACC, true);
    HAL_Delay(2);

    /* 2. 广播同步触发(地址 0x00 为广播地址), 四个电机同时转 */
    Emm_V5_Synchronous_motion(0x00);
}

/**
  * @brief  核心: 麦轮逆运动学, 前进 / 横移 / 自转任意组合
  * @param  vx     前进速度 RPM, 正数前进, 负数后退
  * @param  vy     左移速度 RPM, 正数左移, 负数右移
  * @param  omega  自转速度 RPM, 正数逆时针(原地左转), 负数顺时针
  * @note   四轮速度可能为负, 负值表示该轮要反转,
  *         这里统一换算成 (方向, 绝对值转速) 再交给底层。
  */
void Car_Mecanum(int16_t vx, int16_t vy, int16_t omega)
{
    int16_t  speed[4];      /* 各轮带符号的前进速度 */
    uint8_t  dir[4];        /* 换算后的方向 */
    uint16_t rpm[4];        /* 换算后的转速绝对值 */
    uint8_t  i;

    /* 下标 0~3 依次对应电机 1~4 */
    speed[0] = vx + vy + omega;     /* 1 右前 */
    speed[1] = vx - vy + omega;     /* 2 右后 */
    speed[2] = vx + vy - omega;     /* 3 左后 */
    speed[3] = vx - vy - omega;     /* 4 左前 */

    for (i = 0; i < 4; i++)
    {
        if (speed[i] >= 0)
        {
            dir[i] = WHEEL_DIR_FORWARD;
            rpm[i] = (uint16_t)speed[i];
        }
        else
        {
            dir[i] = WHEEL_DIR_BACKWARD;
            rpm[i] = (uint16_t)(-speed[i]);
        }

        /* 限幅, 防止超过驱动器允许的最高转速 */
        if (rpm[i] > CAR_MAX_RPM)
        {
            rpm[i] = CAR_MAX_RPM;
        }
    }

    fish_Step_All_Control(dir[0], rpm[0], dir[1], rpm[1],
                          dir[2], rpm[2], dir[3], rpm[3]);
}

/**
  * @brief  闭环下发的统一入口: 前进 / 横移 / 自转任意组合
  * @param  vx    前进速度 RPM, >0 前进, <0 后退
  * @param  vy    左移速度 RPM, >0 左移, <0 右移 (不横移就传 0)
  * @param  omega 自转速度 RPM, >0 逆时针(原地左转), <0 顺时针
  * @note   内部就是把三个量交给麦轮逆运动学 Car_Mecanum(); 单独留这个入口,
  *         是为了让闭环那边读起来直白: Car_Drive(基准速度, 横移量, 转向修正量)。
  *
  *         代入 vy = 0 可以看出"四驱差速"的本质:
  *           右前 = 右后 = vx + omega
  *           左后 = 左前 = vx - omega
  *         即右侧轮加速、左侧轮减速 -> 车头往左偏,
  *         正好用来抵消航向误差(误差为正表示车头偏右, 需要往左修)。
  *
  *         三种量可以同时非 0 (运动学上是解耦叠加的):
  *           vx = 150, vy = 0,   omega = 0   -> 直行
  *           vx = 150, vy = 0,   omega = 20  -> 直行中往左修正
  *           vx = 0,   vy = 0,   omega = 100 -> 原地左转
  *           vx = 150, vy = 100, omega = 20  -> 边走边左移, 同时纠偏
  */
void Car_Drive(int16_t vx, int16_t vy, int16_t omega)
{
    Car_Mecanum(vx, vy, omega);
}

/**
  * @brief  直行前进
  */
void Car_MoveForward(uint16_t vel)
{
    Car_Mecanum((int16_t)vel, 0, 0);
}

/**
  * @brief  直行后退
  */
void Car_MoveBackward(uint16_t vel)
{
    Car_Mecanum(-(int16_t)vel, 0, 0);
}

/**
  * @brief  左移(车头方向不变, 整车横着往左走)
  */
void Car_MoveLeft(uint16_t vel)
{
    Car_Mecanum(0, (int16_t)vel, 0);
}

/**
  * @brief  右移(车头方向不变, 整车横着往右走)
  */
void Car_MoveRight(uint16_t vel)
{
    Car_Mecanum(0, -(int16_t)vel, 0);
}

/**
  * @brief  原地左转(俯视逆时针)
  */
void Car_TurnLeft(uint16_t vel)
{
    Car_Mecanum(0, 0, (int16_t)vel);
}

/**
  * @brief  原地右转(俯视顺时针)
  */
void Car_TurnRight(uint16_t vel)
{
    Car_Mecanum(0, 0, -(int16_t)vel);
}

/**
  * @brief  停车: 四个电机立即停止, 最后广播同步触发
  * @note   ⚠ 内部有 4 个 HAL_Delay(2), 不能在中断里调用。
  */
void Car_Stop(void)
{
    Emm_V5_Stop_Now(1, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(2, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(3, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(4, true);
    HAL_Delay(2);

    Emm_V5_Synchronous_motion(0x00);
}

/* ==================== 舵机应用层 ==================== */

/* 缓动时的步进周期(ms)。
 * 取 20ms, 和舵机 PWM 的周期一致 —— 每个 PWM 周期更新一次目标角度。 */
#define SERVO_MOVE_STEP_MS      20u

/**
  * @brief  让某个舵机在指定时间内平滑转到目标角度
  * @param  id            舵机 ID (SERVO_ID1 ~ SERVO_ID5)
  * @param  target_angle  目标角度, 0~180 度, 超出自动限幅
  * @param  duration_ms   缓动时长(毫秒); 传 0 则立即到位
  * @note   实现就是把总变化量按时间线性分成很多小步, 每 20ms 走一步:
  *             当前角度 = 起始角度 + (已过时间/总时间) * (目标 - 起始)
  *         因为每步只动一点点, 舵机就不会猛地冲过去。
  *
  *         ⚠ 这是阻塞函数, 整个缓动期间 CPU 都停在这。
  *           缓动 500ms 就是 500ms 内不处理按键/菜单/跑图逻辑,
  *           所以别在实时性要求高的地方用。
  */
void Servo_MoveTo(uint8_t id, float target_angle, uint16_t duration_ms)
{
    float    start_angle;
    float    delta;
    uint32_t t0;
    uint16_t elapsed;

    if (id >= SERVO_NUM)
    {
        return;
    }

    /* 目标角度限幅 */
    if (target_angle < 0.0f)
    {
        target_angle = 0.0f;
    }
    else if (target_angle > 180.0f)
    {
        target_angle = 180.0f;
    }

    /* duration_ms = 0 就直接到位, 等价于 Servo_SetAngle() */
    if (duration_ms == 0u)
    {
        Servo_SetAngle(id, target_angle);
        return;
    }

    start_angle = Servo_GetAngle(id);
    delta       = target_angle - start_angle;

    t0 = HAL_GetTick();

    while (1)
    {
        elapsed = (uint16_t)(HAL_GetTick() - t0);

        if (elapsed >= duration_ms)
        {
            break;
        }

        Servo_SetAngle(id,
                       start_angle + delta * ((float)elapsed / (float)duration_ms));

        HAL_Delay(SERVO_MOVE_STEP_MS);
    }

    /* 收尾: 保证精确落在目标角度上(循环里最后一步可能差一点) */
    Servo_SetAngle(id, target_angle);
}

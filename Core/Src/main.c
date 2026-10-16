/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Handle_One.h"      /* 航向角 PD 闭环: 中断取数/算 + 主循环下发 */
#include "Handle_Second.h"   /* 机械臂舵机调试(底座扫描找零度) + USART3 占位 */
#include "jy901s.h"
#include "OLED.h"
#include "key.h"
#include "Menu.h"
#include "wireless_uart.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向角 PD 闭环 ====================
 * 闭环的状态量(g_yaw_* / g_drive_*)、PD 参数、TIM3 中断回调、
 * 启动停车逻辑、输出下发, 全部搬到了 Function/Handle_One.c,
 * 对外接口见 Function/Handle_One.h。
 * main.c 这边只需要: 初始化调 Handle_One_Init(), 主循环调 Handle_One_Run()。
 * ================================================================= */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* (SysTick 优先级在 Handle_One_Init() 里设置 —— 它是 TIM3 中断里
   *  读 I2C 能正常判超时的前置条件, 属于闭环模块自己的事) */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化 JY901S 姿态模块(I2C 接口, PB6=SCL, PB7=SDA) */
  JY901S_Init();

  /* 初始化 OLED(软件I2C, PD10=SCL, PD8=SDA) */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "JY901S init...");
  OLED_Refresh();

  /* 等模块上电稳定, 然后主动读一帧, 确认它能正常应答 */
  HAL_Delay(500);
  (void)JY901S_ReadOnce();

  if (!jy901s_data.ready)
  {
    /* 读不到数据: 检查 PB6/PB7 接线与 4.7k 上拉电阻 */
    OLED_ShowString(0, 2, "No IMU data!");
    OLED_ShowString(0, 4, "Chk wire+pullup");
    OLED_Refresh();
    HAL_Delay(1500);
  }
  else
  {
    JY901S_SetOutputRate(JY901S_RRATE_50HZ);
  }

  /* 起跑前把当前朝向清零, 之后读到的 Yaw 都是相对这个方向 */
  JY901S_ZeroYaw();
  /* 初始化按键(PE0=KEY1, PE1=KEY2, PE2=KEY3, 上拉输入, 按下为低电平)*/
  Key_Init();

  /* 初始化 OLED 菜单, 从第一页(选择界面)开始 */
  Menu_Init();
  /* 初始化无线串口上报(VOFA+ 波形), 只复位内部计时 */
  Wireless_Init();
  /* 初始化舵机: 5 路按 servo.c 里 s_servo_power_on_pulse[] 给的脉宽直接输出 */
  Servo_Init();

#if (ARM_TEST_MODE == ARM_TEST_MODE_OUTPUT)
  /* 舵机输出对账(调试用): 不动舵机, 只把 5 路的 mem/ccr/引脚实测波形打到
   * OLED 上, 用来和示波器读数对账。★ 内部是死循环 —— 看完把 Handle_Second.h
   * 里的 ARM_TEST_MODE 改回 ARM_TEST_MODE_OFF 再重新编译。 */
  Arm_OutputCheck();
#endif
	
	
  /* 初始化航向角 PD 闭环模块*/
  Handle_One_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 航向闭环: 处理启动/停车请求 + 把中断算出的结果下发给电机
     * (内部限频 30ms, 三个量和上次一样就不重发) */
    Handle_One_Run();

    /* 按键轮询检测(内部按 10ms 间隔采样并消抖) */
    Key_Scan();

    /* OLED 菜单: 内部处理按键(切换页面/移动光标)并绘制当前页面,
     * 显示按 100ms 限频刷新。显示内容全部由 Menu.c 负责。 */
    Menu_Run();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ==================== 航向角 PD 闭环实现 ====================
 * 这部分(PD 参数宏、状态量、TIM3 中断回调、启动/停车、输出下发)
 * 已经整体搬到 Function/Handle_One.c, 接口见 Function/Handle_One.h。
 * 这里不再放具体实现, 免得两处逻辑对不上。
 * ================================================================= */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  ******************************************************************************
  * @file    main.c
  * @brief   01-blink：板载 LED（PC13）+ 外接 LED（PA5）非阻塞闪烁
  *
  *          硬件：STM32F103C8T6 最小系统板（Blue Pill）
  *          时钟：系统 72 MHz（HSE 8 MHz 晶振 × PLL9），APB1 = 36 MHz，APB2 = 72 MHz
  *
  *          - 板载 LED 接在 PC13，低电平点亮（电流从 3.3 V 经 LED "吸"进引脚）
  *          - 外接 LED 接在 PA5，高电平点亮（电流从引脚"吐"出来经 LED 到 GND）
  *          - 两个灯用不同的半周期闪烁，靠 HAL_GetTick() 做非阻塞延时
  *
  *          说明：本文件按 STM32CubeMX 6.x 生成的 main.c 骨架整理。
  *          USER CODE BEGIN / USER CODE END 成对注释之间的内容是我自己写的，
  *          重新用 CubeMX 生成代码时这部分会被保留；其余框架代码与 CubeMX 生成的一致。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 本章不需要额外的头文件：HAL_GPIO_xxx / __HAL_RCC_xxx 都由 main.h 里的
   stm32f1xx_hal.h 带进来了 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LED 闪烁的"半周期"：一个电平保持多久才翻转一次，单位 ms。
   半周期 500 ms -> 一个完整亮灭周期 1000 ms -> 1 Hz
   半周期 100 ms -> 一个完整亮灭周期  200 ms -> 5 Hz */
#define LED_PC13_HALF_PERIOD_MS   500U   /* 板载 LED（PC13） */
#define LED_PA5_HALF_PERIOD_MS    100U   /* 外接 LED（PA5）  */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  uint32_t now;        /* 本次循环读到的时刻，单位 ms */
  uint32_t tick_pc13;  /* 板载 LED 上一次翻转的时刻，单位 ms */
  uint32_t tick_pa5;   /* 外接 LED 上一次翻转的时刻，单位 ms */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* 两个"上次翻转时刻"都从当前 tick 开始计时，
     这样上电后要等一个完整半周期才会翻转第一次，起始节奏一致 */
  now       = HAL_GetTick();
  tick_pc13 = now;
  tick_pa5  = now;

  /* 进主循环之前把两个灯都放回"灭"的状态。
     PC13 是低电平点亮，所以写 SET（高）才是灭；
     PA5  是高电平点亮，所以写 RESET（低）才是灭。
     这一步的作用是让上电后的初始状态是确定的——
     复位后 ODR 全为 0，PC13 一旦被切成输出就会立刻亮起来 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* PC13 = 高 -> 板载 LED 灭 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET); /* PA5  = 低 -> 外接 LED 灭 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 一个循环只读一次时间，两个灯共用同一个时间基准 */
    now = HAL_GetTick();

    /* ---------- 板载 LED（PC13）：每 500 ms 翻转一次 ---------- */
    /* 判断"过去了多久"用 (now - tick) >= 周期，而不是 now >= tick + 周期。
       HAL_GetTick() 返回 uint32_t 的毫秒数，连续运行约 49.7 天会溢出回绕；
       无符号数相减在回绕后依然得到正确的时间差，而先加后比会算错 */
    if ((uint32_t)(now - tick_pc13) >= LED_PC13_HALF_PERIOD_MS)
    {
      tick_pc13 = now;                          /* 先把时间基准推到当前 */
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);    /* 再翻转电平 */
    }

    /* ---------- 外接 LED（PA5）：每 100 ms 翻转一次 ---------- */
    if ((uint32_t)(now - tick_pa5) >= LED_PA5_HALF_PERIOD_MS)
    {
      tick_pa5 = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }

    /* 这个 while(1) 里没有任何 HAL_Delay：两个灯互不阻塞。
       以后往主循环里加串口收发、ADC 采样、PID 计算时，
       也不会因为某个灯要等 500 ms 而让其他任务停摆。
       代价是这一轮循环会以"最快速度"空转（72 MHz 下每秒几十万次），
       真正的项目里会在末尾加一个短延时或改成定时器中断里翻转 */
    /* USER CODE END 3 */
  }
  /* USER CODE BEGIN 4 */
  /* USER CODE END 4 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;             /* 板上 8 MHz 无源晶振 */
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;    /* 8 MHz / 1 = 8 MHz */
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;             /* 内部 8 MHz RC，留作备用 */
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;           /* 8 MHz × 9 = 72 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; /* SYSCLK = 72 MHz */
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         /* HCLK   = 72 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           /* PCLK1  = 36 MHz（上限 36 MHz） */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           /* PCLK2  = 72 MHz（GPIO 挂在这条总线上） */

  /* 72 MHz 时 Flash 需要 2 个等待周期，否则取指会出错 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable ------------------------------------------------*/
  /* 这三行是 CubeMX 自动生成的。GPIO 挂在这条时钟上，
     不使能时钟，端口寄存器的写操作全部无效——引脚不会有任何反应 */
  __HAL_RCC_GPIOC_CLK_ENABLE();   /* 板载 LED 在 PC13，必须开 GPIOC 时钟 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();   /* 外接 LED 在 PA5，必须开 GPIOA 时钟 */

  /*Configure GPIO pin Output Level -----------------------------------------*/
  /* 注意这两行在 HAL_GPIO_Init() 之前：先把 ODR 里的电平写成"灭"，
     再把引脚切成输出，引脚一被使能就是灭的，不会闪一下 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* PC13 高 = 灭 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET); /* PA5  低 = 灭 */

  /*Configure GPIO pin : PC13 -----------------------------------------------*/
  GPIO_InitStruct.Pin   = GPIO_PIN_13;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;    /* 推挽输出：能吐能吸 */
  GPIO_InitStruct.Pull  = GPIO_NOPULL;            /* 输出模式下的上下拉不起作用 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* F1 的 HAL 里 LOW = 2 MHz 档。
                                                     PC13 属于后备域，手册要求限制在
                                                     2 MHz 以内，而且点灯也不需要更快 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 ------------------------------------------------*/
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* 1 Hz / 5 Hz 翻转，2 MHz 档绰绰有余 */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 出错时把 CPU 停在这里，方便用调试器看调用栈。
     真项目里会在这里点亮一个故障指示灯或者复位看门狗 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* 用户可以用 printf 把 file 和 line 打出来，定位是哪个参数配错了 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

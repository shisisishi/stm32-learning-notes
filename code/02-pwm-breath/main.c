/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   第 04 章 · 定时器与 PWM：呼吸灯 + 舵机
  *
  *  功能 1  TIM2_CH1 (PA5) → 1 kHz PWM 驱动外接 LED，做 2.00 s 周期的呼吸灯
  *  功能 2  TIM3_CH1 (PA6) → 50 Hz PWM 驱动舵机，在 0~180° 之间来回扫
  *
  *  ---------------------------------------------------------------------------
  *  时基计算（全部由 72 MHz 推出）
  *  ---------------------------------------------------------------------------
  *  定时器时钟：APB1 = 36 MHz，但 APB1 预分频系数 = 2（≠ 1），
  *              硬件把 TIM2~TIM7 的时钟 ×2  →  f_CK_PSC = 36 MHz × 2 = 72 MHz
  *              （RM0008 第 7 章 RCC；CubeMX 里叫 "APB1 Timer clocks"）
  *
  *  TIM2（呼吸灯）
  *      f_CK_CNT = 72 MHz / (PSC + 1) = 72 MHz / (71 + 1) = 1 MHz
  *                 → 1 个计数 = 1 µs
  *      f_PWM    = 1 MHz / (ARR + 1)  = 1 MHz / (999 + 1) = 1000 Hz = 1 kHz
  *                 → 周期 1 ms，占空比分辨率 = ARR + 1 = 1000 级
  *      验算：72 000 000 / 72 / 1000 = 1000 Hz ✅
  *
  *  TIM3（舵机）
  *      f_CK_CNT = 72 MHz / (71 + 1) = 1 MHz  → 1 个计数 = 1 µs
  *      f_PWM    = 1 MHz / (19999 + 1) = 50 Hz
  *                 → 周期 20 ms，分辨率 = 1 µs
  *      验算：72 000 000 / 72 / 20000 = 50 Hz ✅
  *      舵机角度换算（CCR 的数值就是高电平的微秒数）：
  *          0°   → 0.5 ms → CCR =  500
  *          90°  → 1.5 ms → CCR = 1500
  *          180° → 2.5 ms → CCR = 2500
  *
  *  ---------------------------------------------------------------------------
  *  哪些是 CubeMX 生成的，哪些是我写的
  *  ---------------------------------------------------------------------------
  *  CubeMX 生成：文件头之外的 HAL 初始化、SystemClock_Config()、
  *               MX_GPIO_Init()、MX_TIM2_Init()、MX_TIM3_Init()、
  *               stm32f1xx_hal_msp.c 里的 HAL_TIM_Base_MspInit / HAL_TIM_MspPostInit
  *  我写的     ：所有 USER CODE BEGIN xxx 到 USER CODE END xxx 之间的内容，
  *               也就是第 4.1 / 4.2 / 4.8 节里的定义、变量和 App_xxx 函数，
  *               以及三个初始化函数里我标的参数值。
  *               （C 的块注释不能嵌套，所以本行和内层注释符号做了区分，
  *                 实际代码里它们都带块注释符号。）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 本章没用到额外头文件：呼吸灯是查表 + 整数递推，不需要 math.h */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- 呼吸灯：查表参数的来源 ----
 * 每 50 ms 更新一次 CCR，走完 41 个采样点 = (41-1) × 50 ms = 2000 ms = 2.00 s。
 * 注意"更新节拍"和"PWM 载波频率"是两件独立的事：
 *   载波频率 1 kHz  → 决定 LED 亮灭切换得多快，快到人眼看不出闪
 *   更新节拍 50 ms  → 决定亮度多久变一次，也就是呼吸的快慢 */
#define BREATH_LUT_LEN        41U
#define BREATH_STEP_MS        50U

/* ---- 舵机参数 ----
 * TIM3 的 1 个计数 = 1 µs，所以 CCR 的数值就等于高电平的微秒数。 */
#define SERVO_MIN_CCR         500U     /* 0.5 ms 高电平 → 0°   */
#define SERVO_MAX_CCR         2500U    /* 2.5 ms 高电平 → 180° */
#define SERVO_MID_CCR         1500U    /* 1.5 ms 高电平 → 90°  */
#define SERVO_STEP_MS         20U      /* 每 20 ms 让角度走 1°，180° 单程 3.6 s */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* 呼吸灯亮度表：CCR 值，三角波（线性升 1.00 s + 线性降 1.00 s，合计 2.00 s）
 *
 * 生成规则：一个周期有 40 个台阶，每台阶 50 ms，两端都要采到，所以表长 41（下标 0~40）。
 *   下标 0       ：CCR = 0                                        （全灭）
 *   下标 1~20    ：CCR = 50, 100, 150, ..., 950                   亮度上升
 *   下标 20      ：CCR = 998                                      （最亮）
 *   下标 21~39   ：CCR = 948, 898, ..., 48                        亮度下降
 *   下标 40      ：CCR = 0                                        （回到全灭，与下标 0 同一个状态）
 *
 * 一个完整周期的时长 = 40 个台阶 × 50 ms = 2000 ms = 2.00 s。
 * （41 个采样点之间有 40 段间隔，所以是 40 × 50 ms，不是 41 × 50 ms。）
 *
 * 最大值故意只取 998，不取 999（= ARR）：
 * PWM 模式 1 下 CCR > ARR 时 CNT 永远追不上 CCR，输出会贴死在常高电平，
 * 亮度就"卡"在最大值不动了。留一格余量最保险。
 *
 * 为什么用查表而不是算 sin()：查表是整数运算，不用调 libm，
 * 而且每个台阶的 CCR 值在源码里看得见，方便对着逻辑分析仪核。 */
static const uint16_t g_breath_lut[BREATH_LUT_LEN] = {
       0U,   50U,  100U,  150U,  200U,     /* 下标  0~  4 时刻   0~ 200 ms */
     250U,  300U,  350U,  400U,  450U,     /* 下标  5~  9 时刻 250~ 450 ms */
     500U,  550U,  600U,  650U,  700U,     /* 下标 10~ 14 时刻 500~ 700 ms（中等亮度） */
     750U,  800U,  850U,  900U,  950U,     /* 下标 15~ 19 时刻 750~ 950 ms */
     998U,  948U,  898U,  848U,  798U,     /* 下标 20~ 24 时刻 1000~1200 ms（20 是最亮，之后转暗） */
     748U,  698U,  648U,  598U,  548U,     /* 下标 25~ 29 时刻 1250~1450 ms */
     498U,  448U,  398U,  348U,  298U,     /* 下标 30~ 34 时刻 1500~1700 ms */
     248U,  198U,  148U,   98U,   48U,     /* 下标 35~ 39 时刻 1750~1950 ms */
       0U                                   /* 下标 40     时刻 2000 ms（回到全灭） */
};

static uint8_t  g_breath_idx  = 0U;     /* 呼吸灯查表下标，0 ~ 40 */
static uint16_t g_servo_angle = 0U;     /* 舵机当前角度，0 ~ 180（单位：度） */
static int8_t   g_servo_dir   = 1;      /* 舵机扫描方向：+1 增大，-1 减小 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void App_BreathLed_Init(void);
static void App_BreathLed_Task(void);
static void App_Servo_Init(void);
static void App_Servo_Task(void);
static uint16_t App_Servo_AngleToCcr(uint16_t angle_deg);
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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 关键：把 PWM 输出打开 ----
   * MX_TIMx_Init() 只是把寄存器的值配好了，CCER 里的通道使能位 CCxE 还是 0，
   * 通道引脚处于"不输出"状态。少了下面两行，配置全对但引脚上一点波形也没有。
   *   htim2 + TIM_CHANNEL_1 → PA5（呼吸灯）
   *   htim3 + TIM_CHANNEL_1 → PA6（舵机） */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* 给一个确定的初始状态，避免进主循环前输出不定 */
  App_BreathLed_Init();          /* PA5 → CCR = 0，LED 灭 */
  App_Servo_Init();              /* PA6 → CCR = 1500，舵机停在 90° */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    App_BreathLed_Task();        /* 每 50 ms 按表改一次 CCR，2.00 s 走完一个呼吸周期 */
    App_Servo_Task();            /* 每 20 ms 让角度走 1°，在 0° 和 180° 之间来回扫 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                 /* 板载 8 MHz 晶振 */
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;             /* 8 MHz × 9 = 72 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   /* 系统时钟 = PLL = 72 MHz */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          /* HCLK = 72 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           /* APB1 = 36 MHz（分频系数 2） */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           /* APB2 = 72 MHz */

  /* ⚠️ APB1 = 36 MHz，但因为分频系数是 2（≠ 1），硬件把 TIM2~TIM7 的时钟自动 ×2：
   *        36 MHz × 2 = 72 MHz   ← 这才是我的定时器时钟
   *    所以 PSC 填 71（72 MHz / 72 = 1 MHz），不是 35。
   *    这个数在 CubeMX 的 Clock Configuration 页叫 "APB1 Timer clocks"。 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;              /* ← PSC = 71：72 MHz / (71+1) = 1 MHz，1 个计数 = 1 µs */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 向上计数 = 边沿对齐 */
  htim2.Init.Period = 999;                /* ← ARR = 999：(999+1) = 1000 个计数
                                           *   验算：1 MHz / 1000 = 1000 Hz = 1 kHz，周期 1 ms
                                           *   占空比分辨率 = ARR + 1 = 1000 级（0.1%） */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 只影响输入捕获滤波采样，不改 PWM 频率 */
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;  /* 用内部时钟 CK_INT */
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- 通道 1（PA5，呼吸灯）----
   * Pulse 就是 CCR 的初值。边沿对齐 + PWM 模式 1 下：占空比 = CCR / (ARR+1) = CCR / 1000
   * 填 0 → 0% 占空比，上电时 LED 灭，进主循环后被查表值接管。 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  /* 模式 1：CNT < CCR 时输出有效电平 */
  sConfigOC.Pulse = 0;                                 /* CCR1 初值 = 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          /* 有效电平 = 高；PA5 高电平点亮 LED */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

  /* ---- 关键：把 PA5 配成复用推挽输出 ----
   * HAL_TIM_MspPostInit() 本身是 CubeMX 生成的（在 stm32f1xx_hal_msp.c 里），
   * 但 CubeMX 不会自动调它，必须在这里显式调用一次。
   * 少了这一步，定时器内部计数一切正常，引脚上却什么都没有。 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;              /* ← PSC = 71：72 MHz / (71+1) = 1 MHz，1 个计数 = 1 µs
                                           *   TIM3 和 TIM2 一样挂在 APB1 上，
                                           *   时钟同样是 36 MHz × 2 = 72 MHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;              /* ← ARR = 19999：(19999+1) = 20000 个计数
                                           *   验算：1 MHz / 20000 = 50 Hz，周期 20 ms ← 舵机要的
                                           *   分辨率 = 1 个计数 = 1 µs
                                           *   位宽检查：19999 < 65535 ✅ 16 位装得下 */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- 通道 1（PA6，舵机）----
   * Pulse = CCR = 1500 → 高电平 1500 µs = 1.5 ms → 舵机 90°（中间位）
   * 占空比 = 1500 / 20000 = 7.5%（舵机只看脉宽，不看占空比） */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_MID_CCR;                     /* 1500 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

  HAL_TIM_MspPostInit(&htim3);            /* 把 PA6 配成复用推挽输出，必须显式调用 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* PA5 / PA6 本章由 HAL_TIM_MspPostInit() 配成复用功能，不在这里当普通 GPIO 配。
   * 这里只把用到的 GPIO 端口时钟打开。 */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  呼吸灯初始化：CCR = 0 → 占空比 0% → LED 灭
  * @note   TIM2：1 个计数 = 1 µs，ARR = 999，占空比 = CCR / 1000
  */
static void App_BreathLed_Init(void)
{
  g_breath_idx = 0U;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_breath_lut[g_breath_idx]);
}

/**
  * @brief  呼吸灯任务：每 50 ms 取下一个查表值写入 CCR1
  * @note   41 个采样点 × 50 ms = 2.00 s 一个完整呼吸周期
  *         上升段 1.00 s（CCR: 0 → 950），下降段 1.00 s（CCR: 998 → 48）
  *         PA5 是高电平点亮，所以 CCR 越大 LED 越亮。
  *         只改 CCR、不动 PSC/ARR，所以载波频率始终是 1 kHz，变的只有占空比。
  */
static void App_BreathLed_Task(void)
{
  g_breath_idx++;
  if (g_breath_idx >= BREATH_LUT_LEN)
  {
    g_breath_idx = 0U;
  }

  /* 运行时改 CCR 用这个宏，不需要 Stop/Start 一次 PWM */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_breath_lut[g_breath_idx]);

  HAL_Delay(BREATH_STEP_MS);      /* 50 ms 是 HAL_Delay 最小粒度 1 ms 的整数倍，节拍是准的 */
}

/**
  * @brief  角度 → CCR
  * @param  angle_deg: 期望角度，0 ~ 180（单位：度）
  * @retval CCR 值（TIM3 下 CCR 的数值就等于高电平的微秒数）
  * @note   CCR = 500 + angle / 180 × (2500 - 500) = 500 + angle × 2000 / 180
  *         0°   → 500  → 0.5 ms 高电平
  *         90°  → 1500 → 1.5 ms 高电平
  *         180° → 2500 → 2.5 ms 高电平
  */
static uint16_t App_Servo_AngleToCcr(uint16_t angle_deg)
{
  uint32_t ccr;

  if (angle_deg > 180U)          /* 限幅：防止算出超过 2.5 ms 的脉宽（超范围会让舵机堵转） */
  {
    angle_deg = 180U;
  }

  /* 用整数算，避免引入浮点和 libm：
   * (2500 - 500) / 180 = 2000 / 180 = 100 / 9
   * → CCR = 500 + angle × 100 / 9
   * 例：angle = 90 → 500 + 90 × 100 / 9 = 500 + 1000 = 1500 ✅
   *     angle = 180 → 500 + 180 × 100 / 9 = 500 + 2000 = 2500 ✅ */
  ccr = 500U + ((uint32_t)angle_deg * 100U) / 9U;

  if (ccr > 65535U)              /* CCR 是 16 位，保险起见夹一下 */
  {
    ccr = 65535U;
  }
  return (uint16_t)ccr;
}

/**
  * @brief  舵机初始化：摆到中间位 90°
  */
static void App_Servo_Init(void)
{
  g_servo_angle = 90U;
  g_servo_dir   = 1;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, App_Servo_AngleToCcr(g_servo_angle));
}

/**
  * @brief  舵机任务：每 20 ms 让角度走 1°，到 0° 或 180° 就反向
  * @note   180 步 × 20 ms = 3.60 s 扫完单程。
  *         这里把任务节拍设成和舵机 PWM 周期一样（都是 20 ms），
  *         是为了让"每个 PWM 周期收到一条新位置指令"这件事看得最清楚。
  *         注意 CCR 的生效时机：OCxPE = 0 时写进去立刻参与比较，
  *         但如果这一周期的 CNT 已经跑过新 CCR，效果要等下一个周期才看全。
  */
static void App_Servo_Task(void)
{
  g_servo_angle = (uint16_t)((int16_t)g_servo_angle + g_servo_dir);

  if (g_servo_angle >= 180U)
  {
    g_servo_angle = 180U;
    g_servo_dir   = -1;
  }
  else if (g_servo_angle == 0U)
  {
    g_servo_dir   = 1;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, App_Servo_AngleToCcr(g_servo_angle));

  HAL_Delay(SERVO_STEP_MS);
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

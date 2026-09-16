/**
 * main.c —— 第 07 章 ADC 采样实验
 *
 * 实验内容：
 *   1. ADC1_IN1（PA1）接 10k 电位器，读单通道单次转换；
 *   2. 同一路 ADC1 上开扫描序列，规则组里有 2 个通道：
 *      Rank1 = PA1 电位器，Rank2 = 内部温度传感器；
 *   3. 上电先做一次 ADC 校准（HAL_ADCEx_Calibration_Start）；
 *   4. 电位器这一路串上"中值滤波 → 滑动平均 → 一阶低通"，
 *      原始值 / 电压值 / 滤波后电压值三列一起用串口打出来，
 *      串口助手接上就能画曲线。
 *
 * 这份文件的组成：
 *   - `HAL_Init()` / `SystemClock_Config()` / `MX_xxx_Init()` / `Error_Handler()`
 *     这几块是 STM32CubeMX 生成的；夹在 USER CODE BEGIN / END 之间的都是我加的
 *     （保留 CubeMX 的注释框架，重新生成工程时我自己写的代码不会被冲掉，
 *     这也是一开始就把代码写进 USER CODE 区的唯一原因）；
 *   - 滤波算法放在 filter.c / filter.h，是**与硬件无关**的纯 C，
 *     不依赖 HAL，PC 上也能单独编译测试。
 *
 * 硬件接线（Blue Pill）：
 *   PA1  ← 电位器中间脚（电位器两端分别接 3V3 和 GND）
 *   PA9  → USB-TTL 的 RXD      PA10 ← USB-TTL 的 TXD      GND ↔ GND
 *   电位器两端最好各并一个 100nF 到地（第 6 节坑 4 会讲为什么）
 *
 * 串口：USART1，115200 8N1
 * 状态：🔬 代码按 CubeMX + HAL 库写好，我还没烧录实测
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "filter.h"     /* 与硬件无关的滤波模块，在 code/05-adc/filter.c */
#include <stdio.h>      /* snprintf */
#include <string.h>     /* strlen  */
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* ---- ADC 参数 ---------------------------------------------------------- */

/**
 * 规则组里的通道个数。
 * ⚠️ 这个数字必须和 CubeMX 里 ADC1 的 "Number Of Conversions" 填的完全一致，
 *    也要和 MX_ADC1_Init() 里配了几个 Rank 一致，否则读出来的数组是错位的。
 *    填 1 → 就是"单通道单次转换"；填 2 → 就是扫描序列。
 */
#define ADC_CH_NUM          2u

/* 扫描序列里的下标：顺序 = CubeMX 里 Rank 的顺序 */
#define ADC_IDX_POT         0u      /* Rank1：PA1 电位器        */
#define ADC_IDX_TEMP        1u      /* Rank2：内部温度传感器    */

/**
 * 12 位 ADC 的刻度总数 = 2^12 = 4096。
 * LSB = 3300mV / 4096 = 0.80566 mV
 * 注意分母是 4096 而不是 4095，第 2.2 节专门讲了这两种算法的区别。
 */
#define ADC_FULL_SCALE      4096.0f
#define VREF_MV             3300.0f
#define LSB_MV              (VREF_MV / ADC_FULL_SCALE)

/* ---- 内部温度传感器参数（取自 STM32F103 数据手册的电气特性表）---------- */
/* T(℃) = (V25 − V_SENSE) / Avg_Slope + 25                        */
/* V25 典型值 1.43V = 1430mV，Avg_Slope 典型值 4.3mV/℃            */
/* ⚠️ 这两个都是"典型值"，个体差异能到 ±0.1V 量级，所以别拿它当温度计用， */
/*    只能看"有没有在变、往哪边变"。真要测温得外接传感器。          */
#define TS_V25_MV           1430.0f
#define TS_SLOPE_MV_PER_C   4.3f

/* ---- 采样与滤波参数 ---------------------------------------------------- */

#define SAMPLE_PERIOD_MS    5u      /* 采样周期 5ms → 200Hz  */
#define LP_TAU_MS           20.0f   /* 最后一级低通的时间常数 τ = 20ms */
#define LP_DT_MS            ((float)SAMPLE_PERIOD_MS)

/**
 * 一共采样 PRINT_DIV 次才打印一次。
 * 为什么不是每次都打：115200 下一行 24 个字符要占 24 × 86.8µs ≈ 2.1ms，
 * 而整个采样周期只有 5ms —— 打印会吃掉 42% 的时间（见第 5 节）。
 * 4 次打一次 = 20ms 打一次 = 50Hz，画曲线完全够。
 */
#define PRINT_DIV           4u
#define TEMP_PRINT_DIV      200u    /* 200 × 5ms = 1s 打一次温度 */

/* USER CODE END PD */

/* USER CODE BEGIN PV */
static uint16_t g_adc_raw[ADC_CH_NUM];   /* 规则组扫描结果，[0]=电位器 [1]=温度 */
static AdcFilterChain_t g_filter_pot;    /* 电位器这一路的滤波链 */
static uint32_t g_print_div  = 0u;
static uint32_t g_temp_div   = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void     ADC1_Calibrate(void);
static uint16_t ADC1_ReadSingleBlocking(void);
static void     ADC1_ReadSequence(uint16_t *dst, uint16_t n);
static float    ADC_RawToMv(uint16_t raw);
static float    ADC_RawToVolt(uint16_t raw);
static float    InternalTemp_ToCelsius(uint16_t raw);
static void     UART_PrintLine(const char *s);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* ========================================================================== */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t raw, med;
  uint32_t avg;
  float    mv_raw, mv_filt;
  uint32_t uv_raw, uv_filt;
  char     line[64];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* 系统时钟 72MHz：HSE 8MHz 晶振 × PLL9 */
  SystemClock_Config();

  /* 初始化所有外设 */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* ① 先校准。必须在 ADC 上电之后、第一次转换之前做，而且只要做一次。
   *    不做的话误差能到几个 LSB（几 mV），第 6 节坑 5 会算这笔账。 */
  ADC1_Calibrate();

  /* ② 初始化滤波链：τ = 20ms，采样周期 dt = 5ms */
  AdcFilterChain_Init(&g_filter_pot, LP_TAU_MS / 1000.0f, LP_DT_MS / 1000.0f);

  /* ③ 打印表头，方便把串口数据直接存成 CSV 丢给绘图工具 */
  UART_PrintLine("\r\n[07] ADC1 ready: 12-bit, VREF=3.3V, 1LSB=0.806mV\r\n");
  UART_PrintLine("sample, raw, volt_mV, filt_mV\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t n = 0u;

  while (1)
  {
    /* ---- 第 1 步：把规则组里 ADC_CH_NUM 个通道全读一遍 ---- */
#if (ADC_CH_NUM == 1u)
    /* 单通道单次转换（CubeMX 里 ScanConvMode = Disable） */
    g_adc_raw[ADC_IDX_POT] = ADC1_ReadSingleBlocking();
#else
    /* 多通道扫描（CubeMX 里 ScanConvMode = Enable） */
    ADC1_ReadSequence(g_adc_raw, (uint16_t)ADC_CH_NUM);
#endif

    /* ---- 第 2 步：滤波 ---- */
    raw = g_adc_raw[ADC_IDX_POT];
    med = Median_Update(&g_filter_pot.med, raw);            /* 抠脉冲尖峰 */
    avg = MovAvg_Update(&g_filter_pot.avg, (uint32_t)med);  /* 压随机噪声 */
    mv_raw  = ADC_RawToMv(raw);
    mv_filt = LowPass_Update(&g_filter_pot.lp, ADC_RawToMv((uint16_t)avg));

    /* ---- 第 3 步：每 PRINT_DIV 次打印一行 ---- */
    g_print_div++;
    if (g_print_div >= PRINT_DIV)
    {
      g_print_div = 0u;

      /* 单位统一成 微伏(µV) 的整数再打印：
       * 12 位 ADC 满量程才 3300mV，用整数打 µV 分辨率到 1µV，
       * 比用 printf 的 %f 省掉一大堆浮点格式化代码（MicroLIB 的 %f 又慢又占空间）。 */
      uv_raw  = (uint32_t)(mv_raw  * 1000.0f + 0.5f);
      uv_filt = (uint32_t)(mv_filt * 1000.0f + 0.5f);

      (void)snprintf(line, sizeof(line),
                     "%lu, %u, %lu.%03lu, %lu.%03lu\r\n",
                     (unsigned long)n,
                     (unsigned)raw,
                     (unsigned long)(uv_raw  / 1000u), (unsigned long)(uv_raw  % 1000u),
                     (unsigned long)(uv_filt / 1000u), (unsigned long)(uv_filt % 1000u));
      UART_PrintLine(line);
    }

    /* ---- 第 4 步：每秒打一次内部温度，量个趋势 ---- */
    g_temp_div++;
    if (g_temp_div >= TEMP_PRINT_DIV)
    {
      float   t   = InternalTemp_ToCelsius(g_adc_raw[ADC_IDX_TEMP]);
      int32_t t10 = (int32_t)(t * 10.0f);   /* 以 0.1℃ 为单位的整数 */
      int32_t whole = t10 / 10;
      int32_t frac  = t10 % 10;

      g_temp_div = 0u;
      if (frac < 0) { frac = -frac; }       /* 负温度时别打出 "-2.-5" */

      (void)snprintf(line, sizeof(line), "TEMP, %lu.%lu, C, raw=%u\r\n",
                     (unsigned long)whole, (unsigned long)frac,
                     (unsigned)g_adc_raw[ADC_IDX_TEMP]);
      UART_PrintLine(line);
    }

    n++;

    /* HAL_Delay 是阻塞延时。这里用它是为了先把逻辑跑通；
     * 它有个 1ms 的量化误差（HAL 的 tick 是 1ms），再加上串口打印的耗时，
     * 实际采样周期是 5ms + 一点点。要严格的等间隔采样，得改成
     * "定时器中断里置标志位、主循环等着读"，这个留到做闭环时再改。 */
    HAL_Delay(SAMPLE_PERIOD_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration —— 72MHz
  *        HSE 8MHz 晶振 × PLL9 = 72MHz，ADC 时钟 = PCLK2 / 6 = 12MHz
  *        （F103 的 ADC 时钟上限是 14MHz，所以不能直接给 72MHz）
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

  /* 振荡器：打开 HSE，PLL 源选 HSE，倍频 9 → 8MHz × 9 = 72MHz */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 时钟树：SYSCLK = 72MHz，AHB = 72MHz，APB1 = 36MHz，APB2 = 72MHz */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  /* 72MHz 超出 Flash 的 24MHz 上限，必须插 2 个等待周期 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC 专用时钟分频：ADC 时钟 = PCLK2 / 6 = 72MHz / 6 = 12MHz ≤ 14MHz ✅ */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  *        —— 这一段是 CubeMX 生成后我又手动改了采样时间的版本，
  *           实际的工程里它在 adc.c，这里放一份方便对照第 7 节的配置步骤。
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* ---- ADC1 整体配置 ---- */
  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;      /* 扫描模式：一个序列多个通道 */
  hadc1.Init.ContinuousConvMode    = DISABLE;              /* 单次：读完就停，由主循环决定何时再读 */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;   /* 软件触发（HAL_ADC_Start） */
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;  /* 12 位右对齐：0~4095 */
  hadc1.Init.NbrOfConversion       = ADC_CH_NUM;           /* 序列长度，必须和 ADC_CH_NUM 一致 */
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- Rank1：PA1 电位器，ADC1_IN1 ---- */
  /* 采样时间 55.5 个 ADC 周期 = 55.5 × 83.3ns ≈ 4.63µs。
   * 需要多长时间是按第 2.4 节的公式算的，电位器最大源阻抗 R/4 = 2.5kΩ 时
   * 只要 252.6ns 就够，55.5 周期留了很大余量，是因为同一序列里还有温度传感器。 */
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- Rank2：内部温度传感器 ---- */
  /* ⚠️ 温度传感器必须用 239.5 周期：数据手册要求它的采样时间不少于 17.1µs，
   *    而在 12MHz 的 ADC 时钟下 239.5 周期 = 19.96µs，是唯一够用的档位。
   *    F103 的采样时间是"按通道"配的（SMPR1/SMPR2），所以同一个序列里
   *    两个通道可以用完全不同的采样时间。 */
  sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank         = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function —— 115200 8N1，PA9=TX PA10=RX
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function —— 这个实验里 PA1 被 ADC 占用，
  *        所以 CubeMX 会自动把它配成模拟输入（GPIO_MODE_ANALOG），
  *        千万不要再手动把它设成推挽输出，那样 ADC 读到的是引脚驱动的电平，
  *        而不是外部分压点的电压。
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* PC13 板载 LED（低电平点亮）—— 留一个心跳灯，一眼看出程序没跑飞 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
  * @brief  给 ADC 做一次自校准
  *         HAL 里这一步在 stm32f1xx_hal_adc_ex.h，
  *         底层做的是 RM0008 第 11.4 节 Calibration 描述的流程：
  *         置 CAL 位 → 等 CAL 位被硬件清零 → 校准完成。
  * @param  None
  * @retval None
  */
static void ADC1_Calibrate(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  单通道单次转换（阻塞式）
  *         要求 CubeMX 里 ScanConvMode = Disable、NbrOfConversion = 1
  * @param  None
  * @retval 0~4095 的原始转换结果；超时则返回 0
  */
static uint16_t ADC1_ReadSingleBlocking(void)
{
  uint16_t value = 0u;

  HAL_ADC_Start(&hadc1);                                  /* 软件触发一次转换 */

  /* 等转换结束。10ms 是超时上限：一次转换只要几微秒，
   * 真等满 10ms 说明 ADC 根本没在跑，这时候宁可返回 0 也不要死循环。 */
  if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK)
  {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);           /* 读 DR 寄存器 */
  }

  HAL_ADC_Stop(&hadc1);
  return value;
}

/**
  * @brief  把规则组里 n 个通道依次读出来（扫描模式，阻塞式）
  *         要求 CubeMX 里 ScanConvMode = Enable、NbrOfConversion = n
  * @param  dst 结果数组，长度至少 n
  * @param  n   通道个数
  * @retval None
  *
  * 说明：F103 的 EOC（转换结束）标志是**每转完一个通道就置一次**，
  *       所以这个 for 循环能一个通道一个通道地往下读。
  *       如果实跑发现读到的顺序不对、或者第二个通道超时，
  *       就改成 DMA 搬运（CubeMX 里给 ADC1 加 DMA 请求，循环模式），
  *       那样 CPU 完全不用管，最稳。
  */
static void ADC1_ReadSequence(uint16_t *dst, uint16_t n)
{
  uint16_t i;

  if ((dst == 0) || (n == 0u))
  {
    return;
  }

  HAL_ADC_Start(&hadc1);

  for (i = 0u; i < n; i++)
  {
    if (HAL_ADC_PollForConversion(&hadc1, 10u) != HAL_OK)
    {
      break;   /* 超时就用数组里的旧值，不要让程序卡死在这里 */
    }
    dst[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  HAL_ADC_Stop(&hadc1);
}

/**
  * @brief  原始值 → 电压(mV)      V = raw × 3300mV / 4096
  * @param  raw 0~4095
  * @retval 电压，单位 mV
  */
static float ADC_RawToMv(uint16_t raw)
{
  /* 故意写成乘 LSB_MV 而不是每次除 4096：
   * LSB_MV 是编译期常量，乘法比除法快，而且只有一个地方能写错分母。 */
  return ((float)raw) * LSB_MV;
}

/**
  * @brief  原始值 → 电压(V)
  * @param  raw 0~4095
  * @retval 电压，单位 V
  */
static float ADC_RawToVolt(uint16_t raw)
{
  return ADC_RawToMv(raw) / 1000.0f;
}

/**
  * @brief  内部温度传感器原始值 → 摄氏度
  *         T(℃) = (V25 − V_SENSE) / Avg_Slope + 25
  * @param  raw 温度传感器通道的原始值
  * @retval 摄氏度（典型值参数，个体误差大，只能看趋势）
  */
static float InternalTemp_ToCelsius(uint16_t raw)
{
  float v_sense_mv = ADC_RawToMv(raw);

  return (TS_V25_MV - v_sense_mv) / TS_SLOPE_MV_PER_C + 25.0f;
}

/**
  * @brief  一次把整行字符串发出去
  * @param  s 以 '\0' 结尾的字符串
  * @retval None
  *
  * 为什么不直接用 printf：
  *   Keil 里 printf 的每个字符都要走一次 fputc → 一次阻塞式
  *   HAL_UART_Transmit。115200 下发一个字节要 86.8µs，一行 24 个字符
  *   就是 2.1ms 的纯等待。先在内存里拼好整行再一次性发，
  *   收发次数从 24 次降到 1 次，函数调用开销也省了。
  *   （printf 重定向本身在第 05 章已经做过了，这里不再重复。）
  */
static void UART_PrintLine(const char *s)
{
  if (s == 0)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100u);
}

/* USER CODE BEGIN 4 */

/* ---- printf 重定向 -------------------------------------------------------
 * 两套工具链的底层函数不一样，用条件编译分开：
 *   - Keil MDK-ARM (ARMCC / ARMCLANG)：重定义 fputc，
 *     并且要在 Options for Target → Target 里勾上 "Use MicroLIB"；
 *   - arm-none-eabi-gcc：重定义 _write。
 * 第 05 章踩过这个坑（只勾了 MicroLIB 没重定义 fputc 的变体），这里保留一份备用。
 * ------------------------------------------------------------------------ */
#if defined(__GNUC__)
int _write(int fd, char *ptr, int len)
{
  (void)fd;
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}
#else
int fputc(int ch, FILE *f)
{
  (void)f;
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1u, HAL_MAX_DELAY);
  return ch;
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* 出错就把 PC13 的灯闪一下（板载 LED 低电平点亮） */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile uint32_t i = 0u; i < 200000u; i++) { }
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  /* 用户自己加打印或断点 */
}
#endif

/* USER CODE END 4 */

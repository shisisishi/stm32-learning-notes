/**
 ******************************************************************************
 * @file    main.c
 * @brief   第 06 章 · I2C 总线与 0.96 寸 OLED（SSD1306）
 *
 * 工程由 STM32CubeMX 生成（STM32F103C8Tx，HSE 8MHz，PLL x9 -> SYSCLK 72MHz），
 * 下面用注释标出了"CubeMX 生成的"和"我自己写的"两部分。
 *
 * 外设配置：
 *   I2C1  : PB6 = I2C1_SCL，PB7 = I2C1_SDA，复用开漏，400kHz Fast Mode
 *   USART1: PA9/PA10，115200 8N1（只用来看 printf，可不接）
 *   PC13  : 板载 LED，推挽输出，低电平点亮（用来做心跳）
 *
 * 🔬 本文件尚未上机验证（硬件到货后按第 7 节复现步骤执行）。
 *    预期现象见第 4 节末尾。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* 重定向 printf 到 USART1，方便在串口助手里看打印。
 * fputc 是 Keil MicroLIB 用的钩子函数；没勾 MicroLIB 时用 _write。 */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
#endif

/**
 * @brief  主函数
 */
int main(void)
{
    /* ---- 以下到 while(1) 之前都是 CubeMX 生成的框架 ---- */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* ---- USER CODE BEGIN 2：我自己写的 ---- */

    uint32_t counter = 0U;
    char     line[24];

    printf("\r\n=== STM32F103 + SSD1306 (I2C1 @400kHz) ===\r\n");

    /* 初始化 OLED。返回 HAL_ERROR 通常只有两个原因：
     *   1) 上拉电阻没接 / 没接好，SDA 或 SCL 被拉死
     *   2) 从机地址不对（模块是 0x3D 的时候要改成 (0x3D << 1)）
     * 所以这里把结果打印出来，比"屏幕不亮然后干瞪眼"好定位。 */
    if (SSD1306_Init() != HAL_OK)
    {
        printf("OLED init FAILED: check pull-up resistors and slave address\r\n");
        Error_Handler();
    }
    printf("OLED init OK, addr = 0x%02X (7-bit 0x%02X)\r\n",
           SSD1306_I2C_ADDR, SSD1306_I2C_ADDR >> 1);

    /* 第 1 行（y=0）：静态信息，只写一次显存 */
    SSD1306_ShowString(0, 0, "BNGU! shisisishi");

    /* 第 3 行（y=16）：命令提示，也只写一次 */
    SSD1306_ShowString(0, 16, "I2C1 400kHz");

    /* 推一次屏，把上面两行显示出来 */
    SSD1306_Refresh();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* 先把这一行显存清掉，否则上一次的数字会和新的叠在一起。
         * memset 只清 3 个字符 = 18 个字节，比整屏清 1024 字节便宜得多 */
        SSD1306_ShowString(0, 32, "      ");

        /* 格式化第 41 行（y=32，第 5 页）要显示的文本。
         * snprintf 最多写 sizeof(line) 字节，永远不会越界 */
        (void)snprintf(line, sizeof(line), "cnt=%lu", (unsigned long)counter);

        SSD1306_ShowString(0, 32, line);

        /* 每 500ms 刷一次屏。
         * 刷一次 = 8 页 x (6 条命令 + 4 块数据) ≈ 80 次 I2C 传输，
         * @400kHz 实测应该在 3ms 上下（🔬 待用逻辑分析仪确认）*/
        SSD1306_Refresh();

        counter++;

        /* 板载 LED 翻转，用肉眼确认程序没卡在 I2C 里。
         * PC13 是低电平点亮，所以这里翻转的节奏和屏幕数字一致 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        HAL_Delay(500);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief  系统时钟配置：HSE 8MHz 经 PLL x9 = 72MHz（CubeMX 生成）
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 打开 HSE 和 PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;        /* 8MHz x 9 = 72MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* AHB = 72MHz，APB1 = 36MHz（I2C1 挂在 APB1 上），APB2 = 72MHz */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;       /* 36MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;       /* 72MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  I2C1 初始化：PB6/PB7，400kHz Fast Mode（CubeMX 生成）
 * @note   引脚模式必须是 GPIO_MODE_AF_OD（复用 + 开漏），
 *         不能是推挽。推挽的话两个设备同时驱动会直接短路打架。
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;                 /* 400kHz Fast Mode */
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;        /* 占空比 Tlow/Thigh = 2 */
    hi2c1.Init.OwnAddress1     = 0;                      /* 本机做主机，不用自己的地址 */
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  /* 允许从机拉低 SCL 做时钟拉伸 */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  USART1 初始化：PA9/PA10，115200 8N1（CubeMX 生成）
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
 * @brief  GPIO 初始化（CubeMX 生成）
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PC13 板载 LED，推挽输出，默认灭（高电平） */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB6 = I2C1_SCL，PB7 = I2C1_SDA。
     * 关键点是 GPIO_MODE_AF_OD：复用功能 + 开漏输出。
     * 开漏才能让多个设备共用一根线（线与），上拉电阻负责把线拉回高电平 */
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;      /* 上拉靠外部电阻，不用内部弱上拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 400kHz 需要快速翻转 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  出错时的死循环（CubeMX 生成）
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* 卡在这里时可以量 PC13：如果 LED 一直不闪，说明确实进 Error_Handler 了 */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

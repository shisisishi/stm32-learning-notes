/* main.c —— 第 05 章 串口通信
 * 功能：USART1 (PA9/PA10) 115200 8N1，printf 重定向到串口，
 *       中断接收 + 环形缓冲区，主循环解析命令：'L' 点亮 LED，'O' 熄灭 LED。
 * 生成方式：STM32CubeMX 6.x 生成框架，USER CODE 段内是我自己写的。
 * 依赖：ringbuf.h（同目录，需要手动加入 Keil 工程）
 * 注意：Keil 里要在 Options for Target → Target 勾选 "Use MicroLIB"，否则重定向不生效。
 */
#include "main.h"
#include "ringbuf.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>                    /* 显式包含，不依赖 main.h 间接引入（uint32_t 用得到） */

/* ---------- CubeMX 自动生成的句柄 ---------- */
UART_HandleTypeDef huart1;

/* ---------- 我自己加的全局变量 ---------- */
static ringbuf_t g_rx;                 /* 串口接收环形缓冲区 */
static uint8_t   g_rx_byte;            /* 中断单字节接收的落点，必须常驻内存（不能是局部变量） */

/* ---------- 函数声明 ---------- */
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void        uart_printf_fast(const char *fmt, ...);

int main(void)
{
    uint8_t ch;

    HAL_Init();
    SystemClock_Config();              /* HSE 8MHz × PLL9 = 72MHz */

    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    ringbuf_init(&g_rx);               /* 先清空缓冲区再开中断，顺序不能反 */

    /* 启动第一次中断接收：收满 1 个字节就触发 HAL_UART_RxCpltCallback */
    if (HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1) != HAL_OK) {
        Error_Handler();               /* 返回非 HAL_OK 说明 USART 没准备好 */
    }

    printf("UART1 ready: 115200 8N1\r\n");   /* 启动横幅，看到它就说明收发通了 */
    printf("send 'L' = LED on , 'O' = LED off\r\n");
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        /* 主循环从缓冲区取字节。所有"慢"处理都在这里做，中断里只负责搬运 */
        while (ringbuf_get(&g_rx, &ch) == 1u)
        {
            if (ch == 'L' || ch == 'l') {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* PC13 低电平点亮 */
                printf("LED ON\r\n");
            }
            else if (ch == 'O' || ch == 'o') {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    /* 高电平熄灭 */
                printf("LED OFF\r\n");
            }
            else if (ch == '\r' || ch == '\n') {
                /* 串口助手通常会追加回车换行，直接忽略，不要当成错误命令 */
            }
            else {
                printf("unknown cmd: 0x%02X\r\n", (unsigned int)ch);
            }
        }

        /* 每约 1 秒报一次溢出计数。只要它一直是 0，就说明主循环跟得上收包速度 */
        {
            static uint32_t last_tick = 0;
            if (HAL_GetTick() - last_tick >= 1000u) {
                last_tick = HAL_GetTick();
                if (g_rx.overflow > 0u) {
                    uart_printf_fast("!! rx overflow count = %lu\r\n",
                                     (unsigned long)g_rx.overflow);
                }
            }
        }
        /* USER CODE END 3 */
    }
}

/* ================= 中断回调：中断里只做一件事——把字节扔进环形缓冲区 ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        ringbuf_put(&g_rx, g_rx_byte);          /* 只入队；绝不在中断里解析协议或 printf */

        /* 关键：HAL 的 Receive_IT 是"收满指定个数就停"，必须在回调里重新武装，
           否则永远只能收到第一个字节 */
        HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);
    }
}

/* ================= 错误回调：显式处理 ORE 等硬件错误 =================
 * 如果不处理，一次溢出就可能让 HAL 停掉接收，现象是"跑一会儿就没反应了"。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* HAL_UART_GetError() 返回位掩码，含 HAL_UART_ERROR_ORE / _FE / _NE / _PE。
           调试时可以在这里打断点看它到底是哪一位。 */
        (void)HAL_UART_GetError(huart);

        /* 清掉溢出标志并重新开接收，否则接收会一直卡死 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);

        /* 故意不在这里 printf：错误回调可能运行在中断上下文里，printf 太慢 */
    }
}

/* ================= printf 重定向 =================
 * Keil MDK-ARM (ARMCC / ARMCLANG) 走的是 fputc：
 * 库里的 fputc 是弱符号，我在这里重新定义它就完成了重定向。
 * 慢的原因：每调用一次只发 1 个字节，而 HAL_UART_Transmit 是阻塞的，
 * 115200 下每个字节要等 86.8 µs。printf 一行 20 个字符 = 20 次 × 86.8 µs ≈ 1.7 ms。
 */
int fputc(int ch, FILE *f)
{
    (void)f;                                     /* 参数用不到，显式忽略，避免编译警告 */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ================= 更快的打印写法：先格式化到内存，再一次发出去 =================
 * 总线上一共要发多少位是省不掉的，省掉的是"每次只发 1 字节"的调用开销，
 * 而且这样可以直接换成 DMA 版本，让 CPU 发完就返回。
 */
void uart_printf_fast(const char *fmt, ...)
{
    char    buf[64];                             /* 放栈上；如果用 DMA 就必须改成 static/全局 */
    va_list args;
    int     n;

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0) {
        return;                                  /* 格式化失败，直接返回 */
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;                /* 被截断了，按实际长度发 */
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
    /* 想彻底不阻塞就把上面这行换成：
       HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, (uint16_t)n);
       但那时 buf 必须是 static 或全局——局部数组在函数返回后就失效了，
       DMA 还在搬一块已经被复用的内存，发出去的内容必然对不上。 */
}

/* ================= USART1 初始化：115200 8N1 =================
 * HAL_UART_Init 内部会按 RM0008 27.3.4 节的公式算 BRR：
 *   USARTDIV = f_CK / (16 × baud) = 72 000 000 / (16 × 115200) = 39.0625
 *   整数 39 = 0x27，小数 0.0625 × 16 = 1 → BRR = 0x271
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;                  /* 波特率 */
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;      /* 8 位数据位 */
    huart1.Init.StopBits     = UART_STOPBITS_1;         /* 1 位停止位 */
    huart1.Init.Parity       = UART_PARITY_NONE;        /* 无校验 → 合起来就是 8N1 */
    huart1.Init.Mode         = UART_MODE_TX_RX;         /* 收发都要 */
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;     /* 不用 RTS/CTS 硬件流控 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;    /* 16 倍过采样，对应 27.3.4 节的公式 */
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ================= GPIO：板载 LED ================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();          /* PA9/PA10 的时钟在 MspInit 里也要开 */

    /* PC13 板载 LED，低电平点亮。上电先熄灭 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ================= 串口引脚的复用配置在哪儿 =================
 * PA9/PA10 的 AFIO 配置**不在 main.c 里**，它由 CubeMX 生成在
 * Core/Src/stm32f1xx_hal_msp.c 的 HAL_UART_MspInit() 中，由 HAL_UART_Init 自动调用。
 * 它的作用等价于下面这段（贴在这里只是为了说明"到底配了什么"，
 * 不要在 main.c 里重复实现 HAL_UART_MspInit，否则会和 msp.c 里的定义冲突）：
 *
 *   __HAL_RCC_USART1_CLK_ENABLE();              // USART1 挂在 APB2
 *   __HAL_RCC_GPIOA_CLK_ENABLE();
 *   PA9  = GPIO_MODE_AF_PP,    GPIO_SPEED_FREQ_HIGH   // USART1_TX，复用推挽输出
 *   PA10 = GPIO_MODE_INPUT,    GPIO_NOPULL            // USART1_RX，浮空输入
 *
 * 这三行记住一件事：**TX 是输出、RX 是输入**，所以接线必须 TX↔RX 交叉。
 */

/* ================= 错误处理 ================= */
void Error_Handler(void)
{
    __disable_irq();                        /* 关总中断，让现场"静止"下来便于调试 */
    while (1)
    {
        /* 让 LED 快闪，比一句死循环更容易看出来出事了 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 200000u; i++) { }   /* 忙等，约几十 ms */
    }
}

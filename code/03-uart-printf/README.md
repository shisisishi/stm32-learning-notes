# code/03-uart-printf —— 第 05 章配套工程

USART1 收发最小可用工程：`printf` 重定向 + 中断接收 + 环形缓冲区 + 简单命令解析。
对应笔记：[`docs/05-串口通信.md`](../../docs/05-串口通信.md)

> **状态**：🔬 待上机验证。下面是**预期现象**，我上机跑通后会把实际结果补回来。

## 文件说明

| 文件 | 说明 |
|---|---|
| `main.c` | 主程序。CubeMX 生成框架，`USER CODE` 段和几个回调是我自己写的 |
| `ringbuf.h` | 环形缓冲区，纯 C、不依赖 HAL，只有 `static inline` 函数，可以单独拿到 PC 上编译测试 |

**`ringbuf.h` 必须手动加入 Keil 工程**：复制到 `Core/Inc/` 目录下，或者在 Keil 里右键 `Application/User` → `Add Existing Files` 添加。

## 引脚分配

| 功能 | 引脚 | 外设 | 配置 |
|---|---|---|---|
| 串口 TX | **PA9** | USART1_TX | 复用推挽输出（AF Push-Pull），高速 |
| 串口 RX | **PA10** | USART1_RX | 浮空输入（或上拉输入） |
| 板载 LED | **PC13** | GPIO 输出 | 推挽输出，**低电平点亮**，上电初始为高（熄灭） |
| 调试 | PA13 / PA14 | SWD | ST-Link 下载调试 |

这块板子上的 LED 是接在 PC13 和 VCC 之间的（低电平点亮），所以代码里点亮写 `GPIO_PIN_RESET`、熄灭写 `GPIO_PIN_SET`——和直觉相反，别写反了。

## 串口参数

**115200 8N1**：

| 参数 | 值 | 对应 HAL 字段 |
|---|---|---|
| 波特率 | 115200 bit/s | `huart1.Init.BaudRate` |
| 数据位 | 8 | `UART_WORDLENGTH_8B` |
| 校验位 | 无（None） | `UART_PARITY_NONE` |
| 停止位 | 1 | `UART_STOPBITS_1` |
| 方向 | 收发 | `UART_MODE_TX_RX` |
| 硬件流控 | 无 | `UART_HWCONTROL_NONE` |
| 过采样 | 16 倍 | `UART_OVERSAMPLING_16` |

72 MHz 下 HAL 算出的 **BRR = 0x271**（手算见笔记第 2.3 节）。串口助手上的数据位/校验位/停止位**必须**和这里逐项一致，少对一项就是一屏乱码。

## CubeMX 配置要点

完整的点菜单步骤在笔记第 7.1 节，这里只列最容易漏的几项：

1. **RCC → HSE** 选 `Crystal/Ceramic Resonator`；Clock Configuration 里配到 HCLK = 72 MHz（HSE 8 MHz × PLL9）。
2. **SYS → Debug** 选 `Serial Wire`。
3. **Connectivity → USART1 → Mode** 选 `Asynchronous`，参数按上表填。
4. ⚠️ **NVIC Settings 标签页里勾选 `USART1 global interrupt`**——不勾的话回调永远不会被调用，这是最容易漏的一步。
5. PC13 设为 `GPIO_Output`，`Output Level` = `High`。
6. **Keil → Options for Target → Target → 勾选 `Use MicroLIB`**，否则 `printf` 重定向不生效（Keil 标准库走半主机调用）。

## 串口助手设置

| 项目 | 值 |
|---|---|
| 端口 | USB-TTL 对应的 COM 号（设备管理器里查） |
| 波特率 | 115200 |
| 数据位 / 校验位 / 停止位 | 8 / None / 1 |
| 流控 | None |
| 接收显示 | 文本模式（想看原始字节就切 Hex 模式） |
| 发送 | 文本模式，**勾上"发送新行"** |

## USB-TTL 接线（⚠️ 交叉）

| USB-TTL | STM32F103C8T6 |
|---|---|
| **TX** | **PA10（RX）** |
| **RX** | **PA9（TX）** |
| GND | GND（**必须共地**） |
| 3V3 / 5V | 不接（板子已由 ST-Link 供电，两个电源同时接会打架） |

一句话记住：**TX 接对方的 RX**。接反了的表现是"完全收不到任何数据"，不是乱码。

## 预期输出示例 🔬

上电复位后，串口助手接收区应出现（`\r\n` 在文本模式里就是换行）：

```text
UART1 ready: 115200 8N1
send 'L' = LED on , 'O' = LED off
```

发送 `L`（勾了"发送新行"就会带上 `\r\n`）：

```text
LED ON
```

LED 点亮，`\r\n` 被解析逻辑忽略、不会报错。

发送 `O`：

```text
LED OFF
```

发送 `X`（未定义的命令）：

```text
unknown cmd: 0x58
```

`0x58` 就是字符 `X` 的 ASCII 码。

连续快速发送 200 个 `L`：LED 反复开关，接收区刷出 200 行 `LED ON`，**且不会出现 `!! rx overflow count = ...`**。这个计数只要一直是 0，就说明"中断只搬运、主循环做处理"这套结构是跟得上的。

把串口助手波特率改成 9600：立刻满屏乱码。这是验证"双方约定必须一致"最快的方法。

## 用逻辑分析仪抓波形验证帧格式 🔬

这是把笔记第 2.1 节的时序图**真正对上实物波形**的做法（8 通道 USB 逻辑分析仪 + PulseView 之类的上位机即可）。

**接线**：通道 0 探针接 **PA9（TX）**，逻辑分析仪的 GND 夹子接板子 GND。地一定要共，否则采到的是悬空噪声。

**采样率**：至少 **1 MS/s**。115200 下一位是 8.68 µs，1 MS/s 时一位约 115 个采样点，足够看清位的边界。想量得更准就上 4 MS/s 或 10 MS/s。

**解码器设置**（PulseView：Add decoder → UART）：

| 项 | 值 |
|---|---|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | none |
| Stop bits | 1 |
| Bit order | **LSB first** |
| Idle level | **High** |
| Invert | no |

**该看到什么**（每一项都对着笔记第 2.1 节的表核对）：

| 验证项 | 波形上应该看到 |
|---|---|
| 空闲电平 | 帧与帧之间是一条**平的高电平**线，不是低电平 |
| 起始位 | 从高到低的**下降沿**，宽度 ≈ **8.68 µs** |
| 数据位 | 紧接 8 个 8.68 µs 宽的位，最先出现的是 bit0（LSB） |
| 停止位 | 最后回到高电平，宽度 ≈ **8.68 µs** |
| 一帧总长 | 从起始位下降沿到停止位结束 ≈ **86.8 µs** |
| 相邻帧间隔 | 连续发送时，两个起始位下降沿的间隔 ≈ 86.8 µs 的整数倍 |

**推荐用字符 `'U'`（0x55）做第一个验证字符**：它的数据位是 `0101 0101`，LSB 先发 → 波形上依次是 **高-低-高-低-高-低-高-低**，一段规则的方波，一眼就能数清 8 位。如果 LSB/MSB 顺序被弄反，会变成低-高-低-高……正好反过来，比看乱码直观得多。

启动横幅第一个字符就是 `U`，所以一复位就能抓到。

## 编译与下载

1. CubeMX 按上面的要点配好，生成 `MDK-ARM` 工程。
2. 把 `main.c`、`ringbuf.h` 放进工程（`main.c` 覆盖 CubeMX 生成的那份，或把 `USER CODE` 段的内容粘进去）。
3. Keil：`Options for Target → Target → 勾选 Use MicroLIB`。
4. `Build`（F7），确认 0 Error。
5. ST-Link 接好 SWD（SWDIO / SWCLK / GND / 3V3），`Download`（F8）。
6. 按板子上的复位键，同时看串口助手。

## 没硬件时能在 Wokwi 上跑吗

USART 在 Wokwi 的 STM32F103 仿真里是 ✅ 支持的，**收发逻辑和命令解析可以在仿真里验证**：把串口助手的口接上 Wokwi 的串口监视器，波特率设 115200 就行。

但要注意两点限制：

| 能力 | Wokwi 支持 | 影响 |
|---|---|---|
| GPIO / USART / I2C / SPI / TIM1-4 / EXTI | ✅ | 本章的中断接收、环形缓冲、命令解析都能验证 |
| **DMA** | ❌ 未实现 | 笔记里讲的 `HAL_UART_Transmit_DMA` / DMA + 空闲中断**在仿真里跑不了**，必须上真板 |
| CAN | ❌ 未实现 | RM 的电机通信走 CAN，仿真里练不了 |

另外仿真里**抓不到真实的位时序**——波特率和帧格式这些"物理层"的东西，最终还是要靠逻辑分析仪在真板子上量。

## 已知的坑（详细版见笔记第 6 节）

| 现象 | 最常见原因 |
|---|---|
| 一个字符都收不到 | TX/RX 接反了；或没共地 |
| 满屏乱码 | 两边波特率/数据位/校验位不一致；或系统时钟没配到 72 MHz |
| 只能收到第一个字节 | `HAL_UART_RxCpltCallback` 里没重新调用 `HAL_UART_Receive_IT` |
| 发数据时回调不触发 | CubeMX 的 NVIC 里没勾 `USART1 global interrupt` |
| 发快一点就丢字符 | 中断里做了耗时操作（比如 `printf`），触发 ORE 溢出 |
| `printf` 完全没输出 | Keil 没勾 `Use MicroLIB` |

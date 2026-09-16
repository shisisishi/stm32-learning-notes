# 02-pwm-breath · TIM2 呼吸灯 + TIM3 舵机 PWM

> 第 04 章配套工程：[`docs/04-定时器与PWM.md`](../../docs/04-定时器与PWM.md)
> 芯片：STM32F103C8T6（Blue Pill）｜ 库：HAL ｜ 工具：STM32CubeMX 6.x + Keil MDK-ARM 5.x
> **状态：🔬 待上机验证** —— 代码已写完并通过语法检查，硬件到手后会逐条验证下面写的"预期现象"。

---

## 1. 引脚接线

| 功能 | 引脚 | 外设 | 接线方式 |
|---|---|---|---|
| 外接 LED（呼吸灯） | **PA5** | TIM2_CH1（PWM 输出） | PA5 → 1 kΩ 电阻 → LED 阳极（长脚）；LED 阴极（短脚）→ GND。**高电平点亮** |
| 舵机信号 | **PA6** | TIM3_CH1（PWM 输出） | 橙色信号线 → PA6 |
| 舵机电源 | 独立 5 V | — | 红线 → 5 V 电源正极；棕线 → 电源负极，**并且必须与板子 GND 相连（共地）** |
| 调试器 | PA13 / PA14 | SWD | ST-Link V2 的 SWDIO / SWCLK；另接 GND 和 3.3 V |
| 逻辑分析仪（可选） | CH0 → PA5，CH1 → PA6 | — | **GND 必须与板子共地**，否则测出来的波形没有意义 |

> ⚠️ **两件事别做错**
> 1. **LED 必须串限流电阻**。PA5 直接接 LED 会烧 LED 或烧引脚，限流电阻的算法见[第 03 章](../../docs/03-电路基础-限流与分压.md)。
> 2. **舵机不要从 ST-Link 的 3.3 V 取电**。SG90 空载就要 100 mA 以上，堵转能到 700 mA 左右，ST-Link 供不起，还会把板子电压拉塌。用独立 5 V 电源并且共地。

### 为什么舵机用 PA6 而不是 PA2

仓库统一引脚表里写的是「舵机 PWM = PA2 / TIM2_CH3」，这个工程改成了 **PA6 / TIM3_CH1**，原因：

**同一个定时器的所有通道共享 `PSC`、`ARR`，也就是共享同一个溢出周期。** 呼吸灯要 1 ms 的帧（1 kHz），舵机要 20 ms 的帧（50 Hz），一个 TIM2 给不出两个频率。所以拆成两个定时器：TIM2 管呼吸灯，TIM3 管舵机。

如果把舵机硬放在 PA2（TIM2_CH3）上，只能把整个 TIM2 的时基降到 50 Hz，呼吸灯的载波就跟着变成 50 Hz。详细推导见第 04 章的[第 5.4 节](../../docs/04-定时器与PWM.md)和坑 6。

---

## 2. CubeMX 里每个参数填什么

### 2.1 时钟树（Clock Configuration 页）

| 项目 | 值 | 说明 |
|---|---|---|
| `Input frequency` | **8** MHz | 板上 HSE 晶振 |
| `PLL Source` / `PLLMul` | **HSE** / **×9** | 8 MHz × 9 = **72 MHz** |
| `System Clock Mux` | **PLLCLK** | |
| `HCLK (AHB)` | **72** MHz | |
| `APB1 Prescaler` | **/2** → 36 MHz | ⚠️ 分频系数 ≠ 1，所以定时器时钟要 **×2** |
| `APB2 Prescaler` | **/1** → 72 MHz | |
| **`APB1 Timer clocks`** | **72 MHz** | **把鼠标悬停在这一格确认。** TIM2/TIM3 的实际时钟是 72 MHz，不是 36 MHz |

### 2.2 TIM2 —— 呼吸灯（1 kHz）

| CubeMX 字段 | 填什么 |
|---|---|
| `Clock Source` | Internal Clock |
| `Channel1` | **PWM Generation CH1**（→ PA5） |
| `Prescaler (PSC - 16 bits value)` | **71** |
| `Counter Mode` | Up |
| `Counter Period (AutoReload Register)` | **999** |
| `Clock Division` | No Division |
| `Auto-reload preload` | Disable |
| `Mode` | PWM mode 1 |
| `Pulse (16 bits value)` | **0** |
| `Output compare preload` | Disable |
| `Fast Mode` | Disable |
| `CH Polarity` | High |

**这些数字是怎么来的（完整算式）**

```text
定时器时钟 f_CK_PSC = 36 MHz × 2 = 72 MHz        （APB1 分频系数 = 2，硬件倍频 ×2）

第一步 定分辨率：想要 1000 级占空比
       ARR + 1 = 1000            →  ARR = 999

第二步 反推预分频（两个分频系数的乘积 = 定时器时钟 ÷ 目标频率）：
       (PSC + 1) = f_CK_PSC / f_目标 / (ARR + 1)
                 = (72 000 000 Hz / 1000 Hz) / 1000
                 = 72 000 / 1000 = 72      →  PSC = 71

验算   f = 72 000 000 / (71 + 1) / (999 + 1)
         = 72 000 000 / 72 / 1000
         = 1 000 000 / 1000 = 1000 Hz = 1 kHz      ✅ 周期 = 1 ms
       f_CK_CNT = 72 MHz / 72 = 1 MHz → 1 个计数 = 1 µs
       占空比 = CCR / 1000
```

### 2.3 TIM3 —— 舵机（50 Hz）

| CubeMX 字段 | 填什么 |
|---|---|
| `Clock Source` | Internal Clock |
| `Channel1` | **PWM Generation CH1**（→ PA6） |
| `Prescaler (PSC - 16 bits value)` | **71** |
| `Counter Mode` | Up |
| `Counter Period (AutoReload Register)` | **19999** |
| `Clock Division` | No Division |
| `Auto-reload preload` | Disable |
| `Mode` | PWM mode 1 |
| `Pulse (16 bits value)` | **1500** |
| `Output compare preload` | Disable |
| `Fast Mode` | Disable |
| `CH Polarity` | High |

**这些数字是怎么来的**

```text
第一步 定分辨率：舵机控制脉宽只有 0.5~2.5 ms，要有 1 µs 的细分
       f_CK_CNT = 1 MHz  →  (PSC + 1) = 72  →  PSC = 71

第二步 定周期：50 Hz → T = 1 / 50 = 0.02 s = 20 ms = 20 000 µs
       ARR + 1 = 20 000          →  ARR = 19999

验算   f = 72 000 000 / (71 + 1) / (19 999 + 1)
         = 72 000 000 / 72 / 20 000
         = 1 000 000 / 20 000 = 50 Hz             ✅ 周期 = 20 ms
       位宽检查：ARR = 19999 < 65535 ✅ 16 位装得下
```

**角度 → `CCR`（1 个计数 = 1 µs，所以 `CCR` 的数值就是高电平的微秒数）**

| 角度 | 高电平 | 占空比 | `CCR` |
|---|---|---|---|
| 0° | 0.5 ms | 2.5% | 500 |
| 90° | 1.5 ms | 7.5% | 1500 |
| 180° | 2.5 ms | 12.5% | 2500 |

公式：`CCR = 500 + 角度 × 2000 / 180 = 500 + 角度 × 100 / 9`（代码里用整数算，见 `App_Servo_AngleToCcr()`）

### 2.4 生成代码时的选项

- `Toolchain / IDE` = **MDK-ARM**，版本 **V5**
- `Code Generator` 页勾上 **`Generate peripheral initialization as a pair of .c/.h files per peripheral`**（让 `MX_TIM2_Init()` / `MX_TIM3_Init()` 放进 `tim.c`，结构清楚）
- 我写的代码全部放在 `/* USER CODE BEGIN xxx */` 和 `/* USER CODE END xxx */` 之间，**否则下次在 CubeMX 里改配置重新生成会被覆盖**

---

## 3. 代码结构

`main.c` 里我写的部分：

| 函数 | 干什么 |
|---|---|
| `App_BreathLed_Init()` | 呼吸灯初始化，`CCR = 0`（LED 灭） |
| `App_BreathLed_Task()` | 每 50 ms 取下一个查表值写入 `CCR1`，40 段 × 50 ms = 2.00 s 一个呼吸周期 |
| `App_Servo_AngleToCcr()` | 角度 → `CCR`，整数运算 `500 + angle × 100 / 9` |
| `App_Servo_Init()` | 舵机初始化，摆到 90° |
| `App_Servo_Task()` | 每 20 ms 让角度走 1°，到 0°/180° 反向，单程 3.60 s |

呼吸灯亮度表 `g_breath_lut[41]`：三角波，`CCR` 从 0 升到 998（1.00 s）再降回 0（1.00 s）。
**最大值取 998 而不是 999（= `ARR`）：`CCR > ARR` 时 `CNT` 永远追不上 `CCR`，输出会贴死在常高电平，亮度会"卡"在最亮不动。**

启动 PWM 的两行必须写在 `main()` 里，不能省：

```c
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* PA5 */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PA6 */
```

`MX_TIMx_Init()` 只是把寄存器配好，`CCER` 里的通道使能位还是 0；不调 `HAL_TIM_PWM_Start()`，配置全对但引脚上一个波形也没有。

---

## 4. 预期现象 🔬

> **以下全部是预期现象，我还没上机实测。** 硬件到手后会逐条验证并回来改成实测值。

### 4.1 肉眼可见

| 现象 | 预期 |
|---|---|
| PA5 的外接 LED | 平滑地由暗变亮、再由亮变暗，一个完整周期 **2.00 s**（用秒表量两次"最亮"之间应该是 2.00 s） |
| PA6 上的舵机 | "啪"地转到 90° 稳住，然后缓慢地从 0° 扫到 180° 再扫回来，**单程 3.60 s**，全程不抖 |
| 板载 PC13 LED | 本工程没动它，保持上电默认状态 |

### 4.2 逻辑分析仪 / 示波器应该量到

| 探针 | 预期读数 | 不对时先查什么 |
|---|---|---|
| **PA5** | 频率 **1.000 kHz**（周期 **1.000 ms**） | 频率是 2 倍或一半 → 定时器时钟算错（`APB1 Timer clocks` 那一格） |
| PA5 占空比 | 从 **0%** 线性升到 **99.8%**（`CCR = 998`，即 `998/1000`）再降回 0%，变化周期 2.00 s | 频率对但占空比不动 → `__HAL_TIM_SET_COMPARE()` 没生效 |
| PA5 平均电压 | 0 V ~ **3.29 V** 之间线性变化（`998 / 1000 × 3.3 V`） | 恒定 3.3 V 且无跳变 → `CCR` 超出了 `ARR` 范围 |
| **PA6** | 频率 **50.00 Hz**（周期 **20.00 ms**） | 量到 1 ms 或 500 Hz → 时基配错了 |
| PA6 高电平宽度 | 在 **0.5 ms ~ 2.5 ms** 之间按三角波变化 | 脉宽超出这个范围 → 角度换算或限幅有问题 |

**用示波器验证公式对不对**（最直接的一步）：在 `PSC = 71, ARR = 999` 下，`CCR = 500` 应该对应高电平 **500 µs**、占空比 **50%**。先把 `App_BreathLed_Task()` 暂时改成写死一个值，量到 500 µs 就说明整条换算链是对的。

### 4.3 没有仪器怎么验证

- 呼吸灯：秒表量周期，只能验证数量级，量不出 1 kHz。
- 舵机：先把角度写死在上电位置（比如只调 `App_Servo_AngleToCcr(90)`），上电后舵机应该稳住不动。**如果它一直抖，先查电源和共地，不要先怀疑代码。**

### 4.4 Wokwi 能不能仿真

Wokwi 的 STM32F103C8 仿真**支持 TIM1~TIM4**，所以 PWM 波形可以在里面跑起来看。但 Wokwi 里没有"舵机"这个元件能直接看角度变化，只能看波形。

⚠️ Wokwi 里**未实现**的外设：**CAN / DMA / RTC / IWDG / PWR**（本章用不到）。ADC 只实现了 ADC1 的基础转换。

---

## 5. 已知的坑（详见第 04 章第 6 节）

| # | 坑 | 一句话原因 |
|---|---|---|
| 1 | 定时器时钟算错 | APB1 分频 ≠ 1 时 `TIMxCLK` = APB1 × 2 = 72 MHz，不是 36 MHz |
| 2 | 忘了 `HAL_TIM_PWM_Start()` / `HAL_TIM_MspPostInit()` | 只配了寄存器，`CC1E` 还是 0 / 引脚还是浮空输入 |
| 3 | 舵机频率给成 1 kHz | 舵机测的是脉宽不是占空比，1 ms 一条指令会让它堵转发热 |
| 4 | 改 `CCR` 没等重装载 | 值写到了"已经过期"的那一拍，要等下一个周期才生效 |
| 5 | 占空比给到 100% | `CCR > ARR` 时 `CNT` 追不上，输出贴死在常高电平，看起来像"没输出" |
| 6 | 以为 `CCR > ARR` 能让一个通道变低频 | `CNT` 每个周期都从 0 重来，到不了的数就永远翻不过去。**一个定时器只有一个时基** |

---

## 6. 编译与烧录

1. Keil 打开 `MDK-ARM/02-pwm-breath.uvprojx`
2. **F7** 编译，确认 `0 Error(s)`
3. 魔术棒 → `Debug` 页 → 选 **`ST-Link Debugger`** → `Settings` → `Port` 选 **SW**
4. `Flash Download` 页勾上 **`Reset and Run`**（烧完自动运行）
5. **F8** 下载

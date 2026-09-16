# 04 · 定时器与 PWM：把呼吸灯讲到能自己算出频率

> **本章目标**：拿到任意一个想要的 PWM 频率和分辨率，能自己反推出 `PSC` 和 `ARR` 该填多少；能用定时器做出呼吸灯，并给舵机输出标准的 50 Hz 控制信号。
> **前置章节**：[02 GPIO 与点灯](02-GPIO与点灯.md) · [03 电路基础（一）](03-电路基础-限流与分压.md)
> **硬件**：STM32F103C8T6 最小系统板、ST-Link V2、面包板、外接 LED（配 1 kΩ 限流电阻）、SG90 舵机（可选）、逻辑分析仪或示波器
> **代码**：[`code/02-pwm-breath`](../code/02-pwm-breath)
> **状态**：🔬 待上机验证

---

## 1. 先想清楚一个问题

我一开始是这么想的：板子上那个 LED 我会点了，`HAL_GPIO_WritePin()` 一高一低，再配 `HAL_Delay(500)`，就是 1 Hz 闪烁。

但马上卡住了。我要的是**呼吸灯**——亮度平滑地亮起来、再暗下去，不是"啪"一下亮、"啪"一下灭。而 `HAL_GPIO_WritePin()` 只有高和低两个状态，它没法表达"半亮"。

那么"半亮"到底是什么？我盯着电路想了一会儿：LED 串了限流电阻接在 3.3 V 上，导通时电流是恒定的，"半亮"不是把电压减半，而是**在一段时间里，让它亮一半的时间、灭一半的时间**。只要这个切换够快（超过人眼的闪烁融合频率），看到的就只是一个偏暗的 LED。

这个"快速开关、通过开关比例控制平均功率"的思路，就是**脉冲宽度调制（Pulse Width Modulation, PWM）**。

接下来我算了一下：我想让 LED 每 1 ms 亮一次。这句话翻译成参数就是——周期 1 ms，频率 1 kHz。然后我打开了 CubeMX 的 TIM2 配置页，看到两个框：

```text
Prescaler (PSC - 16 bits value)          [ 0 ]
Counter Period (AutoReload Register)     [ 65535 ]
```

我当时完全不知道这两个数字是干什么的，就往里填了默认值。烧进去以后 LED 是暗的，只有一点点微微的光。

问题就变成了这一章要解决的事：

> **我想让输出周期是 1 ms，这两个框到底该填什么数？它们和 72 MHz 之间是什么关系？**

等到把这一章算完，我不光知道 PSC 该填 71、ARR 该填 999，还知道为什么不是 35 和 19999，也能反着来：给一个舵机需要的 50 Hz，我能推出 `PSC = 71、ARR = 19999`，并且能说清 0.5 ms 的脉宽对应 `CCR = 500`。

---

## 2. 原理：它到底是怎么工作的

### 2.1 先把 STM32F103 的定时器全貌看一遍

我以前以为"定时器"就是一个，配置一下就好。翻 RM0008 第 15 章开头那张 TIMx 功能对比表才发现，F103 上有一堆定时器，能力完全不同：

| 定时器 | 类别 | 计数位宽 | 通道数 | 能干什么 |
|---|---|---|---|---|
| **TIM1** | 高级控制定时器（advanced-control） | 16 位 | 4 个独立通道，其中 CH1/CH2/CH3 各有互补输出 | 带**死区插入（dead-time）**、**刹车输入（break）**、重复计数器、编码器接口；专门为**三相电机驱动**这类需要互补输出 + 死区保护的场合设计 |
| **TIM8** | 高级控制定时器 | 16 位 | 同 TIM1 | 同 TIM1（**F103C8T6 上没有**，是 100 脚以上型号才有的） |
| **TIM2 / TIM3 / TIM4 / TIM5** | 通用定时器（general-purpose） | 16 位 | 各 4 个独立通道 | 输入捕获、输出比较、PWM 输出、单脉冲、编码器接口、触发 ADC/DAC |
| **TIM6 / TIM7** | 基本定时器（basic） | 16 位 | **0 个通道** | 只能产生时基和更新中断/事件，主要用来触发 DAC、做纯计时；**没有任何引脚可以输出 PWM** |

关于 F103C8T6 具体有哪些，一个容易记错的点：**它是 48 脚的，TIM5 和 TIM8 都不存在**。所以实际能用的定时器是 TIM1、TIM2、TIM3、TIM4、TIM6、TIM7。基本定时器 TIM6/TIM7 没有输出通道，这次做 PWM 用不上。

这里我要特意记一笔，因为它和我要报的方向直接相关：

> **战队机器人上驱动无刷电机用的就是高级定时器的互补输出 + 死区。**
> 原因是无刷电机的一个桥臂由上下两个 MOS 管组成（半桥），这两个管子如果同时导通，电源就被直接短路了，叫**直通（shoot-through）**，会烧管子。所以必须保证"上管关断之后，下管才允许导通"，中间留一段两路都关断的死时间——**死区时间（dead time）**。这个延迟不能靠软件算，软件几十 ns 的抖动就够出事，必须由定时器硬件保证。TIM1 的 `BDTR` 寄存器里的 `DTG[7:0]` 就是配死区时间的（RM0008 第 15.4.18 节）。
>
> 我这一章先用**通用定时器**做 PWM（它最简单、最能看清 PSC/ARR/CCR 三者的关系）。等时基单元彻底算明白了，再去看 TIM1 的互补输出会顺很多。**这一章的代码里没有用 TIM1，我不要在报名材料里把"会用 TIM1 互补输出"写成已完成的事。**

TIM2 和 TIM3 的通道引脚在我这块板子上的分配：

| 定时器 | 通道 | 引脚 | 我用来做什么 |
|---|---|---|---|
| TIM2 | CH1 | **PA5** | 外接 LED，做呼吸灯 |
| TIM2 | CH2 | PA1 | 不用（第 07 章 PA1 用作 ADC1_IN1） |
| TIM2 | CH3 | PA2 | 不用（踩坑时用过，见第 6 节坑 6） |
| TIM2 | CH4 | PA3 | 不用 |
| TIM3 | CH1 | **PA6** | 舵机 PWM |
| TIM3 | CH2 | PA7 | 不用 |
| TIM3 | CH3 | PB0 | 不用 |
| TIM3 | CH4 | PB1 | 不用 |

这里有一条必须先说清楚的限制：**同一个定时器的所有通道共享同一个时基单元**，也就是共享同一个 `PSC` 和同一个 `ARR`。所以同一个定时器的四个通道，PWM **频率一定相同**，能各自独立设的只有 `CCR`（各自的占空比）。

这条限制不是理论——我一开始想把呼吸灯和舵机都塞进 TIM2 的两个通道，被它结结实实挡了一次，过程写在[坑 6](#坑-6这个坑是我自己推理推错的记下来最有价值以为-ccr--arr-能让一个通道变成低频)。最后的方案是**呼吸灯用 TIM2（1 kHz），舵机用 TIM3（50 Hz）**，一个定时器干一件事。

### 2.2 时基单元：三个寄存器怎么配合

时基单元（time-base unit）就三个东西：`PSC`、`CNT`、`ARR`。按 RM0008 第 15.3.1 节，它的结构是这样的：

```text
                CK_PSC               CK_CNT
  72 MHz ──────────────►[ 预分频器 PSC ]──────────────►[ 计数器 CNT ]
                          ÷(PSC+1)                      ↑       │
                                                         │       │ 数到 ARR
                                                         │       ▼
                                                   [ 自动重装载寄存器 ARR ]
                                                         ↑
                                                         └── 溢出时自动重装
```

一步步说：

1. **`CK_PSC`**：定时器的输入时钟。TIM2/TIM3 挂在 APB1 上，本章配置下 `CK_PSC = 72 MHz`（为什么是 72 而不是 36，见 2.3）。
2. **预分频器 `PSC`**：把输入时钟除以 `(PSC + 1)`，得到真正驱动计数器的时钟 `CK_CNT`。
   注意是 **`PSC + 1`**，因为 `PSC = 0` 表示"不分频"。填 71 才是除以 72。
3. **计数器 `CNT`**：在 `CK_CNT` 的每个上升沿加 1。边沿对齐（向上计数）模式下它从 0 数到 `ARR`，然后**溢出**。
4. **溢出时发生两件事**：`CNT` 自动清 0 并从 `ARR` 重新装载；同时产生一个**更新事件（Update Event, UEV）**，把更新标志位 `UIF` 置 1。

一个"从 0 数到 ARR 再溢出"的循环，就是这个定时器的一个周期。所以：

```text
                 (ARR + 1)
   T_周期 = ───────────────────
              f_CK_CNT

                 f_CK_CNT        f_CK_PSC
   f_周期 = ─────────────── = ───────────────────
              (ARR + 1)        (PSC+1) × (ARR+1)
```

**为什么是 `ARR + 1` 而不是 `ARR`**：计数器从 0 开始数，`ARR = 999` 时它经过的状态是 0、1、2、…、999，一共 **1000** 个计数。所以 `ARR = 999` 对应的分频系数是 1000。

这个式子是整章的核心，后面所有计算都从它出发：

> **⭐ `f_CNT = f_CK_CNT / ((PSC + 1) × (ARR + 1))`**
>
> 反过来，要凑一个目标频率的时候，把两个分频系数看成一个整体：
> **`(PSC + 1) × (ARR + 1) = f_CK_PSC / f_目标`**
> （右边是 Hz ÷ Hz，约掉之后就是"总共要分多少倍"，没有单位问题。如果写成 `f_CK_PSC` 除以某个中间频率，量纲就乱了。）

### 2.3 ⭐ 那个害我算错一整轮的"定时器时钟倍频"

这是本章最关键、也最容易错的一节。

我当时的推理是这样的：

> "系统时钟 72 MHz，APB1 分频系数是 2，所以 APB1 总线时钟 = 72 / 2 = **36 MHz**。TIM2 挂在 APB1 上，所以 TIM2 的时钟也是 **36 MHz**。"
>
> "那我要 1 MHz 的计数时钟，就填 `PSC = 36 - 1 = 35`。"

前两句对，第三句错了。**实际 TIM2 的时钟是 72 MHz，不是 36 MHz。**

正确的规则写在 RM0008 第 7 章（RCC，复位和时钟控制）关于 APB 预分频的那张对应关系表里，意思是：

> 如果 APB 预分频系数为 1，则定时器时钟 `TIMxCLK` 等于 APB 时钟；
> **否则（APB 预分频系数 ≠ 1），定时器时钟 = APB 时钟 × 2。**

也就是：

| APB 预分频系数 | APB1 时钟 | TIM2~TIM7 的时钟 `TIMxCLK` |
|---|---|---|
| 1 | 36 MHz（如果 HCLK = 36 MHz） | **等于 APB1，不倍频** |
| **2** | **36 MHz** | **72 MHz** ← 我的配置 |
| 4 | 18 MHz | 36 MHz |
| 8 | 9 MHz | 18 MHz |
| 16 | 4.5 MHz | 9 MHz |

我配置的是 **APB1 预分频系数 = 2**，`APB1 = 36 MHz`，但因为系数不等于 1，**TIM2/TIM3 的时钟被硬件自动 ×2 变成 72 MHz**。TIM1 挂在 APB2 上，规则完全一样：APB2 分频系数为 1 时 TIM1 用 APB2 时钟（我的配置下是 72 MHz），分频时同样 ×2。

**ST 为什么要这么设计？** 我的理解是：定时器需要比总线更高的计数精度，而 APB 为了照顾低速外设常被分频降速。这个"倍频补偿"保证只要系统时钟是 72 MHz，定时器总能拿到 72 MHz，不用为了定时器精度去迁就总线的分频。

**怎么在 CubeMX 里确认（这比背结论靠谱）**：打开 CubeMX 的 **Clock Configuration** 页，把鼠标悬停在时钟树最右边那一列 `APB1 Timer clocks` 上，它会直接显示 **72 MHz**。我第一次发现算错，就是把这个数字和草稿纸上写的 36 MHz 对了一下。

**这个错会导致什么（我把自己的数写出来）**：

- 我以为 `f_CK_CNT = 36 MHz`，填 `PSC = 35`，想得到 `36 MHz / 36 = 1 MHz`；再填 `ARR = 999`。
- 如果时钟真是 36 MHz，结果是 `36 MHz / (35+1) / (999+1) = 1000 Hz`，正好 1 kHz。
- 但时钟实际是 72 MHz，于是实际频率是 `72 MHz / 36 / 1000 = 2000 Hz`，也就是 **2 kHz，周期 0.5 ms——正好是我想要的两倍**。
- **反推**：如果我要 1 kHz 却测到 2 kHz，说明真实的 `f_CK_CNT` 是我以为的 2 倍，也就是 72 MHz。频率翻倍这个现象本身就是"倍频规则"的指纹。

> **一句话总结**：APB1 预分频系数 ≠ 1 时，`TIM2~TIM7 的时钟 = APB1 时钟 × 2`。我的配置下是 **72 MHz**。算 PWM 频率之前，先去 Clock Configuration 页把 `APB1 Timer clocks` 那一格看清楚。

### 2.4 三个算例，全部验算

有了核心公式，下面三个是我实际配到工程里的参数，每个都正着算一遍、再验算一遍。

#### 算例 A：1 kHz PWM，占空比分辨率 1000 级（呼吸灯用）

要求 `f = 1 kHz`（周期 1 ms），占空比能分成 1000 个等级（也就是能表达 0.1% 的占空比）。

选 `ARR` 时有个直接关系：**边沿对齐 PWM 的占空比等级数 = `ARR + 1`**。想要 1000 级：

```text
ARR = 1000 - 1 = 999
```

再由核心公式反推 `PSC + 1`：

```text
(PSC+1) × (ARR+1) = f_CK_CNT / f_目标 = 72 000 000 Hz / 1000 Hz = 72 000
PSC + 1 = 72 000 / 1000 = 72
PSC = 71
```

于是 `f_CK_CNT = 72 MHz / (71+1) = 1 MHz`，一个计数 = **1 µs**。

**验算**：

```text
f = 72 000 000 / (71 + 1) / (999 + 1)
  = 72 000 000 / 72 / 1000
  = 1 000 000 / 1000
  = 1000 Hz          ✅ 周期 = 1 / 1000 s = 1 ms
```

**顺带一个很好用的副产品**：因为 `f_CK_CNT = 1 MHz`、1 个计数 = 1 µs，**`CCR` 的数值正好等于高电平的微秒数**。`CCR = 500` 就是高电平 500 µs、占空比 50%。这个巧合后面调舵机时省了不少事。

#### 算例 B：50 Hz 舵机 PWM（周期 20 ms）

舵机要 50 Hz，也就是周期 20 ms，分辨率要做到 1 µs（舵机的控制脉宽区间只有 0.5~2.5 ms，必须有 1 µs 级的细分）。

先定分辨率 —— 让一个计数等于 1 µs：

```text
(PSC + 1) = 72 000 000 / 1 000 000 = 72
PSC = 71            （f_CK_CNT = 1 MHz，1 个计数 = 1 µs）
```

再定周期 —— 20 ms = 20000 µs，需要 20000 个计数：

```text
ARR + 1 = 20 000
ARR = 19 999
```

**验算**：

```text
f = 72 000 000 / (71 + 1) / (19 999 + 1)
  = 72 000 000 / 72 / 20 000
  = 1 000 000 / 20 000
  = 50 Hz            ✅ 周期 = 1 / 50 = 0.02 s = 20 ms
```

（另一个记法：`(PSC+1) × (ARR+1)` 必须等于 `72 000 000 / 50 = 1 440 000`。`72 × 20000 = 1 440 000` ✅）

舵机角度与 `CCR` 的对应，因为 1 个计数 = 1 µs，可以直接换算：

| 角度 | 高电平宽度 | 占空比 | `CCR` |
|---|---|---|---|
| 0° | 0.5 ms = 500 µs | 500 / 20000 = 2.5% | 500 |
| 45° | 1.0 ms = 1000 µs | 5.0% | 1000 |
| 90° | 1.5 ms = 1500 µs | 7.5% | 1500 |
| 135° | 2.0 ms = 2000 µs | 10.0% | 2000 |
| 180° | 2.5 ms = 2500 µs | 12.5% | 2500 |

线性关系：`CCR = 500 + 角度 / 180 × 2000`，反过来 `角度 = (CCR - 500) × 180 / 2000`。

**为什么舵机的占空比只有 2.5%~12.5% 这么小？** 因为它和 LED 不一样。LED 用的是**平均功率**，占空比直接决定亮度；舵机用的是**脉宽本身携带的信息**——舵机内部电路测量的是那一段高电平持续了多久，再把这个时长翻译成目标角度。占空比是多少根本不重要，重要的是高电平有多宽。这一条我在第 8 节 Q2 里展开说。

#### 算例 C：1 Hz（周期 1 s）的 LED 闪烁

直接让总计数等于 72000000：

```text
(PSC + 1) × (ARR + 1) = 72 000 000 / 1 = 72 000 000
取 PSC + 1 = 7200  →  PSC = 7199   （f_CK_CNT = 72 MHz / 7200 = 10 kHz，1 个计数 = 100 µs）
取 ARR + 1 = 10000 →  ARR = 9999
```

**验算**：

```text
f = 72 000 000 / (7199 + 1) / (9999 + 1)
  = 72 000 000 / 7200 / 10000
  = 10 000 / 10 000
  = 1 Hz             ✅ 周期 = 1 s
```

**三个算例放在一起，能看出一个取舍**：同样是分频，`PSC` 和 `ARR` 谁大谁小是有讲究的。

- `ARR` 越大 → 占空比可分的等级越多（`= ARR + 1`），但 `ARR` 上限是 **65535**（16 位）。
- 目标频率越低、分辨率要求越高，`(PSC+1) × (ARR+1)` 这个乘积就越大，早晚会撑爆 16 位的 `ARR`。
- 所以顺序应该是：**先用分辨率要求定 `ARR`（不超过 65535），再把剩下的分频量丢给 `PSC`**（`PSC` 也是 16 位，最大 65535）。

整理成表，这几个数以后直接抄：

| 用途 | 目标频率 | 周期 | `PSC` | `ARR` | `f_CK_CNT` | 分辨率 | 验算频率 |
|---|---|---|---|---|---|---|---|
| 呼吸灯（TIM2） | 1 kHz | 1 ms | 71 | 999 | 1 MHz（1 µs/计数） | 1000 级 | `72 M/72/1000 = 1000.0 Hz` |
| 舵机（TIM3） | 50 Hz | 20 ms | 71 | 19999 | 1 MHz（1 µs/计数） | 20000 级 | `72 M/72/20000 = 50.00 Hz` |
| LED 闪烁 | 1 Hz | 1 s | 7199 | 9999 | 10 kHz（100 µs/计数） | 10000 级 | `72 M/7200/10000 = 1.000 Hz` |
| 电机驱动常见载频 🔬 | 20 kHz | 50 µs | 3 | 899 | 18 MHz（≈55.6 ns/计数） | 900 级 | `72 M/4/900 = 20000.0 Hz` |

（最后一行我还没上机测，标 🔬。20 kHz 是电机驱动常用的载波频率，因为它刚好在人耳听觉上限附近，可以避免电机线圈发出啸叫。）

### 2.5 `CCR` 与占空比：PWM 模式 1 和模式 2

前面一直在说 `ARR` 定频率，那**占空比是谁定的？** 是 `CCR`（Capture/Compare Register，捕获/比较寄存器）。

时基单元数数的过程中，硬件会不停地拿 `CNT` 和 `CCR` 比较，比较结果直接驱动输出引脚：

```text
PWM 模式 1（OCxM = 110），高电平有效（CCxP = 0），ARR = 9，CCR = 4

CNT  │0  1  2  3  4  5  6  7  8  9 │ 0  1  2 ...
     │  ←── CNT < CCR ──→│← CNT ≥ CCR →│
OUT  │▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁│▔▔▔▔▔▔▔
     │    高电平（有效）      低电平（无效） │
     │←──── ARR + 1 = 10 个计数 = 一个周期 ────→│
```

规则说清楚（RM0008 第 15.3.9 节「PWM 模式」）：

| 模式 | `OCxM` 编码 | `CNT < CCR` 时 | `CNT ≥ CCR` 时 | 高电平有效（`CCxP=0`）时，高电平出现在 |
|---|---|---|---|---|
| **PWM 模式 1** | 110 | **有效**电平 | 无效电平 | `CNT < CCR` 的这段时间 |
| **PWM 模式 2** | 111 | **无效**电平 | 有效电平 | `CNT ≥ CCR` 的这段时间（和模式 1 恰好互补） |

在**边沿对齐、向上计数、PWM 模式 1、高电平有效**这组最常见的设置下（也是我这一章的配置）：

```text
占空比 = CCR / (ARR + 1)

高电平时间 = CCR × (PSC + 1) / f_CK_PSC = CCR / f_CK_CNT
```

第二个式子在"1 个计数 = 1 µs"的配置下特别好用：`高电平(µs) = CCR` 的数值。

关于模式 1 / 模式 2，有个我一开始觉得别扭、后来想通了的地方：

> 模式 1 和模式 2 的差别，本质是"`CNT < CCR` 时输出有效还是无效"。
> 但"有效电平"具体是高还是低，还由 `CCxP` 位单独决定（`CCxP = 0` 高有效，`CCxP = 1` 低有效）。
> **所以这四种组合里，有两种做出"占空比越大越亮"、两种做出"越大越暗"，效果上等价于反相。**
> 我在代码里用**模式 1 + 高电平有效**，`CCR` 越大高电平越长；而我的外接 LED 是 PA5 → 电阻 → LED → GND，**高电平点亮**，所以 `CCR` 越大 LED 越亮——这一点我在注释里专门标了，免得以后自己看混。

### 2.6 输出比较 vs PWM 模式：它们不是一回事

CubeMX 里 TIM 通道的模式下拉框有 `Output Compare CH1` 和 `PWM Generation CH1` 两个选项，我一开始不知道选哪个。翻手册才明白，这两个虽然共用 `CCR` 和同一套比较器，行为完全不同：

| | 输出比较（Output Compare） | PWM 模式（PWM Generation） |
|---|---|---|
| 触发动作 | **只在 `CNT == CCR` 的那一个计数时刻**改变引脚（翻转/置高/置低），一个周期只动一次 | 硬件**持续**按 `CNT` 与 `CCR` 的比较结果驱动引脚，`CNT` 一复位电平就跟着回来 |
| 一个周期内的波形 | 只有一次跳变，要自己在下一次更新中断里改 `CCR` 才能凑出连续波形 | 自动就是"高一段、低一段"，一直重复 |
| 需要 CPU 参与吗 | **需要**，每个周期都要进中断改状态或改 `CCR` | **不需要**，配置 + 启动之后自己跑 |
| 典型用途 | 单脉冲输出、精确的定时翻转、软件生成任意波形 | 呼吸灯、舵机、电机调速 |

一句话区分：**输出比较是"到点了通知一下（并动一次）"，PWM 模式是"这一整个周期里，前面一段高、后面一段低，硬件负责重复"。**

**那为什么做 LED 和电机一定要用 PWM 模式，不能用软件延时？**

我一开始写过软件 PWM，思路是 `HAL_GPIO_WritePin(...1); delay_us(500); HAL_GPIO_WritePin(...0); delay_us(500);` 循环。它有几个我当时没想到的问题：

| 问题 | 软件延时 / 软件 PWM | 硬件 PWM |
|---|---|---|
| **CPU 占用** | 100%。`HAL_Delay()` 是空转等待，这段时间 CPU 什么也干不了。程序里如果还要读 ADC、刷 OLED、跑 PID，全被拖慢 | **0%。** `HAL_TIM_PWM_Start()` 执行完之后，波形由定时器硬件产生，**CPU 可以完全不管它** |
| **抖动（jitter）** | 大。`HAL_Delay()` 基于 SysTick 中断，中断随时可能被更高优先级的中断（串口、ADC）打断，每次延时会多出几到几十 µs 的误差，而且每次不一样 | 由 72 MHz 时钟直接分频得到，误差只来自晶振本身（我用的外部 8 MHz 晶振经 PLL ×9），周期稳定 |
| **时间精度** | 受 `HAL_Delay()` 的 1 ms 最小粒度、`delay_us` 循环次数换算误差影响 | 由 `PSC`/`ARR` 精确决定，1 µs 很轻松 |
| **丢波** | 一旦有中断或长任务插进来，某一拍的宽度就变形；对舵机来说这意味着"指令串了" | 定时器独立计数，波形不受主循环影响 |

对一个机器人来说这个差别是决定性的：**电机 PWM 必须一直稳定输出，同时 CPU 还要跑姿态解算和 PID。** 这两件事只能是"硬件输出波形 + CPU 算控制量"，不能是"CPU 一边数时间一边输出波形"。

---

## 3. 手册依据

这一章的结论都是从下面这些文档查的，不是我猜的：

| 结论 | 出处 |
|---|---|
| TIM1/TIM2~5/TIM6~7 的分类、计数位宽、通道数、各自功能（含互补输出、死区、刹车） | **RM0008（STM32F10xxx 参考手册）第 15 章 "General-purpose timers (TIMx)" 章首的定时器功能对比表** |
| 时基单元的构成、`PSC` 的 `+1` 语义、`CNT` 溢出产生更新事件 `UEV`、`ARR` 的自动重装载 | **RM0008 第 15.3.1 节 "Time-base unit"** |
| 预分频器、计数器、自动重装载寄存器的位宽与写入行为（含影子寄存器 / `ARPE` 预装载） | **RM0008 第 15.4.3 节 "TIMx prescaler (TIMx_PSC)"、第 15.4.4 节 "TIMx auto-reload register (TIMx_ARR)"、第 15.4.1 节 "TIMx control register 1 (TIMx_CR1)"**（寄存器映射小节） |
| PWM 模式 1 / 模式 2 的 `CNT` 与 `CCR` 比较规则、边沿对齐下的占空比 | **RM0008 第 15.3.9 节 "PWM mode"** |
| 输出比较模式与 PWM 模式的区别、`OCxM` 位编码、输出比较预装载 `OCxPE` | **RM0008 第 15.3.7 节 "Output compare mode"、第 15.4.7 节 "TIMx capture/compare mode register 1 (TIMx_CCMR1)"** |
| 输出极性 `CCxP`、通道使能 `CCxE`、互补输出与死区时间 `DTG` | **RM0008 第 15.4.9 节 "TIMx capture/compare enable register (TIMx_CCER)"、第 15.4.18 节 "TIMx break and dead-time register (TIMx_BDTR)"**（高级定时器才有 BDTR） |
| **APB 预分频系数为 1 时 `TIMxCLK` = APB 时钟，否则 = APB 时钟 × 2** | **RM0008 第 7 章 "Reset and clock control (RCC)"** 里关于 `APB1/APB2` 预分频与定时器时钟的那张对应关系表 |
| 系统时钟 72 MHz 的配置（HSE 8 MHz × PLL9）、APB1 = 36 MHz、APB2 = 72 MHz | RM0008 第 7 章 RCC；CubeMX 的 Clock Configuration 页 `APB1 Timer clocks` 那一格显示 72 MHz |
| 舵机 50 Hz / 0.5~2.5 ms 脉宽 | 舵机厂家的数据手册。我手上这个 SG90 的标称是 50 Hz、0.5~2.5 ms 对应 0~180°。**具体数值要以自己买的那个舵机的手册为准**，不同型号的行程和脉宽区间会有差别 |
| F103C8T6 上 TIM5 / TIM8 不存在 | STM32F103x8/xB 数据手册的定时器特性列表（48 脚封装的引脚与外设数量限制） |

> **一条我记下来的自查习惯**：算 PWM 频率之前，先去 CubeMX 的 Clock Configuration 页把 `APB1 Timer clocks` 看清楚，不要凭"APB1 = 36 MHz"这个印象直接推。2.3 节那个坑就是没去点开看一眼造成的。

---

## 4. 完整代码

完整工程文件在 [`code/02-pwm-breath/main.c`](../code/02-pwm-breath/main.c)，同目录的 [`README.md`](../code/02-pwm-breath/README.md) 里写了引脚表、CubeMX 配置参数和验证方法。

**先说结论，免得看代码时绕**：呼吸灯用 **TIM2 / CH1 / PA5**（1 kHz），舵机用 **TIM3 / CH1 / PA6**（50 Hz）。为什么不是两个通道共用一个 TIM2，见第 6 节坑 6。

下面这份是 `main.c` 里我自己写的部分（`/* USER CODE BEGIN */` 到 `/* USER CODE END */` 之间的内容），以及三个初始化函数。CubeMX 生成的骨架我原样保留，只标出哪些参数是我填的。

### 4.1 头部与私有定义

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 第 04 章 · 定时器与 PWM
  *                   TIM2_CH1 (PA5) → 1 kHz PWM 驱动外接 LED，做 2 s 周期的呼吸灯
  *                   TIM3_CH1 (PA6) → 50 Hz PWM 驱动舵机，在 0~180° 之间来回扫
  *
  * 时基计算（全部由 72 MHz 推出，验算见注释）：
  *   f_CK_CNT = 72 MHz / (PSC + 1) = 72 MHz / 72 = 1 MHz   → 1 个计数 = 1 µs
  *
  *   TIM2（呼吸灯）: f = 1 MHz / (ARR + 1) = 1 MHz / 1000  = 1 kHz  (ARR = 999)
  *                   → 周期 1 ms，占空比分辨率 1000 级
  *   TIM3（舵机）  : f = 1 MHz / (ARR + 1) = 1 MHz / 20000 = 50 Hz  (ARR = 19999)
  *                   → 周期 20 ms，CCR 的数值 = 高电平的微秒数
  *                     0.5 ms → CCR 500 → 0°，2.5 ms → CCR 2500 → 180°
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
/* 本章没用到额外头文件：呼吸灯是查表 + 整数递推，不需要 math.h */
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ---- 呼吸灯：查表参数的来源 ----
 * 每 50 ms 更新一次 CCR，走完 41 个采样点，两个端点之间有 (41-1) = 40 段间隔，
 * 所以一个完整周期 = 40 × 50 ms = 2000 ms = 2.00 s。
 * 更新节拍（50 ms）和 PWM 载波频率（1 kHz）是两件独立的事：
 * 载波决定"亮灭切换得多快"（快到看不出闪），更新节拍决定"亮度多久变一次"（也就是呼吸的速度）。 */
#define BREATH_LUT_LEN        41U
#define BREATH_STEP_MS        50U

/* ---- 舵机参数 ----
 * TIM3 的 1 个计数 = 1 µs，所以 CCR 的数值就是高电平的微秒数。 */
#define SERVO_MIN_CCR         500U     /* 0.5 ms 高电平 → 0°   */
#define SERVO_MAX_CCR         2500U    /* 2.5 ms 高电平 → 180° */
#define SERVO_MID_CCR         1500U    /* 1.5 ms 高电平 → 90°  */
#define SERVO_STEP_MS         20U      /* 每 20 ms 让角度走 1°，180° 单程 3.6 s */

/* USER CODE END PD */

/* USER CODE BEGIN PV */

/* 呼吸灯亮度表：CCR 值，三角波（线性升 1.00 s + 线性降 1.00 s，合计 2.00 s）
 *
 * 生成规则：一个周期有 40 个台阶，每台阶 50 ms，两端都要采到，所以表长 41（下标 0~40）。
 *   下标 0       ：CCR = 0                                    （全灭，周期起点）
 *   下标 1~19    ：CCR = 50, 100, 150, ..., 950               亮度上升
 *   下标 20      ：CCR = 998                                  （最亮）
 *   下标 21~39   ：CCR = 948, 898, ..., 48                    亮度下降
 *   下标 40      ：CCR = 0                                    （回到全灭，和下标 0 是同一个状态）
 *
 * 一个完整周期的时长 = 40 个台阶 × 50 ms = 2000 ms = 2.00 s。
 * （41 个采样点之间有 40 段间隔，所以是 40 × 50 ms，不是 41 × 50 ms——
 *   这个 +1 和 PSC/ARR 那个 +1 是同一类错误，我差点又栽一次。）
 *
 * 最大值故意只取 998，不取 999（= ARR）：
 * PWM 模式 1 下 CCR > ARR 时 CNT 永远追不上 CCR，输出会贴死在常高电平，
 * 亮度就"卡"在最大值不动了。留一格余量最保险。详见第 6 节坑 5。
 *
 * 为什么用查表而不是算 sin()：查表是整数运算，不用调 libm（省 Flash 也省时间），
 * 而且每个台阶的 CCR 值在源码里看得见，方便对着逻辑分析仪逐点核对。 */
static const uint16_t g_breath_lut[BREATH_LUT_LEN] = {
       0U,   50U,  100U,  150U,  200U,     /* 下标  0~  4，时刻   0~ 200 ms */
     250U,  300U,  350U,  400U,  450U,     /* 下标  5~  9，时刻 250~ 450 ms */
     500U,  550U,  600U,  650U,  700U,     /* 下标 10~ 14，时刻 500~ 700 ms（中等亮度） */
     750U,  800U,  850U,  900U,  950U,     /* 下标 15~ 19，时刻 750~ 950 ms */
     998U,  948U,  898U,  848U,  798U,     /* 下标 20~ 24，时刻 1000~1200 ms（下标 20 最亮，之后转暗） */
     748U,  698U,  648U,  598U,  548U,     /* 下标 25~ 29，时刻 1250~1450 ms */
     498U,  448U,  398U,  348U,  298U,     /* 下标 30~ 34，时刻 1500~1700 ms */
     248U,  198U,  148U,   98U,   48U,     /* 下标 35~ 39，时刻 1750~1950 ms */
       0U                                   /* 下标 40，   时刻 2000 ms（回到全灭） */
};

static uint8_t  g_breath_idx  = 0U;     /* 呼吸灯查表下标，0 ~ 40 */
static uint16_t g_servo_angle = 0U;     /* 舵机当前角度，0 ~ 180（单位：度） */
static int8_t   g_servo_dir   = 1;      /* 舵机扫描方向：+1 增大，-1 减小 */

/* USER CODE END PV */
```

### 4.2 函数声明

```c
/* USER CODE BEGIN PFP */
static void App_BreathLed_Init(void);
static void App_BreathLed_Task(void);
static void App_Servo_Init(void);
static void App_Servo_Task(void);
static uint16_t App_Servo_AngleToCcr(uint16_t angle_deg);
/* USER CODE END PFP */
```

### 4.3 `main()`：先启动 PWM，再进主循环

```c
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
  SystemClock_Config();          /* HSE 8 MHz × PLL9 = 72 MHz；APB1 = 36 MHz，APB1 Timer clocks = 72 MHz */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();                /* 呼吸灯：PSC=71, ARR=999 → 1 kHz */
  MX_TIM3_Init();                /* 舵机：  PSC=71, ARR=19999 → 50 Hz */
  /* USER CODE BEGIN 2 */

  /* ---- 关键：把 PWM 输出打开 ----
   * MX_TIMx_Init() 只是把寄存器的值配好了，CCER 里的通道使能位 CCxE 还是 0，
   * 通道引脚处于"不输出"状态。少了下面两行，配置全对但引脚上一点波形也没有（坑 2）。
   * TIM_CHANNEL_1 + htim2 → PA5（呼吸灯）
   * TIM_CHANNEL_1 + htim3 → PA6（舵机） */
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
```

### 4.4 `SystemClock_Config()`：72 MHz 从哪来

```c
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           /* APB1 = 36 MHz ← 分频系数是 2 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           /* APB2 = 72 MHz */

  /* ⚠️ 注意：APB1 = 36 MHz，但因为 APB1 的分频系数是 2（≠ 1），
   * 硬件会把 TIM2~TIM7 的时钟自动 ×2：
   *      36 MHz × 2 = 72 MHz  ← 这才是我的定时器时钟
   * 所以下面 PSC 填的是 71（72 MHz / 72 = 1 MHz），不是 35。
   * 这个数在 CubeMX 的 Clock Configuration 页叫 "APB1 Timer clocks"。 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
```

### 4.5 `MX_TIM2_Init()`：呼吸灯，1 kHz

```c
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;              /* ← PSC = 71：72 MHz / (71+1) = 1 MHz → 1 个计数 = 1 µs */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 向上计数 = 边沿对齐 */
  htim2.Init.Period = 999;                /* ← ARR = 999：(999+1) = 1000 个计数
                                           *   验算：1 MHz / 1000 = 1000 Hz = 1 kHz，周期 1 ms
                                           *   占空比分辨率 = ARR + 1 = 1000 级（0.1%） */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 只影响输入捕获的数字滤波器采样，不改 PWM 频率 */
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;  /* 用内部时钟 CK_INT，不做外部计数 */
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
   * 这里填 0 → 0% 占空比，上电时 LED 灭，进主循环后被查表值接管。 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  /* 模式 1：CNT < CCR 时输出有效电平 */
  sConfigOC.Pulse = 0;                                 /* CCR1 初值 = 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          /* CCxP = 0 → 有效电平是高电平；PA5 高电平点亮 LED */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- 关键：把 PA5 配成复用推挽输出 ----
   * HAL_TIM_MspPostInit() 本身是 CubeMX 生成的（在 stm32f1xx_hal_msp.c 里），
   * 但 CubeMX 不会自动调它，必须由我在初始化里显式调用一次。
   * 少了这一步，定时器内部计数一切正常，引脚上却什么都没有（坑 2 的兄弟）。 */
  HAL_TIM_MspPostInit(&htim2);
}
```

### 4.6 `MX_TIM3_Init()`：舵机，50 Hz

```c
/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;              /* ← PSC = 71：72 MHz / (71+1) = 1 MHz → 1 个计数 = 1 µs
                                           *   和 TIM2 一样：两个定时器都挂在 APB1 上，
                                           *   时钟同样是 36 MHz × 2 = 72 MHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;              /* ← ARR = 19999：(19999+1) = 20000 个计数
                                           *   验算：1 MHz / 20000 = 50 Hz，周期 20 ms ← 舵机要的
                                           *   分辨率 = 1 个计数 = 1 µs
                                           *   检查位宽：19999 < 65535 ✅ 装得下 */
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

  HAL_TIM_MspPostInit(&htim3);            /* 把 PA6 配成复用推挽输出，必须显式调用 */
}
```

### 4.7 GPIO 初始化与 MSP

```c
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* PA5 / PA6 本章由 HAL_TIM_MspPostInit() 配成复用功能，不在这里当普通 GPIO 配。
   * 这里只把用到的 GPIO 端口时钟打开。 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}
```

CubeMX 生成在 `stm32f1xx_hal_msp.c` 里的两个函数（我原样保留，加了几行注释）：

```c
/* stm32f1xx_hal_msp.c */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if (htim_base->Instance == TIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();   /* 不使能 TIM2 时钟，后面所有寄存器写入都不生效 */
  }
  else if (htim_base->Instance == TIM3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (htim->Instance == TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**TIM2 GPIO Configuration
    PA5   ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          /* 复用推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM3)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**TIM3 GPIO Configuration
    PA6   ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  /* 注意 F1 和 F4 的区别：F4 有 GPIO_AFRL/AFRH 寄存器要写复用编号（如 GPIO_AF1_TIM2），
   * F1 没有这个寄存器，外设与引脚的对应关系出厂固定：
   * PA5 只能是 TIM2_CH1，PA6 只能是 TIM3_CH1，想在 F1 上换通道就必须换引脚。 */
}
```

### 4.8 应用层：呼吸灯 + 舵机

```c
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
  * @note   41 个采样点，两端点之间有 40 段间隔，每段 50 ms → 2.00 s 一个完整呼吸周期
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

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_breath_lut[g_breath_idx]);

  HAL_Delay(BREATH_STEP_MS);      /* 节拍 50 ms = HAL_Delay 的最小粒度 1 ms 的整数倍，所以是准的 */
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

  if (angle_deg > 180U)          /* 限幅，防止算出超过 2.5 ms 的脉宽（超范围会让舵机堵转） */
  {
    angle_deg = 180U;
  }

  /* 用整数算，避免引入浮点和 libm：
   * (2500 - 500) / 180 = 2000 / 180 = 100 / 9
   * → CCR = 500 + angle × 100 / 9
   * 例：angle = 90 → 500 + 90 × 100 / 9 = 500 + 1000 = 1500 ✅ */
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
  *         是为了让"每个 PWM 周期收到一条新位置指令"这件事看得最清楚。 */
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
```

（`HAL_Delay()` 的最小粒度是 1 ms，我用的 50 ms 和 20 ms 都是它的整数倍，所以节拍是准的。到第 09 章做 PID 时主循环不能再被 `HAL_Delay()` 堵住，我打算改成用定时器更新中断来定节拍。）

---

## 5. 逐段解释

### 5.1 `MX_TIM2_Init()` 里每个字段是什么意思

| 字段 | 我填的值 | 含义 / 为什么这么填 |
|---|---|---|
| `htim2.Instance` | `TIM2` | 用哪个定时器。选 TIM2 是因为 CH1 正好在 PA5（外接 LED 已接在那儿） |
| `htim2.Init.Prescaler` | `71` | **PSC**。`72 MHz / (71+1) = 1 MHz`，把 1 个计数变成 1 µs，后面 `CCR` 的数值就能直接当微秒读 |
| `htim2.Init.CounterMode` | `TIM_COUNTERMODE_UP` | 向上计数，也就是**边沿对齐（edge-aligned）** PWM。另一种是中央对齐（先上后下），做电机驱动时会用到，因为它的谐波更少，而且波形天然对称 |
| `htim2.Init.Period` | `999` | **ARR**。`(999+1) = 1000` 个计数 → `1 MHz / 1000 = 1 kHz`。字段名是 `Period`，但填的是 `ARR` 而不是"周期的毫秒数"，容易看错 |
| `htim2.Init.ClockDivision` | `TIM_CLOCKDIVISION_DIV1` | 这是输入捕获数字滤波器的采样分频，**完全不改变 PWM 输出频率**。我一开始以为它也在分频 |
| `htim2.Init.AutoReloadPreload` | `TIM_AUTORELOAD_PRELOAD_DISABLE` | 关闭 `ARR` 的预装载：写 `ARR` 立刻生效。我改的是 `CCR` 不是 `ARR`，所以关着没影响；如果要在运行中改频率（改 `ARR`），打开它能避免出现一个畸变的周期 |
| `sClockSourceConfig.ClockSource` | `TIM_CLOCKSOURCE_INTERNAL` | 用定时器内部的 `CK_INT`（72 MHz）。其它选项是外部时钟模式（数引脚上的脉冲）、内部触发（级联另一个定时器） |
| `MasterOutputTrigger` / `MasterSlaveMode` | `TIM_TRGO_RESET` / `DISABLE` | 本章不用定时器级联，保持默认。做定时器触发 ADC 同步采样（第 07 章）时才会动它 |
| `sConfigOC.OCMode` | `TIM_OCMODE_PWM1` | **PWM 模式 1**：`CNT < CCR` 时输出有效电平。换成 `TIM_OCMODE_PWM2` 波形反相 |
| `sConfigOC.Pulse` | TIM2: `0`；TIM3: `1500` | 就是 **CCR** 的初值。TIM2 填 0 = 上电灭灯；TIM3 填 1500 = 1.5 ms = 舵机 90°。**CubeMX 里这个框的单位是"计数个数"，不是时间**——在 `PSC = 71` 的配置下它恰好等于微秒数，纯属数值巧合 |
| `sConfigOC.OCPolarity` | `TIM_OCPOLARITY_HIGH` | `CCxP = 0`，有效电平 = 高电平。外接 LED 接成 PA5 → 1 kΩ → LED → GND，**高电平点亮**，所以"有效电平 = 高"最直观 |
| `sConfigOC.OCFastMode` | `TIM_OCFAST_DISABLE` | 快速模式只在单脉冲 + 输出比较的特定场景有用，PWM 下关掉 |

### 5.2 为什么要单独调用 `HAL_TIM_MspPostInit()`

先分清 CubeMX 生成的两个 `MspInit`，名字像但用途完全不同：

| 函数 | 干什么 | 谁调用 |
|---|---|---|
| `HAL_TIM_Base_MspInit()` | 使能**定时器外设时钟**（`__HAL_RCC_TIM2_CLK_ENABLE()`）。不使能时钟，后面所有寄存器写入都不生效 | HAL 会在 `HAL_TIM_Base_Init()` 内部回调它，不用我管 |
| `HAL_TIM_MspPostInit()` | 把 **PA5 / PA6 配成复用推挽输出**（`GPIO_MODE_AF_PP`） | **必须我在 main 里显式调用**，HAL 不会自动调它 |

**为什么 HAL 不自动调用 `HAL_TIM_MspPostInit()`？** 我理解是这样：一个定时器通道可以当 PWM 输出，也可以当输入捕获、编码器接口——后两种用法引脚是**输入**方向，配置完全不同；还有"某个通道干脆不用"的情况。HAL 在初始化阶段无法判断这个通道最终是输出还是输入，所以把"引脚怎么配"留给用户在确定用途之后自己调。CubeMX 生成的代码里，这个函数就孤零零放在 `stm32f1xx_hal_msp.c` 末尾等着被调用。

**如果忘了调会怎样？** 定时器的 `CNT` 正常在跑、`CCR` 也在比较、状态寄存器里一切正常，但 PA5 引脚仍然是复位后的默认状态（**浮空输入**），示波器上是一条平线。这个现象特别容易误导人——因为你在 Keil 里单步看 `TIM2->CCR1`、`TIM2->CNT` 都是对的，会以为"配置没问题，是不是硬件坏了"。

F1 和 F4 的一个区别也顺便记一下：F4 有 `GPIO_AFRL/AFRH` 寄存器，要写复用编号（比如 `GPIO_AF1_TIM2`）；**F1 没有这个寄存器，外设和引脚的对应关系出厂固定**，PA5 只能是 TIM2_CH1，想换通道就必须换引脚。所以在 F1 上"配复用"就是一句话：`GPIO_MODE_AF_PP`。

### 5.3 `__HAL_TIM_SET_COMPARE()` 是怎么改占空比的

`__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value)` 展开后就是把 `value` 写进 `TIM2->CCR1`（通道 3 就是 `CCR3`）。

它是宏不是函数，展开后是一条赋值语句，比我一开始想的 `HAL_TIM_PWM_Stop()` → 改值 → `HAL_TIM_PWM_Start()` 快得多，也**不需要停掉输出**。呼吸灯每 50 ms 调一次，CPU 花的时间可以忽略。

写 `CCR` 什么时候生效？这取决于 `CCMRx` 里的 `OCxPE` 位（输出比较预装载使能）：

- `OCxPE = 0`（CubeMX 默认生成的配置）：写进去**立刻**参与比较。如果此时 `CNT` 已经跑过新的 `CCR` 了，那这一周期的高电平段就已经过去了，输出保持低电平到本周期结束——**表现为"这一拍没反应"，要等下一个周期才看到完整的新占空比**。
- `OCxPE = 1`：新值先存进预装载寄存器，等**下一次更新事件（`CNT` 溢出）**才整体搬到影子寄存器，这一拍的波形永远是完整的，代价是有一拍延迟。

在 1 kHz（1 ms 周期）下这点差别肉眼看不出；但如果在 50 Hz（20 ms 周期）的通道上改，延迟最长就是 20 ms，能明显感觉到"指令慢了一拍"。

### 5.4 为什么呼吸灯和舵机用了两个定时器

这是本章我最费脑子的一段，结论在坑 6 里，这里只把结论和取舍讲清楚。

**问题的根源**：同一定时器的所有通道共享 `CNT`、`PSC`、`ARR`，也就是**共享同一个溢出周期**。呼吸灯想要 1 ms 的帧（1 kHz），舵机想要 20 ms 的帧（50 Hz），一个 TIM2 给不出两个频率。

我一开始试着"让舵机通道的 `CCR` 大于 `ARR`，靠跨帧保持高电平拼出 1.5 ms 的脉宽"——**这个思路是错的**，原因写在坑 6。正确的结论是：**想要某个通道有精确的周期，那个定时器的时基就必须是那个周期。**

于是只有两条路：

| 方案 | 做法 | 优点 | 缺点 | 我的取舍 |
|---|---|---|---|---|
| ① 两个定时器 | TIM2 给呼吸灯（`PSC=71, ARR=999` → 1 kHz），TIM3 给舵机（`PSC=71, ARR=19999` → 50 Hz） | 两个频率各自精确；呼吸灯 1 kHz 载波完全不闪；概念干净（一个外设干一件事） | 多占一个定时器；多写一份初始化 | ✅ **最终选的**。F103C8T6 有 TIM1/2/3/4 可用，第 05/07 章还要用 USART1 和 ADC1，定时器不紧张 |
| ② 一个定时器，时基取 50 Hz | TIM2 的 `ARR` 改成 19999，呼吸灯和舵机都挂在 TIM2 上 | 配置量最小，一个 `MX_TIM2_Init()` 就够 | 呼吸灯的载波变成 50 Hz（有轻微频闪风险）；占空比分辨率被 `ARR = 19999` 限制；而且两个通道互相牵制 | 我按这个先跑通了逻辑（改 `CCR` 的机制是一样的），但最后还是拆开了 |

**这里有一个必须说清楚的引脚影响**：方案 ① 里舵机从 PA2（TIM2_CH3）换到了 **PA6（TIM3_CH1）**。原因是 PA2 属于 TIM2，挂在 TIM2 上就躲不开"和呼吸灯共用时基"这个问题。

> ⚠️ 这一处和仓库统一引脚表里的「舵机 PWM = PA2 / TIM2_CH3」不一致，我把原因记在这里：**PA2 是 TIM2_CH3，和呼吸灯的 TIM2_CH1 共用时基；想让舵机拿到精确的 50 Hz，就得把整个 TIM2 降到 50 Hz，呼吸灯会跟着变成 50 Hz 载波。** 所以我改用 TIM3_CH1 / PA6。
>
> 顺便记一个我查引脚表时发现的事：**PA2 同时也是 USART2_TX**。虽然第 05 章我用的是 USART1（PA9/PA10），占不到 PA2，但这一条说明"一个引脚能干的活不止一种"，改引脚分配之前应该先在 CubeMX 里点一下那个引脚，看清它有哪些可选功能，别只看当前用的是哪个。
>
> **如果战队要求必须用 PA2 输出舵机 PWM**，那就只能回到上面方案 ②（整个 TIM2 取 50 Hz），代价是呼吸灯的载波也变成 50 Hz。两种做法我都能解释清楚为什么，面试如果被问到，我会说"我选了拆成两个定时器，因为呼吸灯和舵机对帧长的需求是矛盾的，而 F103C8T6 上的定时器够用"。

**方案 ① 还有一个附加好处**：TIM3 的 `ARR = 19999` 让它有 20000 级的占空比分辨率（1 µs），比 TIM2 的 1000 级细得多。舵机需要的是脉宽的精细度（0.5~2.5 ms 区间内要分得开角度），呼吸灯只需要 1000 级就足够平滑——**两个外设各按自己的需求配分辨率，这才是我拆开它们的真正理由。**

---

## 6. 我踩的坑

这一章的坑基本分两类：数字算错，和少调一个函数。每个坑我都把原因分析到位，不只写现象。

### 坑 1：把 TIM2 的时钟当成 36 MHz，频率差了一倍

- **我以为**：APB1 = 36 MHz，TIM2 挂在 APB1 上，所以 `f_CK_CNT = 36 MHz`，于是填 `PSC = 35` 想得到 1 MHz。
- **实际**：APB1 的分频系数是 2（≠ 1），硬件把定时器时钟**自动 ×2**，`f_CK_CNT = 72 MHz`。
- **后果**：实际频率是我目标的 2 倍。反过来也一样——如果按 36 MHz 算好参数、却按 72 MHz 执行，我想要 1 kHz 就会得到 **500 Hz**。
- **怎么定位**：① 打开 CubeMX 的 **Clock Configuration** 页，把鼠标悬在时钟树右侧 `APB1 Timer clocks` 上，它显示 **72 MHz**；② 用逻辑分析仪量 PA5 的周期，是 1.000 ms 还是 2.000 ms——差整整 2 倍就是时钟算错的典型指纹。
- **教训**：算 `PSC`/`ARR` 之前，先把"这个定时器的时钟到底是多少 MHz"写在草稿纸最上面。**"总线时钟"和"定时器时钟"不是同一个数。**

### 坑 2：配置全对，但忘了 `HAL_TIM_PWM_Start()`，引脚一点输出都没有

- **现象**：CubeMX 里 PSC/ARR/Pulse 全填对了，编译烧录也没报错，逻辑分析仪接上 PA5 却是一条平线。
- **原因**：`MX_TIM2_Init()` 做的是"把寄存器配置好"，但 `TIMx_CCER` 里的通道使能位 `CC1E` 仍然是 0，通道处于关闭状态。**配置 ≠ 启动。** `HAL_TIM_PWM_Start()` 才会去置 `CC1E`，把比较器的结果真正接到引脚上。
- **怎么定位**：Keil 里打开 `Peripherals → System Viewer → TIM2`，看 `CCER` 的 bit0（`CC1E`）。它是 0，波形就永远出不来。也可以在 `HAL_TIM_PWM_Start()` 前后各看一眼。
- **教训**：配置和启动是两个独立动作。**中断要 `HAL_TIM_Base_Start_IT()`，PWM 要 `HAL_TIM_PWM_Start()`，哪个都不能漏。** 同一个坑还有第三种形态：`HAL_TIM_MspPostInit()` 没调，症状一样——引脚没被配成复用输出。

### 坑 3：把舵机 PWM 搞成 1 kHz，舵机疯狂抖动、发热

- **现象**：舵机不定位，而是高频抖动，手摸外壳明显发烫，电源电流也上去了。
- **原因**：舵机内部的**比较电路测的是高电平的持续时间**，它把"这一帧的脉宽"当成位置指令。如果频率是 1 kHz（每 1 ms 来一个脉冲），舵机在 20 ms 里会收到 20 条指令，它来不及结算完一条就开始处理下一条，会误判。更糟的是，如果脉宽落在有效的 0.5~2.5 ms 区间之外，舵机认为信号无效或超范围，就朝一个方向不停顶——**堵转**。堵转电流大，温度很快上来，容易烧舵机或烧电源。
- **正确参数**：**50 Hz，周期 20 ms，高电平 0.5~2.5 ms**。
- **怎么定位**：先量周期。舵机信号周期必须是 20.00 ms，量到 1 ms 就说明时基配错了。
- **教训**：**执行器和 LED 不一样，要先看它的数据手册再定频率。** 舵机 50 Hz 是行业约定，不是随便挑的。

### 坑 4：改了 `CCR` 但没重装载，要等下一个周期才生效

- **现象**：呼吸灯每 50 ms 改一次 `CCR`，我把步长缩小到每 1 ms 试的时候，`CCR` 每次确实都写进去了，亮度却像"隔一拍才动"，跟着主循环的节拍打嗝。
- **原因**：`__HAL_TIM_SET_COMPARE()` 把值写进 `CCR`，但**"什么时候生效"要看 `CCMRx` 的 `OCxPE`（输出比较预装载使能）位**。CubeMX 默认没打开它，值是直接写进当前寄存器立刻参与比较的；问题在于一个周期已经被"切"开了——如果我是在 `CNT` 已经跑过 `CCR` 之后才改的值，**这一拍的高电平段已经过去了**，剩下的时间只能是低电平。新值要等下一拍才能看到完整效果，这就是"这一拍没反应"的来源。
- **另一个更长的延迟**：如果打开 `OCxPE`（预装载），新值要等**下一次更新事件（`CNT` 溢出）**才从预装载寄存器搬到影子寄存器。在 1 kHz 通道上这个延迟最长 1 ms，看不出来；但如果是在 50 Hz 的舵机通道上改角度，最长延迟就是 20 ms，能明显感觉到"指令慢了一拍"。
- **怎么定位**：在 `__HAL_TIM_SET_COMPARE()` 前后各读一次 `TIMx->CCR1` 和 `TIMx->CNT`。如果 `CNT > 新写的 CCR`，就能确认这一拍已经"过期"。
- **教训**：改 `CCR` 的时机要么放在更新中断里（保证每拍都完整），要么就把更新节拍设得比 PWM 帧慢得多。我呼吸灯是 50 ms 改一次、帧长 1 ms，那点延迟完全无所谓。

### 坑 5：占空比设到 100%，反而"没有输出"

- **现象**：我以为占空比设成 100% 会得到"一直亮的 LED"，结果逻辑分析仪上是一条**没有任何跳变**的直线，第一反应是"坏了，没输出"。
- **原因**：这其实是**正确行为**。PWM 模式 1（`CNT < CCR` 时输出有效电平）下，如果 `CCR > ARR`，整个周期内 `CNT` 都不可能达到 `CCR`，输出就一直停在**有效电平**上。对"高电平有效"的配置来说就是**常高**。看起来"没有波形"，只是因为没有跳变——平均电压实际上是最大的。
  - 补一个对称的边界：占空比 0%（`CCR = 0`）时，`CNT = 0` 就已经不满足 `CNT < CCR`，输出是**常低**。
  - 所以测量工具对 0% 和 100% 两种情况都读不出"频率"——它们本来就没有跳变。
- **我在呼吸灯里差点吃这个亏**：如果让查表的最大值取到 `ARR + 1 = 1000`，呼吸灯在"最亮"那一小段时间里 `CCR > ARR`，输出变成常高，LED 会"顶"在那里不动，看起来像卡了一下。**所以我的表里最大值只取 998，留一格余量。**
- **一个我一开始担心、后来发现不成立的说法**："占空比 100% 时 `CCR = ARR + 1 = 1000`，`CCR` 只有 16 位，会不会装不下？"——不成立，`CCR` 是 16 位，装得下 1000，一直到 65535。真正的问题不是位宽，而是**输出不再变化**。
- **怎么定位**：用示波器看**平均值（`MEAN` 读数）**，不要只看波形有没有跳变。常高电平的平均值 ≈ 3.3 V，常低 ≈ 0 V，这样就能区分"真的没输出"和"输出了一个不变的 100%"。
- **教训**：**"波形是平的"不等于"没有输出"。** 要同时看直流平均值和有无跳变，才能判断到底是寄存器没生效，还是占空比落在了两个极值上。

### 坑 6（这个坑是我自己推理推错的，记下来最有价值）：以为 `CCR > ARR` 能让一个通道变成低频

这是本章最值钱的一个坑，因为它不是抄错参数，而是我的推理本身错了。

- **我以为**：既然 TIM2 的 CH1 和 CH3 共用 1 ms 的帧，那把 CH3 的 `CCR` 设成 1500（= 1.5 ms，大于 `ARR = 999`），输出就会"跨过 1 ms 的帧边界继续保持高，等到 1.5 ms 才翻低"，这样就能拼出 1.5 ms 的脉宽，也就顺便得到了低频的舵机信号。
- **实际**：`CNT` **每个溢出周期都从 0 重新开始**。它从 0 数到 999 就被重载回 0，**永远到不了 1500**。所以每一帧都是"整帧常高"，一帧接一帧，输出变成**恒定高电平**，永远不翻低。我想要的 1.5 ms 脉宽压根没有出现。
- **正确的做法**：想让舵机拿到精确的 20 ms 周期，只能让**时基本身就是 20 ms**。而时基是通道共享的，所以要么给舵机单独一个定时器（我最终用 TIM3），要么把整个 TIM2 降到 50 Hz（呼吸灯的载波会跟着变成 50 Hz）。
- **怎么定位**：量 PA2 的周期。如果示波器平均值 ≈ 3.3 V 且完全没有跳变，就说明 `CCR` 落到了 `ARR` 之外。
- **教训**：**比较器的输入是 `CNT` 的即时值，不是"从上次翻转到现在的累计时间"。** `CNT` 到不了的数，输出就永远翻不过去。一个定时器只有一个时基，这条没有例外。

| 坑 | 一句话原因 | 我先看到的现象 | 判据 / 定位手段 |
|---|---|---|---|
| 1 定时器时钟算错 | APB1 分频 ≠ 1 时 `TIMxCLK` = APB1 × 2 = 72 MHz | 频率正好是期望值的 2 倍或一半 | Clock Configuration 页的 `APB1 Timer clocks`；量周期 |
| 2 忘记 `HAL_TIM_PWM_Start()`（或 `HAL_TIM_MspPostInit()`） | 只配了寄存器，`CC1E` 还是 0 / 引脚还是浮空输入 | 配置全对，引脚一条平线 | 看 `TIMx->CCER` 的 bit0；确认 MspPostInit 被调用 |
| 3 舵机频率给成 1 kHz | 舵机测的是脉宽不是占空比，1 ms 一条指令导致堵转 | 舵机抖动、发热 | 量信号周期，必须是 20 ms |
| 4 改 `CCR` 没等重装载 | 值写到了"已经过期"的那一拍；开了预装载则要等下一帧 | 亮度/角度慢一拍，跟主循环节拍打嗝 | 对比 `CCR` 与 `CNT` 的当前值 |
| 5 占空比给到 100% | `CCR > ARR` 时 `CNT` 追不上，输出贴死在常高电平 | 波形是直线，误判为"无输出" | 看示波器平均值（≈3.3 V）+ 有无跳变 |
| 6 想让 `CCR > ARR` 变低频 | `CNT` 每帧都从 0 重来，到不了的数就翻不过去 | 想拼出 20 ms，结果得到恒定高电平 | 量周期；记住"一个定时器只有一个时基" |

---

## 7. 复现步骤

### 7.1 硬件准备

| 器件 | 连接 |
|---|---|
| 外接 LED 一路 | PA5 → 1 kΩ 限流电阻 → LED 阳极（长脚），LED 阴极（短脚）→ GND。**限流电阻的算法见[第 03 章](03-电路基础-限流与分压.md)** |
| 舵机（可选） | 舵机红线（+5 V）→ 独立 5 V 电源正极；棕线（GND）→ 独立电源负极 **并和板子 GND 相连**；橙线（信号）→ PA6（TIM3_CH1） |
| ST-Link V2 | SWDIO → PA13，SWCLK → PA14，GND → GND，3.3 V → 3.3 V |
| 逻辑分析仪 | CH0 → PA5（看呼吸灯），CH1 → PA6（看舵机），**GND 必须和板子共地** |

> ⚠️ 舵机的电流不小（SG90 空载 100 mA 以上，堵转能到 700 mA 左右）。**不要从 ST-Link 的 3.3 V 取电**，要用独立的 5 V 电源，并且**必须共地**——不共地的话信号没有共同的参考点，舵机会乱抖，这个现象很容易被误判成"PWM 参数不对"。

### 7.2 CubeMX 配置（一步步来）

1. **新建工程**：`File → New Project`，`Part Number` 搜 `STM32F103C8`，选 **`STM32F103C8Tx`**，点 `Start Project`。
2. **调试接口**：`System Core → SYS`，`Debug` 选 **`Serial Wire`**。
   （不选这一步，第一次烧录后 SWD 引脚会被当普通 GPIO 用掉，之后就再也连不上，得靠 BOOT0 上拉才能救回来。）
3. **时钟源**：`System Core → RCC`，`High Speed Clock (HSE)` 选 **`Crystal/Ceramic Resonator`**（板上是 8 MHz 晶振）。
4. **配时钟树**：切到 **Clock Configuration** 页：
   - `Input frequency` 填 **8**（MHz）；
   - `PLL Source` 选 **HSE**，`PLLMul` 选 **×9**；
   - `System Clock Mux` 选 **PLLCLK**；
   - `HCLK (AHB)` 填 **72**，回车让它自动算分频；
   - `APB1 Prescaler` 选 **/2**（→ 36 MHz），`APB2 Prescaler` 选 **/1**（→ 72 MHz）；
   - **把鼠标悬停在右侧 `APB1 Timer clocks` 上，确认显示 `72 MHz`**。这一格是本章最容易搞错的地方。
5. **配 TIM2（呼吸灯）**：`Timers → TIM2`：
   - `Clock Source` = **Internal Clock**；
   - `Channel1` = **PWM Generation CH1**；
   - `Parameter Settings` 里：`Prescaler (PSC - 16 bits value)` = **71**，`Counter Mode` = **Up**，`Counter Period (AutoReload Register - 16 bits value)` = **999**，`Clock Division` = **No Division**，`Auto-reload preload` = **Disable**；
   - `PWM Generation Channel 1` 里：`Mode` = **PWM mode 1**，`Pulse (16 bits value)` = **0**，`Output compare preload` = **Disable**，`Fast Mode` = **Disable**，`CH Polarity` = **High**；
   - 右侧芯片图上确认 **PA5** 被标成 `TIM2_CH1`（绿色）。
6. **配 TIM3（舵机）**：`Timers → TIM3`：
   - `Channel1` = **PWM Generation CH1**；
   - `Prescaler` = **71**，`Counter Mode` = **Up**，`Counter Period` = **19999**；
   - `Mode` = **PWM mode 1**，`Pulse` = **1500**，`CH Polarity` = **High**；
   - 确认 **PA6** 被标成 `TIM3_CH1`。
7. **检查引脚**：回到 `Pinout & Configuration` 页，确认 PA5 = TIM2_CH1、PA6 = TIM3_CH1，没有红色冲突提示。
8. **生成代码**：`Project Manager` 页：
   - `Project Name` 填 `02-pwm-breath`；`Toolchain / IDE` 选 **MDK-ARM**，版本 **V5**；
   - `Code Generator` 页勾上 **`Generate peripheral initialization as a pair of .c/.h files per peripheral`**（让 `MX_TIM2_Init()` / `MX_TIM3_Init()` 单独放在 `tim.c` 里，结构清楚）；
   - 点 `GENERATE CODE`。
9. **把我写的部分填进去**：打开生成的 `Core/Src/main.c`，把第 4 节里 `/* USER CODE BEGIN xxx */` 和 `/* USER CODE END xxx */` 之间的内容抄进去。
   > ⚠️ **一定要写在 `USER CODE BEGIN/END` 之间**，否则下次在 CubeMX 里改配置重新生成时，我写的代码会被直接覆盖掉。

### 7.3 Keil 里编译和烧录

1. 用 Keil MDK 打开生成的 `MDK-ARM/02-pwm-breath.uvprojx`。
2. `Project → Build Target`（**F7**）编译，确认 `0 Error(s)`。
3. 点 `Options for Target`（魔术棒图标）→ `Debug` 页：
   - 右侧下拉选 **`ST-Link Debugger`**，点 `Settings`；
   - `Port` 选 **SW**（不是 JTAG），`Max Clock` 保持默认；
   - 切到 `Flash Download` 页，勾上 **`Reset and Run`**（烧完自动运行，不用每次手动按复位）。
4. ST-Link 插好，点 **`LOAD`**（或 F8）下载。

### 7.4 怎么验证（🔬 以下全部是预期现象，我还没上机测）

**用逻辑分析仪**（比示波器更适合看"周期对不对"）：

| 探针 | 预期看到 | 判据 / 不对时先查什么 |
|---|---|---|
| PA5 | 频率 **1.000 kHz**（周期 1.000 ms）；占空比从 **0%** 线性升到 **99.8%**（`CCR = 998`，即 `998/1000`）再降回 0%，**整体亮度周期 2.00 s** | 频率不对 → 查 `PSC`/`ARR`/定时器时钟（坑 1）；频率对但占空比不变 → 查 `__HAL_TIM_SET_COMPARE()` 有没有真的在执行 |
| PA6 | 频率 **50.00 Hz**（周期 20.00 ms）；高电平宽度在 **0.5 ms ~ 2.5 ms** 之间按三角波变化 | 量到 1 ms 或 500 Hz → 时基配错了（坑 3） |

**用示波器**：

- 直接量 PA5 的周期和高电平宽度。`PSC = 71, ARR = 999` 时，`CCR = 500` 应该对应高电平 **500 µs**、占空比 **50%**——这是验证"公式对不对"最直接的一步。
- 测 LED 的**平均电压**（示波器的 `MEAN` 读数）：占空比线性变化时，它也应该在 0 ~ 3.3 V 之间线性跟着动。🔬 预期最低约 **0.00 V**（`CCR = 0`），最高约 **3.29 V**（`CCR = 998`，即 `998 / 1000 × 3.3 V = 3.29 V`）。
- 量 PA6 时如果看到 **恒定高电平且平均值 ≈ 3.3 V**，那就是 `CCR` 超出了 `ARR` 范围（坑 5 / 坑 6）。

**没有仪器时的替代验证**：

- 呼吸灯：用秒表量两次"最亮"之间的时间，应该是 2.00 s。这个方法只能验证数量级，量不出 1 kHz。
- 舵机：先把角度写死（比如只调 `App_Servo_AngleToCcr(0)`），上电后舵机应该"啪"地转到一端并稳住、不抖。**如果它一直抖，第一件事不是怀疑代码，而是检查舵机的 5 V 电源是不是和板子共地、电流够不够。**

**Wokwi 仿真能不能验证？** Wokwi 的 STM32F103C8 仿真**支持 TIM1~TIM4**，所以本章的 PWM 可以在里面跑起来看波形，也没有"舵机"元件可以直接看角度变化，只能看波形。⚠️ 另外要注意 Wokwi 里这些是**未实现**的：**CAN / DMA / RTC / IWDG / PWR**——本章用不到它们。

---

## 8. 自测题（面试可能这么问）

### Q1（计算题）给出 72 MHz 的定时器时钟，要产生 **50 Hz 的舵机 PWM**，且分辨率不低于 **1 µs**，`PSC` 和 `ARR` 应该怎么取？

**答**：先看分辨率，再看周期。

分辨率要求 1 µs，就让一个计数 = 1 µs：

```text
f_CK_CNT = 1 MHz
PSC + 1 = 72 000 000 / 1 000 000 = 72
PSC = 71
```

周期要求 50 Hz → `T = 1 / 50 = 0.02 s = 20 ms = 20 000 µs`，需要 20000 个计数：

```text
ARR + 1 = 20 000
ARR = 19 999
```

**验算**：

```text
f = 72 000 000 / (71 + 1) / (19 999 + 1)
  = 72 000 000 / 72 / 20 000
  = 1 000 000 / 20 000
  = 50 Hz                    ✅ 周期 20 ms
分辨率 = 1 / 1 MHz = 1 µs    ✅
位宽检查：ARR = 19999 < 65535 ✅（16 位装得下，PSC = 71 也远小于 65535）
```

有两点要能接着往下说：

- 如果要把分辨率做到 **0.5 µs**：`PSC = 35`（`f_CK_CNT = 2 MHz`），凑 20000 µs 需要 `ARR + 1 = 40000` → `ARR = 39999`，**仍在 16 位内 ✅**。
- 如果再往上要 **0.1 µs**：`PSC = 6`（10 MHz 计数时钟），需要 `ARR + 1 = 200000` → 超过 **65535 了，这条路走不通**。这就是"分辨率越高，越做不出低频"的边界——**在 16 位定时器上，频率和分辨率是一对矛盾，不能同时要。**

### Q2 为什么舵机要 50 Hz，而不是越高越好？

**答**：三个原因。

1. **舵机内部是靠模拟电路比较脉宽的**。它测的是"高电平持续了多久"，再把这个时长和内部基准比较，去驱动电机转到对应角度。如果频率太高（比如 1 kHz，每 1 ms 来一个脉冲），舵机在 20 ms 内会收到 20 条位置指令，内部比较电路来不及结算完一条就开始处理下一条，会**误判**。
2. **每 20 ms 更新一次是行业约定**。这个 50 Hz / 0.5~2.5 ms 的标准来自早期模拟舵机时代，几十年来所有厂商都按它做，所以现在的数字舵机也兼容这套时序。用别的频率不是"更先进"，而是"不兼容"。
3. **频率太高还有实际的坏处**：脉宽一旦落在 0.5~2.5 ms 之外，舵机会认为信号无效或超范围，就一直朝一个方向顶，变成**堵转**——堵转电流大、发热快，容易烧舵机或烧电源。

换个角度说：**舵机的信息载体是脉宽（0.5~2.5 ms），不是占空比。** 这一点和 LED 完全相反——LED 用的是平均功率，占空比直接决定亮度，频率只要高到人眼不闪就行（一般 ≥ 100 Hz）。**执行器（actuator）和指示器（indicator）的需求不一样，不能套同一套参数。**

### Q3 `PSC` 和 `ARR` 谁决定频率、谁决定分辨率？为什么公式里都要 `+1`？

**答**：

- 两者**共同决定频率**：`f = f_CK_CNT / ((PSC+1) × (ARR+1))`，它们是一对分频系数。
- 但**分辨率只由 `ARR` 决定**（边沿对齐向上计数时）：占空比可分的等级数 = `ARR + 1`，因为一个周期里 `CNT` 会经过 `ARR + 1` 个状态，`CCR` 落在哪个状态就决定了高电平占多少。
- 所以定参数的顺序是：**先用分辨率要求定 `ARR`（不超过 65535），再把剩下的分频量交给 `PSC`。**

**为什么都 `+1`**：因为寄存器里存的是"分频系数减一"。`PSC = 0` 的含义是"不分频"，也就是除以 1；`PSC = 71` 是除以 72。计数器同理，从 0 数到 `ARR = 999` 一共经过 0、1、…、999 这 **1000** 个状态，所以一个周期的计数个数是 `ARR + 1 = 1000`。

我记的口诀：**寄存器里写 N，实际就是除以 N+1。** 忘了这个 +1，算出来的频率会偏，而且数越小偏得越多——`PSC = 71` 时忘掉 +1 会让频率偏 `1/72 ≈ 1.4%`（用 1 kHz 来说就是偏到约 986 Hz，逻辑分析仪上能看出来）。

### Q4 PWM 模式 1 和模式 2 有什么区别？输出比较模式和 PWM 模式又差在哪？

**答**：

**模式 1 和模式 2** 差在"`CNT < CCR` 时输出有效还是无效"：

| | `CNT < CCR` | `CNT ≥ CCR` |
|---|---|---|
| PWM 模式 1 | 有效电平 | 无效电平 |
| PWM 模式 2 | 无效电平 | 有效电平 |

这两个模式配合 `CCxP`（输出极性位：0 = 高电平有效，1 = 低电平有效），一共四种组合。**所以在"占空比越大越亮"这件事上，模式 1 + 高有效，和模式 2 + 低有效，效果是一样的**——都能实现，只是配置的语义不同。

**输出比较 vs PWM 模式**：输出比较是"`CNT == CCR` 的**那一个瞬间**动一下引脚"，一个周期只跳变一次，要靠 CPU 每周期进中断去改才能拼出连续波形；PWM 模式是硬件持续按 `CNT` 与 `CCR` 的比较结果驱动引脚，**配置 + 启动之后 CPU 完全不用管**。

**所以机器人必须用 PWM 模式**：电机和舵机需要长时间稳定的波形，同时 CPU 要腾出来跑姿态解算和 PID。软件延时做 PWM 会占满 CPU，而且延时会被更高优先级的中断打断产生抖动——对舵机来说，抖动的脉宽就是"错误的指令"。

### Q5 我写了 `__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999)` 改占空比，为什么引脚上的波形没变化？

**答**：按可能性从高到低排，这也是我真实的排查顺序：

1. **定时器根本没启动**。`HAL_TIM_PWM_Start()` 没调，写 `CCR` 只是写了个数字，引脚没有任何输出。查 `TIM2->CCER` 的 `CC1E` 位。
2. **引脚复用没配**。`HAL_TIM_MspPostInit()` 没被调用，GPIO 还停在复位后的浮空输入状态。症状和上一条几乎一样。
3. **通道对不上**。`TIM_CHANNEL_1` + `htim2` 对应 PA5，`TIM_CHANNEL_1` + `htim3` 对应 PA6。如果探针接的是 PA6 而代码改的是 `htim2`，那就是改了另一个引脚上的波形。
4. **值写进去了，但这一拍已经过期**。`CNT` 已经跑过 `CCR`，这一周期剩下的时间只能是低电平，要等下一个周期才看到新占空比（坑 4）。波形其实变了，只是要等最多一个周期。
5. **值撞到了边界**。`CCR = 999` 在 `ARR = 999` 下是合法的，占空比 = `999 / 1000 = 99.9%`，正常输出。**真正会出问题的是 `CCR > 999`**（比如手滑写成 1000 或更大），那时 `CNT` 永远追不上，输出变成**常高电平**，看起来就像"没有波形"。这种情况要看示波器的**平均值**，而不是看有没有跳变。

**一句话答法**：先确认"**启动了吗 → 引脚复用配了吗 → 通道和引脚对得上吗**"这三件事，再看"**这一拍是不是已经过期了**"，最后看"**值是不是撞到 `ARR` 边界了**"。

---

> **本章小结（给自己看）**
> 我现在能拿着任意目标频率反推 `PSC` 和 `ARR`，能说清"APB1 分频 ≠ 1 时定时器时钟 ×2"这条规则，也会去 Clock Configuration 页确认定时器时钟，而不是凭印象推。我也知道了为什么呼吸灯和舵机要拆成两个定时器——**一个定时器只有一个时基**。
> 还没做的三件事：① **上机实测**，本章所有现象都标了 🔬，等板子/舵机到手后逐条验证；② **TIM1 的互补输出 + 死区**，这是驱动无刷电机真正要用的东西，本章只把概念铺垫完，代码还没写，**不要在报名材料里写成已完成**；③ **定时器更新中断**做精确节拍，第 09 章做 PID 会用到。

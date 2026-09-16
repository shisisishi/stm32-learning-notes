# 02 · GPIO 与第一个点灯程序：从"能亮"到"知道为什么亮"

> **本章目标**：搞明白 STM32 的一个引脚到底有哪几种工作模式、为什么板载 LED 是低电平点亮、限流电阻怎么算，并写出一个不阻塞的闪灯程序。
> **前置章节**：[01 环境搭建](01-环境搭建.md)
> **硬件**：STM32F103C8T6 最小系统板（Blue Pill）、830 孔面包板、红色 LED 1 个、330 Ω 电阻 1 个、杜邦线、ST-Link V2
> **代码**：[`code/01-blink`](../code/01-blink)
> **状态**：🔬 待上机验证（代码已按 STM32CubeMX 生成的 `main.c` 骨架写完整，**尚未在真板上烧录运行**）

> **这一章的真实性说明**
> 我把这一章的程序写完了，但还没在板子上跑过。所以下面凡是描述"灯怎么闪""示波器上看到什么"的地方，
> 写的都是**预期现象**并标了 🔬，等上机之后我会回来把实测结果补上、把 🔬 改成 ✅。
> 只有能用手册和算术推出来的结论（电流、电阻、寄存器行为）我才直接下判断，并且注明出处。

---

## 1. 先想清楚一个问题

**为什么最小系统板上的 LED 接在 PC13，而且要点亮它得把引脚拉低？**

我第一次照着教程写点灯程序的时候，脑子里默认的模型是"输出高电平 = 点亮"。
直到我把板子上这颗灯真正点亮才发现：写 `HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)`
（输出高）的时候，灯是**灭**的；写 `GPIO_PIN_RESET`（输出低）它才亮。

我去翻数据手册，看到 PC13 这一栏跟别的引脚不一样：

1. **PC13 不在普通 IO 域里，它属于后备域（backup domain）**。STM32F103 里 PC13、PC14、PC15
   由后备电源那一套开关供电，而不是和 PA、PB 一样直接挂在主电源上。
2. 因为这个供电开关的能力有限，数据手册对这三个脚的**输出电流限制得非常死**：
   **不超过 3 mA**，而且明确写了一句话——这三个 IO **不能当电流源用，例如不能直接驱动 LED**。
   手册还要求它们只能工作在 2 MHz 输出速度以内。
3. 对比一下：PA5 这种普通 IO，手册给出的电流能力是它的好几倍（绝对最大值单脚 25 mA，
   实际设计一般取 5～8 mA 就够点亮 LED 了）。

所以板子设计者只有两条路：要么别拿 PC13 点灯（可最小系统板的板载灯就是这么设计的），
要么**换一个电流方向**——不让引脚"吐"电流（source，拉电流），而让引脚"吸"电流（sink，灌电流）：

```text
普通 IO 点灯（PA5，"吐"电流）：
    3.3 V 由引脚提供 ──► PA5 ──► 电阻 ──► LED ──► GND

板载 LED（PC13，"吸"电流）：
    3.3 V（电源）──► 板载电阻 ──► LED ──► PC13 ──► GND（引脚内部拉低）
```

- PC13 输出**低电平**（≈0 V）时：3.3 V 电源和 0 V 引脚之间隔着电阻和 LED，有约 3.3 V 的压差，
  电流从电源经电阻、LED 流进引脚。这个电流是**灌**进芯片的，只要电阻把电流压到 3 mA 以内就安全。
- PC13 输出**高电平**（≈3.3 V）时：LED 阴极电位被抬到接近 3.3 V，LED 两端几乎没有压差，不亮。

这就解释了两件事：为什么板载 LED 是低电平点亮，以及**为什么必须串电阻**——
如果省掉电阻直接让 3.3 V 顶在 LED 上，电流就不再受控了（2.5 节算这笔账）。

顺带记下来：**"高电平点亮"还是"低电平点亮"，不是程序员随便定的，是芯片的电流能力和板子的走线决定的。**
我后面写外接 LED（PA5）时用高电平点亮，写板载灯（PC13）时必须用低电平点亮，两套写法的区别在代码里一眼就能看到。

---

## 2. 原理：它到底是怎么工作的

### 2.1 一个 GPIO 引脚内部有什么

通用输入输出（GPIO, General-Purpose Input/Output）引脚不是一根线，里面是一小套电路：

```text
                              VDD (3.3 V)
                                  │
                    ┌─────────────┴─────────────┐
                    │  上拉（P-MOS，约 40 kΩ）    │
                    │  下拉（N-MOS，约 40 kΩ）    │
                    └─────────────┬─────────────┘
                                  │
   ODR 位 / 复用功能输出 ─────►[ 输出驱动器 ]────┬──────► 芯片引脚（PA5 / PC13）
                              （P-MOS ‖ N-MOS）  │
                                                 │
   IDR 位 ◄────[ 输入缓冲器（施密特触发器）]◄─────┤
                                                 │
   ADC ◄───────── 模拟输入通路 ──────────────────┘
                                                 ▲
                                    ESD 保护二极管（接 VDD / VSS）
```

- **输出驱动器**：一对 MOS 管。上面是 P-MOS（负责把引脚拉向 VDD），下面是 N-MOS（负责拉到 VSS）。
  这两个管子怎么用，就是"推挽"和"开漏"的区别。
- **输入缓冲器**：把引脚上的模拟电压整理成干净的 0/1 给内部读，也就是 `GPIOx_IDR` 寄存器。
  它有施密特触发器（Schmitt trigger）特性，所以带一点迟滞、抗一点噪声。
- **上拉/下拉**：两个几十 kΩ 的弱电阻，只在输入模式下需要时才接通。
- **ODR 位**：`GPIOx_ODR` 的对应位就是"我想输出什么电平"。

### 2.2 GPIO 的 8 种工作模式

参考手册里每个引脚用 **CNF[1:0] + MODE[1:0]** 这 4 个位来配置，一共能配出 8 种模式
（引脚 0～7 的配置位在 `GPIOx_CRL` 里，引脚 8～15 在 `GPIOx_CRH` 里，所以 PC13 属于 CRH 管的那一半）：

| CNF[1:0] | MODE[1:0] | 模式 | HAL 里的写法 | 用在哪、为什么 |
|---|---|---|---|---|
| 00 | 00 | 输入浮空（input floating） | `GPIO_MODE_INPUT` + `GPIO_NOPULL` | 外部电路自己已经给出确定电平的场合（比如信号是另一个 MCU 推挽驱动过来的）。悬空时读到什么是随机的，所以不能拿浮空输入直接接按键 |
| 01 | 00 | 输入上拉（input pull-up） | `GPIO_MODE_INPUT` + `GPIO_PULLUP` | 按键一端接 GND、按下为低：不按时靠内部约 40 kΩ 拉到高电平 |
| 10 | 00 | 输入下拉（input pull-down） | `GPIO_MODE_INPUT` + `GPIO_PULLDOWN` | 按键一端接 3.3 V、按下为高（本仓库按键 PA0 就是这个接法） |
| 11 | 任意 | 模拟输入（analog） | `GPIO_MODE_ANALOG` | ADC 采样必须配这个模式：数字输入缓冲器被断开，否则它会一直从被测电路里抽电流，把分压结果拉偏（第 07 章 ADC 会遇到） |
| 00 | 01/10/11 | 通用推挽输出（GP output push-pull） | `GPIO_MODE_OUTPUT_PP` | 点灯、驱动方向控制脚。高低电平都能由芯片提供 |
| 01 | 01/10/11 | 通用开漏输出（GP output open-drain） | `GPIO_MODE_OUTPUT_OD` | I2C 的 SDA/SCL、需要"线与"的场合、需要输出到 5 V 电平的场合 |
| 10 | 01/10/11 | 复用推挽输出（AF output push-pull） | `GPIO_MODE_AF_PP` | 引脚交给外设控制：USART 的 TX、SPI 的 SCK/MOSI、定时器 PWM 输出（第 04 章） |
| 11 | 01/10/11 | 复用开漏输出（AF output open-drain） | `GPIO_MODE_AF_OD` | 引脚交给外设、而且是开漏：硬件 I2C 外设的 SCL/SDA（第 06 章） |

两点补充：

- **输入模式下，"上拉还是下拉"是靠 ODR 的那一位选的**。因为引脚是输入，ODR 的输出功能用不上，
  硬件就把它复用成上拉/下拉的选择开关（写 1 上拉、写 0 下拉）。所以"输入模式没写 ODR"不代表
  ODR 没用——这也是 `HAL_GPIO_Init()` 会替我们处理这些位的原因。
- **F1 的"复用"是另一套机制**：选成复用模式只说明"这个脚由外设来管"，
  至于是哪个外设接管，要看外设时钟有没有使能、AFIO 的重映射寄存器（`AFIO_MAPR`）怎么设、
  以及外设自己怎么配。F1 没有 F4 那种"每个引脚一个复用功能选择器"，
  这也是为什么 F1 的 `GPIO_InitTypeDef` 里**没有** `Alternate` 这个字段。

### 2.3 重点：推挽（push-pull）和开漏（open-drain）的区别

| | 推挽输出 | 开漏输出 |
|---|---|---|
| 内部结构 | 上面 P-MOS + 下面 N-MOS，两个管子都用 | 只用下面 N-MOS，上面的管子不工作 |
| 能输出的电平 | 高、低都能主动输出 | 只能主动**拉低**；高电平要靠外部上拉电阻把线拉上去 |
| 驱动能力 | 强，可以直接驱动 LED（几十 mA 以内） | 弱，高电平驱动能力取决于上拉电阻，几 kΩ 下只有零点几 mA |
| 多个输出接同一根线 | ❌ 危险：一个推高、一个推低，就是 VDD 到 GND 之间一条低阻通路，两个引脚一起发热 | ✅ 安全：这就是**线与（wired-AND）**——只要有一个设备拉低，线就是低 |
| 典型用途 | LED、方向控制脚、USART TX、SPI、PWM | I2C 的 SDA/SCL、中断/握手信号、电平转换 |

**"线与"为什么是 I2C 的基础**（这里先埋个伏笔，第 06 章会展开）：

I2C 总线上挂着好几个设备，共用 SDA（数据）和 SCL（时钟）两根线，而且**任何设备都可能想说话**
（多主机）或者想"拖住"时钟（时钟拉伸）。如果大家都用推挽输出：

- 甲设备输出高、乙设备输出低，两根 MOS 管直通，等于把 3.3 V 和 GND 短接了一下，
  轻则几十 mA 的瞬时电流、电源被拉塌，重则烧引脚。这不是"概率事件"，是必然发生的。

改成开漏之后，逻辑变成"**谁也抬不高，只能拉低**"：

- 所有设备都不说话 → 线上没有设备拉低 → 上拉电阻把线拉到 3.3 V → 读到 1（总线空闲）。
- 任何一个设备拉低 → 线就是 0 V → 所有设备都读到 0。
- 于是"某个设备发现自己的数据被拉成 0 了"就等价于"有别的设备在同时说话"——
  **这就是 I2C 多主机仲裁（arbitration）的硬件基础**：谁先发出高电平谁就出局，剩下的继续发。

同样因为这个原因，I2C 的 SDA/SCL 在外面必须接上拉电阻（典型 4.7 kΩ）。
上升沿不是芯片驱动出来的，而是**电阻给总线电容充电**充上去的，
所以示波器上看 I2C 的上升沿是一条 RC 曲线而不是方波，这也是 I2C 速率做不高（100 kHz / 400 kHz）的直接原因。🔬

另外，开漏还有一个实用价值：上拉电阻可以接到比 VDD 更高的电压上。
数据手册里标注 **FT（five-volt tolerant，5 V 容忍）** 的引脚，在开漏模式下允许上拉到 5 V，
这是不用额外芯片就能做电平转换的常用办法。

### 2.4 GPIO 的速度等级（2 / 10 / 50 MHz）是干什么的

配置表里 MODE[1:0] 除了"是不是输出"，还兼着选速度，而且顺序是交叉的，容易看错：

| MODE[1:0] | 手册里的说法 | HAL 宏 | 我什么时候用 |
|---|---|---|---|
| 01 | Output mode, max speed 10 MHz | `GPIO_SPEED_FREQ_MEDIUM` | 一般数字信号 |
| 10 | Output mode, max speed 2 MHz | `GPIO_SPEED_FREQ_LOW` | 点灯、按键扫描这种"慢慢变"的引脚 |
| 11 | Output mode, max speed 50 MHz | `GPIO_SPEED_FREQ_HIGH` | SPI 时钟、高频 PWM 这类边沿要陡的场合 |

我第一次看这张表就是按 01→10→11 的顺序当成 2/10/50 读的，读反了。**手册里 `10` 才是 2 MHz。**

这个"速度"到底是什么：

- 它**不是** CPU 频率，也**不是**"这个引脚最快能翻转多少次"的直接答案。
  它配的是输出驱动器的**压摆率（slew rate）**——也就是电平从 0 跳到 1 的过程中，边沿有多陡。
- 速度档位越高，MOS 管开关越快，边沿越陡：好处是能带更大的容性负载、信号眼图更好看；
  代价是**电磁干扰（EMI）更大**、瞬时充放电电流更大（电源上看到的毛刺更明显）、
  长线上更容易出现过冲和振铃。
- 所以原则是**按需选，够用就选低的**。我这一章的引脚只是 1 Hz 和 5 Hz 翻转，
  用 2 MHz 档（`GPIO_SPEED_FREQ_LOW`）绰绰有余，还能少制造干扰。
- PC13 是强制的：手册要求后备域这三个脚只能工作在 2 MHz 以内，所以这里必须是 LOW 档。

> ⚠️ 顺便提醒：F1 系列 HAL 里 `GPIO_SPEED_FREQ_LOW` 就是 **2 MHz** 档。
> 别的系列（比如 F4）这几个宏对应的数值不一样，直接照抄别的芯片的代码会配错档位。

### 2.5 限流电阻应该放在哪，为什么不能省

红色 LED 的正向压降（forward voltage, V_f）大约 **1.8～2.2 V**，取 2.0 V 来算；
板子上的 3.3 V 减去 LED 的 2.0 V，**剩下 1.3 V 必须由电阻吃掉**。

```text
外接 LED（PA5，高电平点亮，"吐"电流）

   PA5 ────┬──── 330 Ω ────┬──── ├▶| ────┬──── GND
           │               │     LED    │
        ≈3.3 V         1.3 V 在这      0 V
                       个电阻上

   电流 I = (3.3 V − 2.0 V) / 330 Ω ≈ 3.9 mA

板载 LED（PC13，低电平点亮，"吸"电流）

   3.3 V ──── 板载电阻 ──── ├▶| ──── PC13
                            LED
   电流路径一样，只是"电源 → 电阻 → LED → 引脚"，
   电阻在阳极侧还是阴极侧都行：同一个串联回路里电流完全相同，
   我习惯把电阻放在电源那一侧（阳极侧）。
```

**为什么必须串电阻：**

LED 是二极管，它的 I-V 曲线在开启电压之后**非常陡**——正向电压再高 0.2～0.3 V，电流就可能翻好几倍。
如果不串电阻，直接让 3.3 V 电源顶在 LED 上，LED 会自己找一个"电流大到把 3.3 V 拉下来"的工作点，
这个电流通常是几十 mA 甚至更大：

- 超过数据手册里单个 IO 的**绝对最大值 25 mA**，引脚内部的 MOS 管和键合线就开始发热；
- LED 本身的额定电流一般只有 20 mA，过流后亮度先掉、然后发黑烧断；
- 同时电源和 IO 都在超额工作，是"一次接错烧两个器件"。

电阻的作用不是"限流"这两个字那么简单，它把**恒压源变成了近似恒流源**：
电源电压的小幅波动，只会在 1.3 V 这个电压上引起很小的电流变化。

**取值算一遍（我实际用的数）：**

| 引脚 | 电流上限（来自手册） | 想要的工作电流 | 计算 | 取标称值 | 实际电流 |
|---|---|---|---|---|---|
| PA5（普通 IO） | 绝对最大 25 mA，设计取 5～8 mA | 5 mA | R = (3.3 − 2.0) / 0.005 = **260 Ω** | 270 Ω | ≈ 4.8 mA |
| PA5（普通 IO） | 同上 | 4 mA | R = (3.3 − 2.0) / 0.004 ≈ 325 Ω | **330 Ω** | ≈ 3.9 mA |
| PC13（后备域） | **3 mA** | 2 mA | R = (3.3 − 2.0) / 0.002 = **650 Ω** | 680 Ω | ≈ 1.9 mA |
| PC13（后备域） | **3 mA** | 按上限 3 mA 算最小电阻 | R = (3.3 − 2.0) / 0.003 ≈ **433 Ω** | ≥ 470 Ω | ≈ 2.8 mA |

两个结论：

1. **PA5 上我用 330 Ω**，电流约 3.9 mA，LED 亮度足够看，离电流上限很远。
2. **PC13 上我不会自己再加 LED**——板子上已经有一颗带电阻的板载灯了；
   如果一定要外接，电阻必须 ≥ 470 Ω，因为超过 3 mA 就出了手册的范围。
   （板载那颗灯的限流电阻具体是多少，要看我这块板的原理图，不同厂家的 Blue Pill 版本不一样。
   按 470 Ω 估算是 2.8 mA，正好卡在 3 mA 以内。🔬 上机后我会实测一下这个阻值补在这里。）

### 2.6 为什么点灯要用非阻塞延时，不能用 `HAL_Delay()`

`HAL_Delay(500)` 是**阻塞（blocking）**的：它的实现就是"读一下当前毫秒数，然后在原地
转圈等到毫秒数变化超过 500 才返回"。这 500 ms 里 CPU 哪儿也去不了。

```text
HAL_Delay(500) 阻塞式（我最初的写法）：

 时间 →  0 ms                          500 ms                  1000 ms
 循环     |←──── 卡在 HAL_Delay 里 ────→|←──── 又卡 500 ms ────→|
 状态     这 500 ms 里什么都不做，也不能做

HAL_GetTick() 非阻塞式（本章的写法）：

 时间 →  0 ms     1 ms     2 ms   ...   500 ms   ...   1000 ms
 循环    每轮都回来一次，每次只问一句"到 500 ms 了吗？"
 状态    到时间才翻转一次引脚，其余时间循环继续往下走
```

两种写法的差别：

- `HAL_Delay` 把"等待"写死在函数里，**你没法在这段时间里插别的任务**。
- 用 `HAL_GetTick()` 判断，等待变成了主循环里的一个 if：主循环每轮都跑完，
  以后加串口收发、ADC 采样、PID 计算，都能塞进同一个循环，谁也不会把别人卡住。

必须说明白的一点（免得自己骗自己）：我这段代码里主循环是**忙等（busy-wait）**的，
没有 `HAL_Delay` 之后它会以最快速度空转，CPU 占用率接近 100%。
它比 `HAL_Delay` 强的地方不是"省 CPU"，而是**循环结构上没有把时间焊死**，
任务可以叠加。等到第 04 章学了定时器中断，才会把"翻转引脚"这件事挪到中断里，
主循环连轮询都不用做。

**为什么机器人代码里尤其不能用 `HAL_Delay()`：**

- RM 的车里，控制周期通常是 **1 ms（1 kHz）** 这个量级（底盘功率控制、姿态解算都按这个节奏跑）。
  随手写一个 `HAL_Delay(10)`，一轮循环的时间预算就被吃光了，控制周期从 1 ms 掉到 10 ms 以上，
  而且它保证的是"**至少** 10 ms"，实际更长。
- 更糟的是它会把别的任务一起推迟：串口收数据晚了，丢帧；ADC 采样晚了，闭环相位滞后。
- 还有一个隐蔽的死锁：`HAL_Delay` 依赖 SysTick 中断里去递增的毫秒计数器，
  在**中断服务函数里**调用它，如果这个中断的优先级不低于 SysTick 的，
  SysTick 就没法抢占它、计数器不再增加，`HAL_Delay` 会**永远等下去**（6.2 节详说）。

### 2.7 `HAL_GPIO_WritePin` / `HAL_GPIO_TogglePin` 和直接写寄存器的区别

先说这三个寄存器（都在 RM0008 第 9.2 节）：

- `GPIOx_ODR`（输出数据寄存器）：每一位对应一个引脚的输出电平，读写都是"当前电平"。
- `GPIOx_BSRR`（置位/复位寄存器）：**低 16 位写 1 → 对应引脚输出高；高 16 位写 1 → 对应引脚输出低；写 0 的位不受影响。**
- `GPIOx_BRR`（复位寄存器）：只有"输出低"这一半功能。

HAL 函数其实就是这几条寄存器操作的封装：

| 我写的代码 | HAL 内部实际做的事 | 是不是原子操作 | 什么时候该用 |
|---|---|---|---|
| `HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)` | `GPIOC->BSRR = GPIO_PIN_13;` | ✅ 一次 32 位写，原子 | 绝大多数场合，可读性最好 |
| `HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)` | `GPIOC->BRR = GPIO_PIN_13;` | ✅ 原子 | 同上 |
| `HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)` | 先读一次 `GPIOC->ODR`，再往 BSRR 里**同时**写"该置位的位"和"该复位的位" | 写是原子的，**读不是** | 翻转一个脚很方便；但主循环和中断同时翻转同一个脚可能丢一次翻转 |
| `GPIOC->ODR \|= (1 << 13);` | 读 ODR → 改一位 → 写回 ODR（读-改-写三步） | ❌ **不是原子** | 不推荐。中间任何一刻被打断，那次修改就丢了 |

**"原子（atomic）"在这里是什么意思：**

`GPIOC->ODR |= (1 << 13);` 编译出来是三条指令：读出整个 32 位 ODR、把 bit13 置 1、写回。
如果在这三步中间来了一个中断，中断里也改了同一个端口的别的引脚（比如 PA5 换成 PC14），
中断里那笔修改会在主程序把旧值写回去的瞬间**被覆盖掉**——现象就是"偶尔有一个引脚该动没动"，
而且这种 bug 很难复现。

`BSRR` 从设计上绕开了这个问题：它**不需要你先把当前值读回来**，
你只要写"哪一位要变成什么"，硬件去改对应的那一位，写 0 的位它一个字都不动。
一次 32 位写是一条指令，不可能被撕开，所以天生是原子的。

`BSRR` 还有一个用了才知道好的性质：**一次写可以让多个引脚在同一时刻变化**。
比如底盘控制里要"把 A 脚拉高的同时把 B 脚拉低"，写成一条 `GPIOx->BSRR = ...`
两个脚是同一次总线写里生效的，没有先后时间差；如果分两次调 `HAL_GPIO_WritePin`，
中间就会隔上几十纳秒的"两个脚都处于危险状态"的窗口。

至于速度：`HAL_GPIO_WritePin` 多了一次函数调用（几个时钟周期），
在 72 MHz 下是几十纳秒的量级，点灯、按键完全无所谓；
真要在高频中断里翻转引脚，才会直接写 `BSRR`。
**我的原则是：先用 HAL 写清楚，等真的成了瓶颈（有测量数据）再换寄存器。**

---

## 3. 手册依据

下面每条结论我都写清是在哪份文档的哪一节看到的。
**我不写具体页码**——不同版本（我手上是 RM0008 Rev 21）和不同排版的中文译本页码对不上，
写章节标题最保险，也方便别人自己去核对。

| 结论 | 出处 |
|---|---|
| GPIO 有 8 种工作模式，由 `CNF[1:0]` + `MODE[1:0]` 决定；`GPIOx_CRL` 管引脚 0～7，`GPIOx_CRH` 管引脚 8～15 | RM0008 参考手册 第 9 章「General-purpose and alternate-function I/Os (GPIO and AFIO)」9.1 节「GPIO functional description」中的**端口位配置表**（Port bit configuration table） |
| 速度档是 `MODE=10 → 2 MHz`、`01 → 10 MHz`、`11 → 50 MHz`（顺序是交叉的） | 同上那张配置表 |
| 输入模式下由上拉/下拉电阻提供电平，典型阻值约 40 kΩ | RM0008 第 9 章 9.1 节 GPIO 功能描述；数值在 STM32F103x8/xB 数据手册 I/O characteristics 表的 `R_PU` / `R_PD` 一栏 |
| `GPIOx_ODR` / `BSRR` / `BRR` / `IDR` / `CRL` / `CRH` / `LCKR` 的位定义；BSRR 高 16 位复位、低 16 位置位 | RM0008 第 9.2 节「GPIO registers」：9.2.1 GPIOx_CRL、9.2.2 GPIOx_CRH、9.2.3 GPIOx_IDR、9.2.4 GPIOx_ODR、9.2.5 GPIOx_BSRR、9.2.6 GPIOx_BRR、9.2.7 GPIOx_LCKR |
| 复位后所有 GPIO 都是**浮空输入**（CRL/CRH 复位值 `0x4444 4444`） | 同上 9.2.1 / 9.2.2 的"复位值"一栏 |
| PC13、PC14、PC15 由后备域供电，输出电流**不超过 3 mA**，输出速度限制在 **2 MHz** 以内，且**不能作为电流源**（手册原文举例就是"不能驱动 LED"） | STM32F103x8/xB 数据手册「Pinouts and pin descriptions」一节中 PC13/PC14/PC15 的脚注；3 mA 这个数值在同一份手册的 I/O characteristics 表 |
| 单个 IO 灌电流 / 拉电流的绝对最大值 25 mA，全部 IO 合计 150 mA | STM32F103x8/xB 数据手册「Absolute maximum ratings」表（这是"绝对不能超过"的红线，不是设计目标） |
| GPIO 挂在 APB2 总线上，所以 GPIO 的翻转速度受 APB2 时钟影响 | RM0008 第 9 章开头的外设总线挂载说明 + 第 8 章「Reset and clock control (RCC)」中 `RCC_APB2ENR` 的 `IOPAEN`/`IOPBEN`/`IOPCEN`/`IOPDEN` 位 |
| 72 MHz 时 Flash 需要 2 个等待周期（`FLASH_LATENCY_2`） | STM32F103x8/xB 数据手册「Flash memory」一节里的等待周期表（按 CPU 频率选 LATENCY） |

下面两条不是手册，是 HAL 库源码里的实现，我也一并列出来，因为它们直接决定了我怎么理解这几行代码：

| 结论 | 出处 |
|---|---|
| `HAL_GPIO_WritePin` 是往 `BSRR`（置位）或 `BRR`（复位）写；`HAL_GPIO_TogglePin` 是先读 `ODR` 再往 `BSRR` 同时写置位和复位两半 | CubeF1 HAL 源码 `stm32f1xx_hal_gpio.c` |
| F1 的 `GPIO_SPEED_FREQ_LOW` 对应 2 MHz 档 | `stm32f1xx_hal_gpio.h` 里的宏定义 |

---

## 4. 完整代码

完整文件在 [`code/01-blink/main.c`](../code/01-blink/main.c)。
**分区说明**：`USER CODE BEGIN` / `USER CODE END` 成对注释之间的内容是我写的，
重新用 CubeMX 生成代码时这部分会被保留；区间外是 CubeMX 生成的框架（我按它的样式整理成同一份文件）。
为了不让正文太长，下面这份略过了几个 CubeMX 生成但内容为空的区块（Private macro、`USER CODE BEGIN 0/4`），
文件 [`main.c`](../code/01-blink/main.c) 里是完整的。

```c
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

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
       也不会因为某个灯要等 500 ms 而让其他任务停摆 */
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; /* SYSCLK = 72 MHz */
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         /* HCLK   = 72 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           /* PCLK1  = 36 MHz（上限 36 MHz） */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           /* PCLK2  = 72 MHz（GPIO 在这条总线上） */

  /* 72 MHz 时 Flash 需要 2 个等待周期，否则取指会出错 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable ------------------------------------------------*/
  /* 这三行是 CubeMX 自动生成的。不使能端口时钟，端口寄存器的写操作全部无效，
     引脚不会有任何反应 */
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
  GPIO_InitStruct.Pull  = GPIO_NOPULL;            /* 输出模式下上下拉不起作用 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* F1 的 HAL 里 LOW = 2 MHz 档。
                                                     PC13 属于后备域，手册要求限制在
                                                     2 MHz 以内，点灯也不需要更快 */
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
  /* 出错时把 CPU 停在这里，方便用调试器看调用栈 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 用户可以用 printf 把 file 和 line 打出来，定位是哪个参数配错了 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
```

预期现象（🔬 待上机确认）：

> 先说明一处"故意的写法"：这份代码里**一行 `HAL_Delay()` 都没有**。
> 不是忘了写，是刻意的——原因见 2.6 节：阻塞延时会把以后要塞进主循环的
> 串口收帧、ADC 采样、PID 计算全部跟着推迟，在 1 ms 控制周期的机器人代码里这是致命的。

| 观察点 | 预期现象 |
|---|---|
| 板载 LED（PC13） | 亮 500 ms、灭 500 ms 交替，1 s 一个完整周期 |
| 外接 LED（PA5） | 亮 100 ms、灭 100 ms 交替，0.2 s 一个完整周期 |
| 上电瞬间 | 两个灯都是灭的，等第一个半周期结束才第一次点亮 |
| 两者关系 | 互不干扰：PA5 闪 5 次的这段时间里，PC13 只翻转 1 次 |

最硬的验证方式是拿逻辑分析仪或者示波器量 PA5，预期一个完整方波周期 **200 ms**、占空比约 **50 %**。🔬

---

## 5. 逐段解释

### 5.1 `GPIO_InitTypeDef` 的每个字段是什么意思

F1 的 HAL 里，这个结构体只有 **4 个成员**（F4 上还有一个 `Alternate`，F1 没有，原因见 2.2 节）：

| 字段 | 我填的值 | 含义 | 填错的后果 |
|---|---|---|---|
| `Pin` | `GPIO_PIN_13` / `GPIO_PIN_5` | 要配置哪个引脚，可以用 `\|` 一次配多个 | 配到别的脚上，那个脚不动 |
| `Mode` | `GPIO_MODE_OUTPUT_PP` | 8 种工作模式里选一种（对应 CNF/MODE 位） | 选成输入，`HAL_GPIO_WritePin` 写了也没反应 |
| `Pull` | `GPIO_NOPULL` | 输入模式下要不要开内部上拉/下拉 | 输入模式配错了会读到随机电平 |
| `Speed` | `GPIO_SPEED_FREQ_LOW` | 输出驱动器的压摆率档位（2 / 10 / 50 MHz） | PC13 配成高速档是超出手册限制的 |

一个容易忽略的点：**`Speed` 只在输出模式下有意义**，输入模式下这几位（MODE[1:0] = 00）
被固定成输入，HAL 会忽略 `Speed`。我一开始以为 `Speed` 是"采样速度"，其实不是。

### 5.2 为什么必须开时钟：`__HAL_RCC_GPIOC_CLK_ENABLE()`

这一行是个宏，展开大致是：

```c
/* 宏展开后大致等于 */
SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);   /* 把 APB2ENR 的 IOPCEN 位置 1 */
```

`RCC_APB2ENR` 是 APB2 外设的时钟使能寄存器，`IOPCEN` 就是"GPIO C 口时钟使能"那一位。

**为什么不开时钟引脚就完全不动：**

- STM32 为了省电，所有的外设（包括 GPIO 端口）默认**时钟是关掉的**。
  没有时钟，端口内部的寄存器根本收不到 APB 总线写进来的数据，你写 `GPIOC->BSRR`，
  硬件直接忽略。
- 表现出来就是最让人迷惑的一类 bug：**编译通过、下载成功、程序在正常运行
  （另一个端口的 LED 还闪得好好的），只有这个端口的引脚纹丝不动。**
  因为程序真的一点错都没有，只是那个外设没通电（时钟就是数字外设的"电"）。
- 这一行是 CubeMX 自动生成的，所以用 CubeMX 的时候很难踩到；
  但只要手写初始化、或者拷贝别的工程改引脚（把 PA 改成 PC 却忘了加 `__HAL_RCC_GPIOC_CLK_ENABLE()`），
  立刻就会踩（6.1 节）。

### 5.3 为什么先写电平、再配模式

`MX_GPIO_Init()` 里这两行的顺序是 CubeMX 故意排的，值得看一眼：

```c
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* 先把输出寄存器写成"灭" */
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                /* 再把引脚切成输出 */
```

- 复位之后 `GPIOx_CRH` 的值是 `0x4444 4444`，也就是所有引脚都是**浮空输入**，
  此时 PC13 是高阻态，LED 不亮。
- `HAL_GPIO_Init()` 一执行，PC13 就变成推挽输出，输出电平**立即等于 ODR 里的值**。
  而复位后的 ODR 是全 0，也就是"低电平"——对低电平点亮的板载 LED 来说就是"**亮**"。
- 所以如果顺序反了（先 Init 再 Write），从上电到执行 Write 的那一小段时间里，
  板载 LED 会先亮一下。时间可能只有几微秒（也可能几毫秒，取决于中间还有多少初始化代码），
  但在示波器上一定看得见。🔬

这也解释了我为什么在 `main()` 里又写了一次 `HAL_GPIO_WritePin`（`USER CODE BEGIN 2` 那两行）：
初始化函数负责"引脚一使能就是灭的"，主循环前那两行负责"进入业务逻辑时状态明确"，
两处不冲突，都是为了让初始状态**可预测**。

### 5.4 时间判断为什么要写成 `(now - tick) >= 周期`

```c
if ((uint32_t)(now - tick_pc13) >= LED_PC13_HALF_PERIOD_MS)
```

- `HAL_GetTick()` 返回的是 `uint32_t` 的毫秒计数，累加到 `0xFFFFFFFF` 之后会**回绕到 0**，
  也就是连续运行约 4294967296 ms ÷ 86400000 ms/天 ≈ **49.7 天**后归零。
- 写成 `now >= tick + 周期` 的话，在 `tick` 接近 `0xFFFFFFFF` 时 `tick + 周期` 会先溢出成一个小数字，
  于是刚数到几百毫秒就误判成"时间到了"，灯会突然乱闪。
- 无符号数相减不受回绕影响：`now - tick` 算出来永远是"从 tick 到现在真正过了多少毫秒"
  （前提是时间间隔小于 2³² ms，这个条件在正常代码里一定满足）。
  这是嵌入式里比较时间戳的标准写法，值得记住。

### 5.5 两个灯为什么互不干扰

`while(1)` 里没有任何等待。每轮循环做三件事：读一次时间、判断 PC13 到时间没有、判断 PA5 到时间没有。
两个灯的判断各用各的时间基准（`tick_pc13` / `tick_pa5`），谁到时间谁翻转，互不影响。

代价：这一轮循环会以最快速度空转。主循环每轮大概几十条指令，在 72 MHz 下每秒要跑几十万到上百万轮
（粗估，具体数字要看反汇编结果），CPU 基本上一直在做这两个 if。
真项目里不会这么写——第 04 章会用定时器中断来干这件事，
但那是"更好的实现"，不是"必须马上改的错误"。

顺带说一下 GPIO 翻转速度的上限：GPIO 挂在 APB2 上，一次 `BSRR` 写要经过 APB2 总线，
在 APB2 = 72 MHz 下粗算一次写约 2 个总线周期（≈ 27.8 ns），
所以**用软件翻转引脚**的方波周期理论下限是几十纳秒的量级、对应十几 MHz 的上限，
再高就得靠硬件的定时器 PWM 输出（第 04 章）。这个理论下限还只是总线层面，
实际会被循环里的指令数拖慢——想要真实数字必须用示波器量。🔬

### 5.6 `SystemClock_Config()` 和点灯的关系

看起来配置时钟跟点灯没关系，其实有关系：

- `HAL_RCC_OscConfig()` 打开外部晶振 HSE（8 MHz），
  `PLL.PLLMUL = RCC_PLL_MUL9` 让 PLL 输出 8 MHz × 9 = **72 MHz**，作为 SYSCLK。
- `APB2CLKDivider = RCC_HCLK_DIV1` → APB2 = **72 MHz**，GPIO 就在这条总线上。
  这是 GPIO 能翻转多快的上限来源。
- `FLASH_LATENCY_2` 必须配对：72 MHz 已经超过 Flash 在 0 等待周期下的工作频率，
  不设等待周期会出现取指错误（现象是程序跑飞或者结果莫名其妙）。
- `APB1CLKDivider = RCC_HCLK_DIV2` → APB1 = 36 MHz，因为 F103 的 APB1 上限就是 36 MHz，
  后面第 04 章的定时器 TIM2/TIM3/TIM4 就挂在这条总线上（所以定时器时钟要按 ×2 算，第 04 章细说）。

---

## 6. 我踩的坑

先说清楚这一节的状态：**我还没上机**，所以硬件层面我严格来说还没"踩"到坑。
下面是**两件**我在写这一章的代码时真的想岔过的事（6.3、6.4），
以及**三件**我知道一定会发生、所以通电之前先查清楚的事（6.1、6.2、6.5）。
哪一条属于哪一类，我在小标题里标了。

| # | 坑 | 一句话原因 | 我的状态 |
|---|---|---|---|
| 6.1 | 忘了使能 GPIO 时钟，引脚毫无反应 | 端口时钟没开，寄存器写入被硬件忽略 | 🔬 通电前必查第一条 |
| 6.2 | 在中断里调用 `HAL_Delay()` 会死等 | `HAL_Delay` 依赖 SysTick 中断递增计数，而中断没法被自己同级的 SysTick 抢占 | 🔬 机制来自 HAL 源码与 CM3 抢占规则，待实测 |
| 6.3 | 上电瞬间 LED 状态不确定 | 复位后 ODR = 0，PC13 一切成输出就立刻是低电平 = 亮 | 写代码时撞到（顺序问题） |
| 6.4 | 想用两个 `HAL_Delay` 让两个灯分别闪 | 阻塞延时不可能同时满足两个不同的周期 | ✅ 写代码时撞到，已改 |
| 6.5 | LED 不串限流电阻直接接 3.3 V，会烧 | LED 的 I-V 曲线很陡，电压没人"管"电流就失控 | 🔬 我还没接线，先把账算清了 |

### 6.1 忘了使能 GPIO 时钟（🔬 通电前必查）

- **现象**：程序编译通过、下载成功、另一个端口的灯正常闪，但这个引脚一点反应都没有。
  用万用表量引脚电压，一直是复位后的状态。
- **原因**：`RCC_APB2ENR` 里对应的 `IOPxEN` 位没置 1，端口没有时钟，
  APB 总线上的写操作被外设直接忽略。数字外设的时钟相当于它的"电源"。
- **怎么定位**：这类问题的特征是"**代码看着完全正确**"，所以要从外设的"有没有开"查起，
  而不是从"代码逻辑对不对"查起。最快的两步：
  1. 在调试器里看 `RCC->APB2ENR` 的对应位是不是 1；
  2. 拿一个已知能工作的示例（CubeMX 空工程 + 同样引脚）对着比初始化函数。
- **怎么改**：在配置引脚**之前**加 `__HAL_RCC_GPIOC_CLK_ENABLE();`。
  用 CubeMX 生成的工程里这行一定会出现，手写或者改引脚的时候最容易漏。

### 6.2 `HAL_Delay()` 在中断里会永远等下去（🔬 待实测）

- **现象**：主循环里跑得好好的程序，把 `HAL_Delay(100)` 挪进某个中断服务函数之后，
  整个程序像死了一样，主循环不再执行。
- **原因**：`HAL_Delay` 的实现是"记下当前毫秒数，然后原地等这个数变化到目标值"，
  而那个毫秒数（`uwTick`）**只在 SysTick 中断里递增**。
  Cortex-M3 的中断抢占规则是"优先级数值更小的可以抢占比它大的"，
  **优先级相同的中断之间不能互相抢占**。
  如果调用 `HAL_Delay` 的这个中断优先级不低于 SysTick 的，SysTick 就进不来，
  `uwTick` 永远不变，`HAL_Delay` 里的 `while` 转不出来。
  （CubeMX 默认给外设中断的优先级是 0，SysTick 的优先级由
  `stm32f1xx_hal_conf.h` 里的 `TICK_INT_PRIORITY` 决定，
  这两者凑在一起正好满足"不被抢占"的条件。）
- **怎么定位**：在中断里翻转一个测试引脚，如果它**只翻转了一次就再也不动**，
  基本就是这个坑；或者用调试器暂停，看程序计数器是不是停在 `HAL_Delay` 的 `while` 里。
- **怎么改**：中断服务函数里**只置标志位、只做最短的活**，延时交给主循环用
  `HAL_GetTick()` 判断；确实需要"过一会儿再执行"就记一个时间戳，在主循环里比。

### 6.3 上电瞬间 LED 状态不确定（写代码时撞到）

- **现象**：灯正常亮，但上电的第一瞬间板载 LED 会先亮一下才进入"灭"的状态。
- **原因**：复位后 `GPIOx_CRH` 是 `0x4444 4444`（浮空输入），ODR 是 0。
  只要 `HAL_GPIO_Init()` 把 PC13 配成推挽输出，它就立刻输出 ODR 里的 0，
  也就是低电平——板载 LED 是低电平点亮的，于是立刻亮。
  如果写成"先 Init、后 WritePin"，这个亮的时间就等于两行代码之间执行了多久。
- **怎么定位**：拿示波器（或者干脆用手机慢动作拍灯）看第一个电平出现在什么时候。
  代码层面就是检查 `MX_GPIO_Init()` 里 WritePin 和 Init 的先后。
- **怎么改**：把"设置输出电平"放在 `HAL_GPIO_Init()` **之前**（CubeMX 生成的就是这个顺序），
  并且在进主循环前再显式写一次灭（本章代码 `USER CODE BEGIN 2` 那两行）。
  这是一个"现象很小、但说明你有没有在意初始状态"的细节。

### 6.4 用两个 `HAL_Delay` 让两个灯分别闪（写代码时撞到，已改）

- **我一开始的想法**：板载灯闪 1 Hz、外接灯闪 5 Hz，那就
  `HAL_Delay(500); Toggle(PC13); HAL_Delay(100); Toggle(PA5);` 循环里轮着来。
- **为什么不行**：`HAL_Delay` 是阻塞的，CPU 在等待期间不做别的事。
  上面这段代码里两次延时是**串起来**的，一轮循环至少 600 ms，
  于是两个灯的实际周期都会变成 600 ms 的整数倍关系，谁也得不到想要的频率。
  想让两个灯各有各的周期，**计时必须是"在循环里判断"而不是"在循环里等待"**。
- **怎么改**：就是本章的写法——给每个灯记一个"上次翻转时刻"，
  每轮循环检查 `HAL_GetTick()` 有没有走过各自的半周期。
  这个坑的价值在于：它让我第一次真正体会到"阻塞"和"非阻塞"的区别，
  以及为什么机器人代码里到处都要避免阻塞。

### 6.5 限流电阻不能省（🔬 我还没接线，先把账算清了）

- **正确的接法**：`PA5 → 330 Ω → LED 阳极 → 阴极 → GND`，
  电流 `(3.3 V − 2.0 V) / 330 Ω ≈ 3.9 mA`。
  如果要在 PC13 上外接 LED，电阻必须 **≥ 470 Ω**，因为那个脚的电流上限是 3 mA。
- **省掉电阻会怎样**：LED 的正向压降对电流极其敏感，电压再高 0.2～0.3 V，电流可能翻几倍。
  没有电阻时 3.3 V 直接压在 LED 上，工作点会跑到几十 mA 以上：
  超过单个 IO 的绝对最大值 25 mA 会烧引脚，超过 LED 自己约 20 mA 的额定电流会烧灯。
- **怎么"定位"**：这不是要等烧了才明白的事，可以在通电前算清楚（就是 2.5 节那张表）。
  想亲眼看看这条 I-V 曲线有多陡，可以做一个安全的实验：用可调电源（限流设在 20 mA）
  串一个 1 kΩ 电阻接到 LED，从 1.6 V 开始每 0.1 V 记录一次电流，画成表。🔬 这个实验我打算上机后做，做完把数据补到这里。
- **我现在的做法**：接线之前先在纸上把每颗 LED 的电阻算出来，再对照数据手册的电流上限，
  两块都过关才通电。

---

## 7. 复现步骤

前提：已经按[第 01 章](01-环境搭建.md)装好 STM32CubeMX 6.x 和 Keil MDK-ARM 5.x，ST-Link 驱动正常。

### 一、CubeMX 里配置

1. 新建工程，`File → New Project`，在 `Part Number` 搜索框输入 `STM32F103C8`，
   选中 `STM32F103C8Tx`，点 `Start Project`。
2. 左侧 `System Core → SYS`：`Debug` 选 **Serial Wire**（不选的话 SWD 烧录会被关掉）。
3. 左侧 `System Core → RCC`：
   - `High Speed Clock (HSE)` 选 **Crystal/Ceramic Resonator**（板上有 8 MHz 晶振）；
   - `Low Speed Clock (LSE)` 保持 Disable。
4. 打开上方 **Clock Configuration** 标签页：
   - `Input frequency` 填 **8 MHz**；
   - `PLL Source Mux` 选 **HSE**；
   - `PLLMul` 选 **X9**；
   - `System Clock Mux` 选 **PLLCLK**；
   - 确认 `HCLK (MHz)` 显示 **72**，`APB1 Prescaler` 是 **/2**（36 MHz），`APB2 Prescaler` 是 **/1**（72 MHz）。
5. 回到 **Pinout & Configuration** 标签页，在芯片图上找到 **PC13**，左键点击选 **GPIO_Output**；
   右键 → `Enter User Label`，输入 `LED_BUILTIN`。
6. 同样操作 **PA5** → **GPIO_Output**，User Label 填 `LED_EXT`。
7. 在左侧 `System Core → GPIO` 里，点开 `GPIO` 列表逐行设置：

   | Pin | GPIO output level | GPIO mode | GPIO Pull-up/Pull-down | Maximum output speed | User Label |
   |---|---|---|---|---|---|
   | PC13 | **High** | Output Push Pull | No pull-up and no pull-down | **Low** | LED_BUILTIN |
   | PA5 | **Low** | Output Push Pull | No pull-up and no pull-down | **Low** | LED_EXT |

   （PC13 的初始电平写 High，就是因为它低电平点亮、初始要灭；PA5 写 Low 同理。）
8. `Project Manager` 标签页：
   - `Project Name` 填 `01-blink`，`Project Location` 选自己的目录；
   - `Toolchain/IDE` 选 **MDK-ARM V5**；
   - `Code Generator` 里勾上 `Generate peripheral initialization as a pair of .c/.h files`
     会把 GPIO 初始化分出去（不勾也行，那样 `MX_GPIO_Init()` 直接放在 `main.c` 里，就是本章的样式）。
9. 点右上角 **GENERATE CODE**。

### 二、把本章的代码填进去

10. 用 Keil 打开生成的工程（CubeMX 会问 `Open Project`，直接点是）。
11. 打开 `Core/Src/main.c`，把本章第 4 节里三个 `USER CODE` 区的内容填进去：
    - `USER CODE BEGIN 1`：三个 `uint32_t` 变量；
    - `USER CODE BEGIN 2`：读初始 tick + 把两个灯写成灭；
    - `USER CODE BEGIN 3`：`while(1)` 里的两段定时判断。
    同时把两个半周期的宏放进 `USER CODE BEGIN PD`。
    `MX_GPIO_Init()` 是生成的，不用手写，但要确认里面有 `__HAL_RCC_GPIOC_CLK_ENABLE();` 和
    `__HAL_RCC_GPIOA_CLK_ENABLE();` 两行。

### 三、接线（**先断电**）

12. 拔掉 USB 线，让板子完全断电再接线。
13. 外接 LED 一条腿接 **PA5**，另一条腿串 **330 Ω** 电阻到 **GND**。
    注意极性：LED 长脚（阳极）朝 PA5 一侧，短脚（阴极）朝 GND 一侧。
    接反了不会烧（反向耐压够），但不会亮。
14. 确认面包板电源轨没有被短路，再插上 USB。

### 四、编译、烧录

15. Keil 里 `Project → Build Target`（快捷键 **F7**），确认 `0 Error(s)`。
16. `Options for Target → Debug` 选 **ST-Link Debugger**；
    点右边 `Settings` → `Flash Download` 勾上 **Reset and Run**（下载完自动复位运行）。
17. `Flash → Download`（快捷键 **F8**）烧录。
18. 现象对照第 4 节那张表；如果不对，按第 6 节的顺序排查：
    先查时钟使能 → 再查引脚号和电平极性 → 最后查接线。

---

## 8. 自测题（面试可能这么问）

**Q1. 推挽和开漏有什么区别？I2C 为什么要用开漏？**

推挽用上下两个 MOS 管，高低电平均由芯片主动输出，驱动能力强；
开漏只有下面那个管工作，**只能主动拉低**，高电平必须靠外部上拉电阻把线拉起来。

I2C 用开漏有三个原因：① 总线上多个设备共用 SDA/SCL，开漏构成"线与"——
任何一个设备拉低总线就是低，多主机同时发送时用这个特性做**仲裁**；
② 如果用推挽，两台设备一个输出高一个输出低就是 VDD 到 GND 的低阻通路，会烧引脚；
③ 开漏的上拉电阻可以接到比 VDD 更高的电压，方便做电平转换（FT 引脚可上拉到 5 V）。
代价是上升沿由电阻给总线电容充电形成，是 RC 曲线而不是方波，所以速率做不高。

**Q2. 为什么板载 LED 是低电平点亮？**

因为它在 PC13 上，而这个脚属于**后备域**。数据手册对 PC13/PC14/PC15 限制得很死：
输出电流不超过 **3 mA**，输出速度限制在 **2 MHz** 以内，并且明确写了这三个脚
**不能作为电流源**（原文举例就是不能用来驱动 LED）。
所以板子把 LED 的阳极经电阻接 3.3 V、阴极接 PC13，让引脚**灌电流**（sink）而不是拉电流。
引脚拉低时 LED 两端有 3.3 V 压差所以点亮，拉高时压差接近 0 所以灭。
顺带一个推论：如果要在 PC13 上自己外接 LED，限流电阻必须 ≥ (3.3 − 2.0) / 0.003 ≈ 433 Ω，
取 470 Ω 以上。

**Q3. `BSRR` 和 `ODR` 有什么区别？为什么推荐 `BSRR`？**

`ODR` 是输出数据寄存器，每一位直接对应引脚电平，但改一位必须"读出来 → 改 → 写回去"，
是三步操作，**中间被打断就会丢掉别的位的修改**（不是原子操作）。
`BSRR` 是置位/复位寄存器：低 16 位写 1 让对应引脚输出高，高 16 位写 1 让对应引脚输出低，
写 0 的位不受影响。**一次 32 位写就能改任意多个引脚，而且不需要先读回当前值，天生原子。**
还有一个好处是可以用同一次写让多个引脚在同一时刻变化，避免分两次写带来的时间差。
`HAL_GPIO_WritePin` 内部用的就是 BSRR/BRR。

**Q4. 为什么机器人代码里不能用 `HAL_Delay()`？**

① 它是阻塞的：等待期间 CPU 不能做别的事，把整个循环的时间预算焊死；
② RM 的控制周期通常在 **1 ms（1 kHz）** 量级，一个 `HAL_Delay(10)` 就把周期拉到 10 ms 以上，
   而且是"至少 10 ms"、抖动不可控，功率控制、姿态解算、串口收帧全被拖慢；
③ 在中断服务函数里调用它可能**永远等下去**：`HAL_Delay` 等的是只在 SysTick 中断里递增的计数，
   而优先级相同或更高的中断无法被 SysTick 抢占。
正确做法是记录时间戳、在主循环里用 `HAL_GetTick()` 判断，或者干脆用定时器中断做周期任务。

**Q5. GPIO 的速度等级（2 / 10 / 50 MHz）配的是什么？**

配的是输出驱动器的**压摆率**，也就是电平跳变的边沿有多陡，**不是 CPU 频率，也不是"引脚的时钟"**。
档位越高边沿越陡，能带更大的容性负载，但电磁干扰和瞬时电源电流也越大，
长线上更容易过冲振铃。所以按需选、够用就选低的：点灯用 2 MHz 档
（F1 的 HAL 里对应 `GPIO_SPEED_FREQ_LOW`，注意 F1 和其他系列的宏取值不一样），
SPI 时钟、高频 PWM 才用 50 MHz 档。
PC13 是强制的：手册要求后备域这三个脚的输出速度不能超过 2 MHz。

**Q6. 外接 LED 的限流电阻怎么算？**

先查这颗 LED 的正向压降（红色约 1.8～2.2 V，取 2.0 V），再定工作电流（一般 3～5 mA 就够亮），
然后 `R = (V_电源 − V_f) / I`。例如 PA5 接红色 LED、想跑 5 mA：
`R = (3.3 − 2.0) / 0.005 = 260 Ω`，取标称值 **270 Ω**（实际约 4.8 mA）或略保守取 **330 Ω**（约 3.9 mA）。
最后一定要回头对照手册的电流上限（单脚绝对最大 25 mA，PC13 只有 3 mA）确认没超。

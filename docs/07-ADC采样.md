# 07 · ADC 采样：从"读到一个数"到"知道这个数准不准"

> **本章目标**：说清 12 位 ADC 的一个数到底代表多少伏（LSB 怎么算、为什么除 4096 不除 4095），会按数据手册的参数手算采样时间（这是最实用的一条），会用三种软件滤波把会跳的读数稳住，并知道每种滤波各自要付出多少延迟代价。
> **前置章节**：[06 I2C 总线与 0.96 寸 OLED](06-I2C与OLED.md)
> **硬件**：STM32F103C8T6 最小系统板、ST-Link V2、10k 电位器、100nF 电容、USB-TTL 模块
> **代码**：[`code/05-adc`](../code/05-adc)
> **状态**：🔬 待上机验证

## 1. 先想清楚一个问题

串口助手上打出来一行：

```text
raw = 2048
```

我第一反应是"这个简单，乘一下就行"：

```text
电压 = 2048 / 4096 × 3.3V = 1.650V
```

数字是算出来了，但我立刻发现自己答不出三个问题：

1. **分母为什么是 4096，不是 4095？** 4096 和 4095，我在不止一个教程里都见过，两个都有人用，谁对？
2. **3.3V 是哪来的？** 如果我这块板子的 3.3V 实际量出来是 3.28V，那我算出来的 1.650V 是不是也偏了？
3. **我凭什么相信 2048？** 万一真实值是 2100 呢？万一这一秒读 2048、下一秒读 2069 呢？

这三个问题刚好对应 ADC 的三件事：**分辨率（resolution）**、**参考电压（reference voltage）**、**精度（accuracy）**。前两个回答"一个数代表多少伏"，第三个回答"这个数能不能信"——而"能不能信"才是这一章真正想搞明白的事。

第 2 节就把这三件事一个个拆开算。

## 2. 原理：它到底是怎么工作的

### 2.1 逐次逼近型（SAR）ADC 是怎么"猜"出来的

F103 里的 ADC 是**逐次逼近型（Successive Approximation Register, SAR）**。它内部没有一个能"直接读出电压值"的元件，它的做法很笨也很聪明：**自己造一个电压，和输入比大小，一点一点逼近。**

```text
                       12 位逐次逼近寄存器 SAR
              ┌──────────────────────────────────────────┐
              │  b11 b10 b9 b8 b7 b6 b5 b4 b3 b2 b1 b0   │
              └──────┬───────────────────────────▲───────┘
                     │ 试探码（当前的猜测值）      │ "大了" / "小了"
                     ▼                           │
              ┌─────────────┐                    │
              │   内部 DAC    │                    │
              └──────┬──────┘                    │
                     │ V_DAC                     │
                     ▼                           │
   V_IN ──────▶ ┌───────────┐                   │
   （采样保持电容 │  模拟比较器 │───────────────────┘
     上存住的电压）└───────────┘
                     ▲
                     │
              ┌──────┴──────┐
              │ C_ADC 约 8pF │   ← 采样阶段由输入通过 R_AIN、R_ADC 给它充电
              └─────────────┘
```

整个转换过程就是**二分查找**：先把最高位猜成 1，造出半个量程的电压跟输入比——太大了就把这一位清掉，不够大就留着；然后试下一位。

**12 位就是 12 次比较**，12 次之后剩下的那个码值就是结果。这个"一位一位往下压"的过程叫逐次逼近，ADC 也就跟着叫逐次逼近型。

光看图还是虚的，我拿一个具体电压走一遍。设输入 **V_IN = 0.800V**，参考电压 **VREF = 3.3V**：

```text
LSB = 3.3V / 4096 = 0.805664 mV ≈ 0.806 mV
```

| 第几步 | 试哪一位 | 试探码 | 内部 DAC 造出的 V_DAC | 和 0.800V 比 | 结果 |
|---|---|---|---|---|---|
| 1 | b11（最高位） | 2048 | 1650.00 mV | 大 | b11 清 0 |
| 2 | b10 | 1024 | 825.00 mV | 大 | b10 清 0 |
| 3 | b9 | 512 | 412.50 mV | 小 | b9 留 1 |
| 4 | b8 | 512+256 = 768 | 618.75 mV | 小 | b8 留 1 |
| 5 | b7 | 768+128 = 896 | 721.88 mV | 小 | b7 留 1 |
| 6 | b6 | 896+64 = 960 | 773.44 mV | 小 | b6 留 1 |
| 7 | b5 | 960+32 = 992 | 799.22 mV | 小（差 0.78 mV） | b5 留 1 |
| 8 | b4 | 992+16 = 1008 | 812.11 mV | 大 | b4 清 0 |
| 9 | b3 | 992+8 = 1000 | 805.66 mV | 大 | b3 清 0 |
| 10 | b2 | 992+4 = 996 | 802.44 mV | 大 | b2 清 0 |
| 11 | b1 | 992+2 = 994 | 800.83 mV | 大 | b1 清 0 |
| 12 | b0（最低位） | 992+1 = 993 | 800.02 mV | 大（只大 0.02 mV） | b0 清 0 |

12 步走完，寄存器里留下的是 **992**。回算一下：

```text
992 × 0.805664 mV = 799.22 mV = 0.79922 V
真实值           = 800.00 mV = 0.80000 V
误差             = 0.78 mV   （小于 1 个 LSB = 0.806 mV）
```

有两处值得停下来说：

- **最后一步特别微妙。** 试探码 993 造出来的电压是 800.02 mV，只比输入 800.00 mV 高 **0.02 mV**，也就是 1 个 LSB 的 1/40。比较器照样得判出"大了"。这就是为什么 ADC 的模拟部分这么讲究——比较器的分辨能力必须远好于 1 个 LSB，否则低几位就是随机的。
- **SAR 的结果是"向下取整"，不是"四舍五入"。** 992 对应的是 799.22 mV，而不是最近的 800.00 mV（993）。这不是 bug：SAR 只会保留那些"造出来的电压 ≤ 输入"的码值，所以单极性 ADC 的量化误差是 **0 ~ −1 LSB**，永远偏低，不会偏高。这一点第 2.3 节会再算一次账。

**转换要花多久？** 手册给的公式是

```text
T_conv = T_s + 12.5 个 ADC 周期
         ↑      ↑
      采样时间   12 次逼近（12 个周期）+ 0.5 个周期把结果写进数据寄存器
```

注意 **12.5 不是一个凑出来的数**：12 次比较确实占 12 个周期，多出来的 0.5 个周期是最后一次比较落地、结果搬进 DR 寄存器的开销。所以"采样时间"和"转换时间"是两回事，第 2.4 节算的就是 T_s 该取多少。

### 2.2 ⭐ 量化与分辨率：LSB 到底该除 4096 还是 4095

**位数决定刻度数。** 12 位 ADC 的输出是 12 个二进制位，能表示的码值是 0 ~ 2¹²−1：

```text
2^12 = 4096        → 一共 4096 个码值：0, 1, 2, …, 4095
```

**LSB（Least Significant Bit，最低有效位）** 就是相邻两个码值之间代表的电压差，也就是"一把尺子上最小的那格有多宽"：

```text
LSB = VREF / 2^n
    = 3.3 V / 2^12
    = 3.3 V / 4096
    = 3300 mV / 4096
    = 0.805664 mV
    ≈ 0.806 mV ≈ 0.8 mV
```

所以"2048 代表多少伏"的完整算法是：

```text
V = raw × LSB
  = 2048 × 0.805664 mV
  = 1650.00 mV
  = 1.650 V
```

**那为什么有人除 4095？** 因为这行代码很常见：

```c
voltage = raw * 3.3f / 4095.0f;      /* 也见过这么写的 */
```

两种说法各自的道理是这样的：

| 算法 | 除 4096（`VREF / 2^n`） | 除 4095（`VREF / (2^n − 1)`） |
|---|---|---|
| 物理含义 | 把 0 ~ VREF 这段输入**切成 4096 个等宽的台阶**，算的是**一级台阶的高度** | 把**两个端点码值** 0 和 4095 分别钉在 0V 和 VREF 上，算的是**端点到端点的平均斜率** |
| 对应 ADC 的真实行为 | ✅ 对得上。码值 k 代表的是输入落在 `[k·LSB, (k+1)·LSB)` 这个区间里 | ❌ 对不上。码值 4095 对应的输入区间是 `[4095·LSB, 4096·LSB)`，不是恰好等于 VREF |
| 算出来的 LSB | 3.3 / 4096 = **0.805664 mV** | 3.3 / 4095 = **0.805861 mV** |
| 两者差多少 | 差 0.000197 mV | 相对差 = 1/4096 ≈ **0.0244%**（正好是最后一位的权重） |
| 什么时候该用 | 算分辨率、算量化误差、算噪声折算成多少 mV、做精度分析 | 已经标定过、并且要求"输入顶到满量程时输出必须显示成 VREF"的场合 |

**我自己的结论**：算物理量（分辨率、量化误差、噪声）**一律除 4096**，因为 ADC 真的就是 4096 个台阶。除 4095 不是"错得更离谱"，它只是把"码值"当"端点"用了，代价是最多偏 1 个 LSB —— 在 12 位上就是 0.8mV，很多时候无所谓，但要是拿它去估算噪声，误差的概念就串了。

顺带说个更实际的做法：与其争论分母，不如**做两点标定**。拿一个准的万用表，给两个已知电压（比如 0.500V 和 2.500V），读两次 raw，用两点求直线：

```text
LSB_实测 = (2.500 V − 0.500 V) / (raw2 − raw1)
V_offset = 0.500 V − raw1 × LSB_实测
```

这样连 VREF 实际是 3.28V 还是 3.30V、有没有零点偏移，全都一起标进去了。第 2.5 节会讲为什么这个偏移是真实存在的。

### 2.3 量化误差：为什么是 ±0.5 LSB

前面 SAR 的过程说明了一件事：**输入电压是连续的，输出码值是离散的**。真实电压落在两个码值中间时，ADC 只能二选一，这中间的差就是**量化误差（quantization error）**。

如果按"离得最近的那个码值"来接（四舍五入），最大的偏差就是半个台阶：

```text
最大量化误差 = ±0.5 LSB
             = ±0.5 × 0.805664 mV
             = ±0.402832 mV
             ≈ ±0.4 mV
```

换成相对值更好记：

```text
±0.5 LSB / 满量程 = ±0.5 / 4096 = ±1/8192 ≈ ±0.0122%
```

**但 F103 的 SAR 不是四舍五入，是向下取整**（第 2.1 节算过 0.800V 读回 992 而不是 993）。所以单极性输入下实际误差落在 **0 ~ −1 LSB** 这个区间里，永远是"偏低"。

这个差别在做控制的时候会露出马脚：如果我用 ADC 读一个恒定的 1.650V 基准，理论上读数会一直偏大约 0.5 LSB 的对应码值。想消掉它有个标准做法——**加半格**：

```c
/* 把向下取整的结果补回半格，就变成"四舍五入到最近码值" */
voltage = ((float)raw + 0.5f) * LSB_MV;
```

多说一句：**±0.4mV 只是"量化"这一个环节引入的误差**。真实 ADC 的总误差还要叠上偏移误差、增益误差、积分非线性（INL）、微分非线性（DNL）、参考电压本身的误差。数据手册的电气特性表里这些参数是分开列的（见第 3 节），量化误差只是其中**不可消除的那一项**：它来自"用有限个台阶表示连续量"这件事本身，跟芯片做得多好无关。

### 2.4 ⭐⭐ 采样时间与外部输入阻抗：本章最实用的一条

这一节是我看这一章时收获最大的一段，因为它解释了一个"代码一个字没写错，读数就是不对"的现象。

**问题的根源：ADC 不是瞬间读到电压的。**

逐次逼近需要一段时间，而在这段时间里输入电压必须**保持不变**——否则一边比一边变，比出来的 12 位就没意义了。为了让它在转换期间不变，ADC 在转换之前先做一步**采样（sampling）**：把一个内部小电容 **C_ADC** 接到输入引脚上，让电容上的电压**追上**输入电压，然后把开关断开（这一步叫**保持 hold**），转换期间比较器比的就是这个电容上的电压。

看内部等效电路。转换开始前，采样开关闭合，外部信号通过源阻抗 R_AIN、再经过内部采样开关的导通电阻 R_ADC，给 C_ADC 充电：

```text
                   采样开关（采样阶段闭合，转换阶段断开）
                        │  R_ADC（F103 典型 1kΩ）
   V_IN ──[ R_AIN ]─────┼──────┬───────────▶ 到比较器
   外部源阻抗            │      │
   （分压电阻并联值）      │     ─┴─ C_ADC（F103 典型 8pF）
                        │      │
                       GND    GND
```

这是一个最普通的 **RC 充电回路**，电容电压按指数规律上升：

```text
v(t) = V_IN × (1 − e^(−t / RC))，其中 R = R_AIN + R_ADC，C = C_ADC
```

充电永远充不到 100%，所以只能问："充到什么程度算够？"答案是**误差小于 ½ LSB**——比量化误差本身还小，那这点充电误差就淹没了，不影响结果。

½ LSB 相当于满量程的多少？12 位下：

```text
½ LSB / 满量程 = 0.5 / 4096 = 1 / 8192 = 1 / 2^13
```

把"剩余误差小于 1/2^13"代进指数式：

```text
V_IN × e^(−t_s / RC)  <  V_IN × (1 / 2^13)

两边约掉 V_IN（这一步很关键：它说明
"要充多准"只跟位数有关，跟输入电压多大无关）：

e^(−t_s / RC) < 2^(−13)

两边取对数：

t_s / RC > 13 × ln2 = 13 × 0.693147 = 9.0109 ≈ 9.02

所以：

┌──────────────────────────────────────────────────────┐
│  t_s ≥ 9.02 × (R_AIN + R_ADC) × C_ADC                │
└──────────────────────────────────────────────────────┘
```

**这个 9.02 的来历必须记住**：它就是 `(n+1) × ln2`，12 位时是 `13 × ln2`。位数越高，系数越大（16 位是 `17 × ln2 = 11.78`）——分辨率越高，对采样时间的要求越狠。

（顺带记一笔：不同资料里这个系数能看到 8.3、9.0、9.7 三个值，差别全在"充到多准才算够"这条标准上——按 ½ LSB 算是 13·ln2 = 9.01，按更保守的 ¼ LSB 算是 14·ln2 = 9.70。我这里统一按 ½ LSB，也就是 9.02。）

#### 算例 1：源阻抗 10kΩ

设从 ADC 引脚往外看，等效源阻抗 **R_AIN = 10kΩ**，查数据手册 F103 的 **R_ADC = 1kΩ、C_ADC = 8pF**：

```text
R_AIN + R_ADC = 10 kΩ + 1 kΩ = 11 kΩ = 11000 Ω

t_s ≥ 9.02 × 11000 Ω × 8 pF
    = 9.02 × 11000 × 8×10⁻¹²
    = 9.02 × 8.8×10⁻⁸ s
    = 7.9376×10⁻⁷ s
    = 793.76 ns
    ≈ 794 ns
```

**794ns 要配多长的采样时间？** STM32 的采样时间不是一个自由填的数字，只能从 8 个档位里选（SMPR 寄存器）：

```text
可选的采样周期数：1.5 / 7.5 / 13.5 / 28.5 / 41.5 / 55.5 / 71.5 / 239.5
```

先看最坏情况，ADC 时钟 14MHz（F103 的上限）：

```text
1 个 ADC 周期 = 1 / 14 MHz = 71.43 ns

需要的周期数 = 793.76 ns / 71.43 ns = 11.11 个周期
```

11.11 个周期落在 7.5 和 13.5 之间 —— **7.5 个周期只有 535.7ns，不够**；所以必须选 **13.5 或更长**：

```text
13.5 周期 × 71.43 ns = 964.3 ns ≥ 794 ns ✅
 7.5 周期 × 71.43 ns = 535.7 ns <  794 ns ❌
```

换成我实际用的配置（ADC 时钟 12MHz，PCLK2/6）结果一样：

```text
1 个 ADC 周期 = 1 / 12 MHz = 83.33 ns
需要的周期数 = 793.76 ns / 83.33 ns = 9.53 个周期  →  同样选 13.5 周期（1125.0 ns）✅
```

#### 算例 2：电位器其实比想象的"好推"

10kΩ 电位器接成两端 3V3、GND，中间脚接 ADC 的用法，**从中间脚往外看的等效源阻抗不是 10kΩ**。电位器相当于两个电阻串联分压，滑片在中间时是两个 5kΩ 并联：

```text
R_AIN(最大) = (R/2 ∥ R/2) = R/4 = 10 kΩ / 4 = 2.5 kΩ   ← 滑片在正中间时最大

t_s ≥ 9.02 × (2500 Ω + 1000 Ω) × 8 pF
    = 9.02 × 3500 × 8×10⁻¹²
    = 2.5256×10⁻⁷ s
    = 252.6 ns
```

252.6ns 在 12MHz 下只要 3.03 个周期，**连 7.5 周期的档都绰绰有余**。实测（🔬 待验证）电位器这一路读数应该挺稳，原因就在这。

#### 算例 3：为什么"用 100kΩ 分压直接接 ADC"会读数偏低

这是我在第 6 节要写的坑，先把账算清楚。假设我用**两个 100kΩ 电阻串联**从 3.3V 分压出 1.65V，从中间点看进去：

```text
R_AIN = 100 kΩ ∥ 100 kΩ = 50 kΩ

t_s ≥ 9.02 × (50000 Ω + 1000 Ω) × 8 pF
    = 9.02 × 51000 × 8×10⁻¹²
    = 9.02 × 4.08×10⁻⁷ s
    = 3.680×10⁻⁶ s
    = 3.68 µs
```

3.68µs 在 12MHz 下是 **44.2 个周期**，只能选 55.5 或更长。**但如果采样时间留在默认的 1.5 个周期呢？**

```text
1.5 周期 = 1.5 × 83.33 ns = 125.0 ns

t_s / RC = 125.0 ns / (51000 Ω × 8 pF)
         = 125.0×10⁻⁹ / 4.08×10⁻⁷
         = 0.3064

电容充到的比例 = 1 − e^(−0.3064) = 1 − 0.7361 = 26.4%

也就是说：输入 1.650 V，采样结束时电容上只有
1.650 V × 26.4% = 0.436 V

ADC 会读回 0.436 V / 0.805664 mV ≈ 541
而正确值应该是 1650.00 mV / 0.805664 mV ≈ 2048

读数偏低 (2048 − 541) / 2048 = 73.6%
```

**偏低 73.6%，不是"不准"，是根本不能用。** 我一开始以为源阻抗大一点顶多让读数偏个百分之几——把公式代进去才知道这个曲线是指数式的，不是线性的。顺手把几个源阻抗都算了一遍（采样时间都取 1.5 周期 = 125ns，12MHz）：

| 源阻抗 R_AIN | R = R_AIN+1kΩ | RC 时间常数 | t_s/RC | 充到的比例 | 读数偏低 |
|---|---|---|---|---|---|
| 1 kΩ | 2 kΩ | 16.0 ns | 7.81 | 99.96% | 0.04% |
| 2.5 kΩ（10k 电位器） | 3.5 kΩ | 28.0 ns | 4.46 | 98.85% | 1.2% |
| 8.7 kΩ | 9.7 kΩ | 77.7 ns | 1.61 | 80.0% | **20.0%** |
| 10 kΩ | 11 kΩ | 88.0 ns | 1.42 | 75.8% | 24.2% |
| 50 kΩ（两个 100k 分压） | 51 kΩ | 408 ns | 0.31 | 26.4% | **73.6%** |
| 100 kΩ | 101 kΩ | 808 ns | 0.155 | 14.3% | 85.7% |

表里那一行 **8.7kΩ → 偏低 20%** 是我特意留的：它是我印象里"好像偏得不多"的那种情况，而这个"感觉上还好"的 20%，对应的源阻抗只有 8.7kΩ。**再往上每加一点阻抗，误差就往 100% 冲。**

顺带一个反直觉但很重要的推论：**分压测量本身就把源阻抗抬高了。** 想测高电压就得用大电阻分压（否则分压电阻自己就在耗电、发热），而大电阻恰好是 ADC 最不喜欢的输入。这两个需求是打架的，所以第 2.9 节讲电池电压监测时，要在"分压电阻取多大"和"采样时间设多长"之间做权衡——而采样时间越长，转换越慢，又会影响控制环。工程上的取舍就是这么一环扣一环。

### 2.5 参考电压 VREF+：3.3V 不是从天上掉下来的

回头看第 1 节的问题 2：**分母上那个 3.3V 是哪来的？**

ADC 的换算比例是相对于**参考电压 VREF+** 的，不是相对于"电源电压"这个概念。F103 的参考电压是 **VREF+ 引脚**上的电压。但这里有个我查手册才知道的细节：

**我这块 Blue Pill 是 LQFP48 封装，48 个脚里根本没有单独的 VREF+ 引脚——它在芯片内部直接接到了 VDDA。** 也就是说，小封装上 VREF+ = VDDA = 3.3V（板子上那颗 AMS1117 稳压器给的）。

这个事实有两个直接后果：

**后果一：输入绝对不能超过 3.3V。** 引脚上的电压只要超过 VDDA + 0.3V（手册的绝对最大额定值），内部那两个保护二极管就会正偏导通，电流从引脚灌进 VDDA 网络，轻则把 AD 结果拉满、污染整个 3.3V 轨，重则直接把引脚或 ESD 结构烧掉。**ADC 引脚是"只能测 0 ~ 3.3V"的**——这一点在第 6 节坑 3 会认真算一遍。

**后果二：3.3V 有多准，读数就有多准。** 我用 3.3V 当分母算电压，可板上那颗稳压器出来的是 3.3V ± 2% 量级的东西，还随负载、温度、USB 供电质量飘。假设实际 VDDA = 3.28V，而我按 3.30V 算：

```text
相对偏差 = (3.30 V − 3.28 V) / 3.30 V = 0.606%

读一个真实 2.000 V 的输入：
  显示值 = 2.000 V × (3.30 / 3.28) = 2.012 V
  多显示 12 mV —— 而且整个量程里所有读数都按这个比例偏
```

**这是"增益误差"，不是"随机误差"**，它不会因为多平均几次就消失，得靠标定或者换更准的基准来治。

**怎么提高精度？** 有两条路：

| 做法 | 怎么做 | 代价 |
|---|---|---|
| 用外部基准（推荐在正式板子上用） | 选一个独立的基准芯片（比如 REF3033，3.300V，初始精度 ±0.05%，温漂典型 30ppm/℃），输出接到 VREF+ 引脚 | 只有 100 脚以上的封装才引出了 VREF+；多一颗芯片、多几毛到几块钱成本 |
| 用内部参考电压 VREFINT 反推 VDDA（**小封装的免费方案**） | F103 内部有一个不受电源波动影响的参考源 VREFINT（典型 1.200V），它接在 ADC 的一个内部通道上（不占外部引脚）。先读它，再反推 VDDA | 精度受 VREFINT 本身 ± 个体差异限制（出厂值有离散），只能把"随电源飘"这一项压下去 |

VREFINT 反推的算式，我把数代进去算了一遍：

```text
① 先读 VREFINT 通道的原始码值 raw_ref
② VDDA = 1200 mV × 4096 / raw_ref
③ 之后所有换算都用这个实测的 VDDA 当 VREF，不再用 3.3V

举例（假设 VREFINT 恰好是 1200mV 且没有个体偏差）：
  VDDA = 3.300 V 时：raw_ref = 1200 / 3300 × 4096 = 1489.5  →  读到 1489
  VDDA = 3.280 V 时：raw_ref = 1200 / 3280 × 4096 = 1498.5  →  读到 1498
  反推：VDDA = 1200 mV × 4096 / 1489 = 3301.0 mV   （对上 3.300V ✅，差 1mV 来自 raw 只能取整）
        VDDA = 1200 mV × 4096 / 1498 = 3281.2 mV   （对上 3.280V ✅）
```

VDDA 只差 20mV（3.300V → 3.280V），落到 raw_ref 上只有 9 个码值的变化（1489 → 1498）——分辨率够用，但这也说明**别指望它把精度提到 1mV 以内**，它的价值在于"跟着电源一起飘"的那部分误差被消掉了。

⚠️ 用 VREFINT 和内部温度传感器还有两个坑：**两者都必须在 ADC 的公共寄存器里把 TSVREFE 位置 1 才能工作**（CubeMX 里勾上对应通道时会自动置位，手写寄存器的话很容易漏），而且**它们的采样时间都要给足**——温度传感器手册要求不少于 **17.1µs**，在 12MHz 的 ADC 时钟下只有 239.5 周期（19.96µs）这一个档够用。

### 2.6 规则组（regular group）与注入组（injected group）

F103 的一个 ADC 有**两组通道**，它们不是"两组一样的"，分工不同：

| | 规则组 regular group | 注入组 injected group |
|---|---|---|
| 最多几个通道 | 16 个 | **4 个** |
| 转换结果存哪 | **全部写进同一个 DR 寄存器**，后一个覆盖前一个 | **4 个独立寄存器 JDR1~JDR4**，互不覆盖 |
| 多通道怎么取数据 | 必须靠 DMA 搬走，或者一个通道一个通道地及时读 | 转完直接读 JDRx 就行，不用 DMA |
| 能不能被抢占 | 会被注入组打断 | **优先级更高，可以打断正在进行的规则组转换**，转完再回到规则组继续 |
| 谁来触发 | 软件、外部触发、定时器 | 软件、外部触发、定时器（注入组常配定时器触发） |
| 典型用途 | 电池电压、温度、慢速监测 | 电流采样、和 PWM 严格同步的采样 |

**"注入"这个名字就是抢占的意思**：注入事件像插队，把规则组的序列暂停，转完自己那 4 个通道再让规则组接着走。

**为什么 RM 上电流采样用注入组？** 我想了一下，理由是三条叠在一起：

1. **必须和 PWM 同步。** 电机的相电流在 PWM 周期里是变化的（尤其在开关瞬间有振铃），如果采样点和 PWM 不对齐，每次采到的相位都不一样，采出来的值就没意义。做法是让 **TIM1 在 PWM 中心对齐模式下的下溢事件触发 ADC 注入组**，采样点落在纹波最小的位置。
2. **时间上不能等。** 电流环是整台车响应最快的环，采样被别的慢速转换（比如电池电压）挡住几十微秒是不能接受的。注入组的抢占正好解决这个——规则组可以在后台慢慢扫电压，电流一来立刻插进去。
3. **4 个独立结果寄存器刚好够用。** 三相电流 Ia/Ib/Ic 再加一个母线电压或者采样电阻温度，正好 4 个，而且不用 DMA 也不会丢数据。

我这一章只在规则组里转（电位器 + 温度），因为要先把"一个数代表多少伏"这件事弄清楚；注入组这条路我记在这里，等做到电流环的时候按这条线走。

### 2.7 ⭐ 软件滤波：三种常用做法，各自付多少延迟

先看为什么必须滤波。12 位 ADC 的 1 LSB 只有 **0.806mV**，而板子上的噪声来源一大堆：

```text
电源纹波（AMS1117 的输出、USB 供电）
数字开关噪声（GPIO 翻转、SPI/I2C 时序）
电机换向的尖峰（同一个电源网络上的电机一换向就是一个脉冲）
面包板 + 杜邦线的接触电阻（我这里就是这种情况）
```

这些加起来让读到的原始值在 **±10 ~ ±30 个 LSB** 里跳是常事（🔬 待上机确认具体数字），也就是 ±8 ~ ±24mV。第 1 节的问题 3"我凭什么相信 2048"的答案就在这里：**原始值本身就是抖的，必须先想办法把它稳住。**

三种做法，我一个个说清楚各自对付什么、代价是什么。

#### （1）滑动平均（moving average）：压随机噪声，最直观

对最近 N 个采样值求算术平均：

```text
y[n] = ( x[n] + x[n-1] + ... + x[n-N+1] ) / N
```

**传递函数**（Z 域，N 个点的平均就是一个长度为 N 的矩形窗）：

```text
H(z) = (1/N) × (1 − z⁻ᴺ) / (1 − z⁻¹)
```

它的幅频特性有几个必须知道的点：

- **直流增益恰好是 1**（不会改变真实值的大小）；
- **第一个零点在 f = fs / N**。我用 fs = 200Hz（5ms 一次）、N = 8，零点就在 **25Hz**，之后每隔 25Hz 一个零点。**50Hz 工频干扰正好落在第二个零点上，会被完全滤掉**——这是个意外的好处；
- **前提是采样间隔严格均匀。** 我的代码里用的是 `HAL_Delay(5)`，周期有抖动（第 5 节会讲），采样一抖，零点的位置就跑掉了，50Hz 这个好处就保不住。要吃到这个好处，得改用定时器严格触发。

**噪声能压多少？** 对白噪声（各次采样互不相关）：

```text
N 个独立随机量求平均，标准差变成原来的 1/√N
N = 8：噪声降 √8 = 2.83 倍（相当于信噪比改善 20·log10(2.83) = 9.0 dB）
```

**代价是延迟**，这一点最容易被忽略：

```text
群延迟 = (N − 1) / 2 个采样周期
N = 8，dt = 5ms：(8 − 1) / 2 = 3.5 个采样周期 = 3.5 × 5ms = 17.5 ms
```

**17.5ms 的延迟对"看电池电压"毫无影响**（电池电压几秒钟才变一点），**但对电流环是致命的**（电流环要在几十微秒里做出反应）。所以滑动平均不能无脑往哪儿都加。

工程实现上别每次重新累加 N 个数，用一个 `sum` 变量做"进一个、出一个"：

```c
/* 环形缓冲 + 一个累加和，每次 O(1) */
sum -= buf[idx];      /* 减掉最老的值 */
buf[idx] = x;         /* 覆盖掉它 */
sum += x;             /* 加上新值 */
idx = (idx + 1) % N;
y = sum / N;
```

#### （2）一阶低通（IIR）：算力最省，延迟可控

```text
y[n] = y[n-1] + α × ( x[n] − y[n-1] )
```

一行代码、一次乘法、一次加法，只要保存**一个**状态变量 `y`，不用像滑动平均那样留 N 个数的缓冲。

**传递函数**：

```text
H(z) = α / (1 − (1 − α) z⁻¹)

极点位置 = 1 − α。只要 α ∈ (0, 1]，极点就始终在单位圆内 → 永远稳定。
（这一点比二阶滤波器省心得多：二阶要担心 Q 值、担心发散，一阶不用。）
直流增益 = α / (1 − (1 − α)) = 1  ✅
```

**α 怎么取？** 两个参数就够：采样周期 dt 和想要的时间常数 τ：

```text
α = dt / (τ + dt)

推导：连续形式是一阶微分方程  τ·(dy/dt) + y = x
     用后向差分 dy/dt ≈ (y[n] − y[n−1]) / dt 代进去：
       τ·(y[n] − y[n−1]) / dt + y[n] = x[n]
     整理：
       y[n]·(τ/dt + 1) = x[n] + (τ/dt)·y[n−1]
       y[n] = y[n−1] + [dt/(τ+dt)]·(x[n] − y[n−1])
     于是 α = dt / (τ+dt)。
     （这是近似式；精确式是 α = 1 − e^(−dt/τ)，两者在 τ ≫ dt 时基本重合。
      我这章取 τ = 20ms、dt = 5ms，τ/dt = 4，两种算法分别是 0.2 和 0.221，
      差 10%。要想更准就用精确式。）
```

**截止频率**（α 比较小时）：

```text
f_c ≈ α / (2π·dt) ≈ 1 / (2π·τ)

τ = 20 ms → f_c ≈ 1 / (2π × 0.02 s) = 1 / 0.12566 s = 7.96 Hz
```

也就是 7.96Hz 以上的噪声按 **−20dB/十倍频** 往下压。注意这个"20ms"同时就是它的**延迟**——一阶惯性环节的等效群延迟就是 τ 本身。

**它比滑动平均好在哪？** 延迟是**连续可调**的：想要 1ms 的延迟就把 τ 设成 1ms，不需要被"N 必须是整数"卡住。而且延迟和抑噪的权衡是一条平滑曲线，不像滑动平均只有 N=2、4、8、16 这几个台阶。

#### （3）中值滤波（median filter）：专治脉冲噪声

取最近 N（N 取奇数）个采样值，**排序后取中间那个**：

```text
y[n] = median( x[n], x[n-1], ..., x[n-N+1] )

伪代码（N = 5）：
    buf[idx] = x;  idx = (idx + 1) % 5;
    复制 buf 到 tmp;            ← 必须复制，不能原地排序（环形缓冲的下标顺序 ≠ 时间顺序）
    对 tmp 做插入排序;           ← N=5 时最坏 10 次比较，比通用排序快得多
    返回 tmp[2];                ← 正中间那个
```

**它和前面两种有本质区别：它是非线性的。** 中值不是输入的线性组合，所以**写不出 H(z) 这样的传递函数**，频域那套分析对它不适用（只能从统计特性和延迟上描述）。这一点值得记牢，不然容易在推导里犯错。

**它对付的是脉冲噪声。** 假设一串平稳数据里插进一个尖峰：

```text
输入： 100, 100, 100, 4000, 100        ← 4000 是电机换向打出来的单点尖峰
中值输出：        100                  ← 尖峰被整个丢掉，前后全是 100
滑动平均(N=5)输出：  900                ← 尖峰还在里面，被摊薄成 900
```

**滑动平均对脉冲噪声是无能为力的**：它只能把尖峰"摊薄"，不能消除。一个 4000 的尖峰平均进 5 个数里还剩 900，照样把控制环带偏。**这是选中值滤波的唯一理由，也是它不可替代的地方。**

中值能扛住多宽的尖峰？**窗口 N 能干掉最多 (N−1)/2 个连续异常点**：

| N | 能干掉连续几个异常点 | 延迟 ≈ (N−1)/2 个采样周期 |
|---|---|---|
| 3 | 1 个 | 1 个采样周期 |
| 5 | 2 个 | 2 个采样周期 |
| 7 | 3 个 | 3 个采样周期 |

窗口开太大有个副作用要留意：**真实信号突变时它会把突变"推迟"N/2 拍才反映出来**——中值滤波分辨不出"尖峰"和"真实跳变"，它只会认为"少数服从多数"。所以电位器从一头拧到另一头，输出会慢半拍跟上。

#### 三种滤波的横向对比

| | 滑动平均 | 一阶低通 | 中值滤波 |
|---|---|---|---|
| 数学形式 | 线性（FIR） | 线性（IIR） | **非线性** |
| 传递函数 | `(1/N)(1−z⁻ᴺ)/(1−z⁻¹)` | `α/(1−(1−α)z⁻¹)` | **写不出来**，只能给伪代码 |
| 对付随机噪声 | ✅ 降 √N 倍（N=8 → 2.83 倍） | ✅ 高频 −20dB/十倍频 | ❌ 对随机噪声作用有限 |
| 对付脉冲噪声/单点尖峰 | ❌ 只能摊薄 | ❌ 只能摊薄 | ✅ **专治这个** |
| 需要的 RAM | N 个数 + 一个累加和 | **一个状态变量** | N 个数 + 一个 N 元素临时数组 |
| 计算量（每次） | 2 次加减 + 1 次除 | **1 次乘加** | N 次比较量级的排序 |
| 延迟 | (N−1)/2 个采样周期 | ≈ τ（连续可调） | ≈ (N−1)/2 个采样周期 |
| 我的结论 | 慢速监测可以随便用 | 通用、首选 | **有电机/继电器的地方必须加** |

#### 组合起来用 + 延迟预算

我这章给电位器这一路串的是 **中值 → 滑动平均 → 一阶低通**。顺序不能随便排：

```text
先中值       ：把脉冲尖峰整个抠掉
再滑动平均   ：压掉剩下的随机噪声
最后一阶低通 ：给后面的控制环一个平滑量

如果顺序反过来（先平均后中值）：
  一个 4000 的尖峰先被 5 点平均摊成 900，
  中值滤波面对的窗口就变成 100,100,900,100,100 —— 900 已经不是"少数"了，
  中值会把它当成正常值留下来。顺序错了，中值滤波就白加了。
```

代价是延迟要**加起来**（dt = 5ms）：

```text
中值 N=5        ：(5−1)/2 = 2 个周期  = 10.0 ms
滑动平均 N=8    ：(8−1)/2 = 3.5 个周期 = 17.5 ms
一阶低通 τ=20ms ：                     20.0 ms
————————————————————————————————————————————
总延迟（输入变化 → 输出反映出来）≈ 47.5 ms
```

**47.5ms 是什么概念？** 拧一下电位器，串口打出来的滤波值要将近 50ms 之后才跟到位；对"看电池电压"这件事，50ms 完全无所谓。**但如果这是电机电流，47.5ms 意味着电流环的带宽连 10Hz 都够呛**——完全不可接受。

所以对**电流采样**我的结论是（这一步是设计阶段的推算，不是实测）：

```text
只用：中值 N=3 （1 个采样周期的延迟）+ 一阶低通 τ = 1 个采样周期
总计 ≈ 2 个采样周期

按电流环 20kHz（dt = 50µs）算：2 × 50µs = 100µs 的总延迟。
电流环自身的周期才 50µs，两拍延迟已经是"能用"的边界了；
再加一级 N=4 的滑动平均（1.5 拍）就到 3.5 拍，闭环带宽会被明显压下去。
```

所以第 6 节我会写：**"中值 + 低通"是电流采样的合理组合，"中值 + 滑动平均 + 低通"是慢速监测的组合，两者不能混用。** 滤波不是"加得越多越稳"，每加一级都在往系统里塞延迟，而延迟在闭环里是直接吃稳定裕度的。

（还有一个我现在就记下的问题：F103 没有硬件浮点单元（FPU），浮点乘加是靠软件库算的，一次几十个周期。200Hz 的采样率下完全无感，但真要做 20kHz 的电流环，滤波得改成定点（Q15）或者整数移位来做。这件事等我真去做电流环的时候再动。）

### 2.8 ADC 时钟：不能超过 14MHz

F103 的 ADC 时钟不是直接等于系统时钟，它由 **PCLK2（APB2）** 再分频得到，而且手册给的**上限是 14MHz**：

```text
ADC 时钟 = PCLK2 / 分频系数，分频系数可选 2 / 4 / 6 / 8

我的配置：PCLK2 = 72MHz，分频 6
        → ADC 时钟 = 72 MHz / 6 = 12 MHz ≤ 14 MHz ✅

（如果分频 2：72/2 = 36MHz > 14MHz ❌ —— 这是 CubeMX 里很容易手滑的地方，
  ADC 会转得莫名其妙，或者在温度高的时候才出错，特别难查。）
```

**为什么要有这个上限？** 因为逐次逼近的 12 次比较里，每一次都要经过"内部 DAC 建立 → 比较器判决 → 结果写回寄存器"这一串模拟过程。时钟给太快，比较器还没判完就被要求进入下一步，结果就错了。**这不是数字逻辑的时序问题，是模拟电路的建立时间问题。**

采样时间只有 8 个固定档位。我把 12MHz 下每一档的**采样时间**和**总转换时间**（T_conv = T_s + 12.5 周期）都算出来了：

```text
1 个 ADC 周期 = 1 / 12 MHz = 83.333 ns
```

| SMPR 档位（采样周期数） | 采样时间 T_s = 档位 × 83.33ns | 总转换时间 T_conv = T_s + 12.5 周期 | 适用场合 |
|---|---|---|---|
| 1.5 | 125.0 ns | 14 周期 = 1166.7 ns ≈ 1.17 µs | 源阻抗 < 1kΩ 的低阻抗信号，追求最快 |
| 7.5 | 625.0 ns | 20 周期 = 1666.7 ns ≈ 1.67 µs | 低阻抗信号（运放输出直连） |
| **13.5** | **1125.0 ns** | 26 周期 = 2166.7 ns ≈ 2.17 µs | **源阻抗 ~10kΩ，我这一章电位器用得上** |
| 28.5 | 2375.0 ns | 41 周期 = 3416.7 ns ≈ 3.42 µs | 源阻抗 ~30kΩ |
| 41.5 | 3458.3 ns | 54 周期 = 4500.0 ns = 4.50 µs | 源阻抗 ~50kΩ |
| 55.5 | 4625.0 ns | 68 周期 = 5666.7 ns ≈ 5.67 µs | **我在代码里给电位器用的档位** |
| 71.5 | 5958.3 ns | 84 周期 = 7000.0 ns = 7.00 µs | 高阻抗分压 |
| 239.5 | 19958.3 ns | 252 周期 = 21000.0 ns = 21.00 µs | **内部温度传感器 / VREFINT 只能用这档** |

在 14MHz（上限）下的对应数字，比上面快 1/6，比如 1.5 周期是 107.1ns、13.5 周期是 964.3ns、239.5 周期是 17107.1ns。

**这张表怎么用**：先用第 2.4 节的公式算出需要的 t_s，再从表里挑**第一个大于等于它的档位**。不用算得太精确，往上取一档的代价只是转换慢一点（多几百纳秒），算错了往下的代价是读数直接错掉。

我代码里给电位器配的是 **55.5 周期**，而不是算出来够用的 13.5 周期。原因不是"保险起见"这种模糊理由：**同一个规则组里还有内部温度传感器（必须 239.5 周期，19.96µs）**，扫描序列的总时间是各通道相加的：

```text
序列总时间 = (55.5 + 12.5) + (239.5 + 12.5)
           = 68 + 252
           = 320 个 ADC 周期
           = 320 × 83.333 ns
           = 26666.7 ns
           ≈ 26.67 µs
```

**26.67µs 已经是我这个 5ms 采样周期的 0.53%**，完全可以接受。但如果有人要把它塞进 20kHz 的电流环（周期 50µs），26.67µs 就占掉一半——那时候就必须把温度传感器踢出这个序列，或者干脆不采它。

### 2.9 RM 上用 ADC 到底在测什么

把原理落地，我在看 RoboMaster 的资料时留意到 ADC 在车上有三个典型用途，顺手把每个的账算了一遍：

**① 电池电压监测**

RM 的整车电源是 24V 标称（6 节锂电串联，满电 25.2V，截止 19.8V 左右）。这个电压要分压到 3.3V 以内才能接 ADC 引脚。

```text
分压比至少要：25.2 V / 3.3 V = 7.64 倍 —— 但"至少要"是不够的，
余量留少了，满电时直接顶到 3.3V 以上，就把引脚往危险区推了。
我按 8 倍以上设计：

取 R1 = 100 kΩ（上），R2 = 12 kΩ（下）：
  分压比 = (100 + 12) / 12 = 112 / 12 = 9.333 倍
  满电 25.2 V 时分压点电压 = 25.2 V / 9.333 = 2.700 V  ✅ 离 3.3V 还有 0.6V 余量
  截止 19.8 V 时          = 19.8 V / 9.333 = 2.121 V

从分压点看进去的源阻抗 R_AIN = 100 kΩ ∥ 12 kΩ = (100 × 12)/(100 + 12) = 10.71 kΩ
  t_s ≥ 9.02 × (10714 Ω + 1000 Ω) × 8 pF
      = 9.02 × 11714 × 8×10⁻¹²
      = 8.453×10⁻⁷ s = 845.3 ns
  12MHz 下：845.3 / 83.333 = 10.14 个周期 → 选 13.5 周期（1125 ns）✅

分压电阻自己耗的电（这条经常被忘）：
  I = 25.2 V / 112 kΩ = 0.225 mA
  一天 = 0.225 mA × 24 h = 5.4 mAh —— 对 5000mAh 的电池可以忽略，
  但如果做成"整车断电后仍接着电池"的常电监测，就得考虑这个数。
```

再补两点：分压点必须并一个 **100nF 到地**（这是给采样电容当"就近的电荷仓库"，能大幅减小源阻抗的影响），并且最好加一级钳位保护（比如 3.3V 稳压管或者对 VDDA 的肖特基），防止分压电阻虚焊、上电瞬间之类的意外把 24V 直接灌进引脚。

**② 电机电流采样**

典型是"采样电阻（shunt）+ 运算放大器 + ADC"：

```text
例：采样电阻 1 mΩ，流过 20 A → 压降 = 20 A × 1 mΩ = 20 mV
    这个电压太小，直接给 ADC 就等于 20mV / 0.806mV ≈ 25 个码值，
    分辨率差、还容易被噪声吃掉 → 用运放放大 50 倍 → 1.000 V
    1.000 V / 0.806mV ≈ 1241 个码值，分辨率就够了

功耗：P = I²R = 20² × 0.001 = 0.4 W —— 采样电阻必须选功率够的（比如 2512 封装的 1W 或 2W 型），
      这个发热是实打实的。
```

这一类信号**必须用注入组 + 定时器触发**（第 2.6 节），因为电流是快速变化量，采样点和 PWM 的相位关系直接决定读到的是不是真实电流。它也是第 2.7 节说的"绝不能加滑动平均"的那一路。

**③ 超级电容电压监测**

超级电容组的电压比电池还高（RM 上常见 30V 以上，规则允许上限每个赛季都在改），所以分压比要更大，而且它的电压跌落直接反映了功率输出状态，是功率控制环的输入之一。这一块**正好是海报上电路组写的"超级电容管理系统"**——分压网络怎么设计、采样点和整车电气怎么集成，是电路组的活。我先把它记下来：**ADC 不只是"读个电压"，它是功率控制环路里的一个传感器**，读得准不准直接决定后面 PID 算出来的功率对不对。

## 3. 手册依据

这一章的结论我都是从下面这两份原始文档里找的。**遇到对不上的地方，以手册为准，不要以我这篇笔记为准。**

| 结论 | 出处 |
|---|---|
| 12 位逐次逼近、T_conv = T_s + 12.5 周期 | **RM0008 参考手册**（STM32F101xx/102xx/103xx/105xx/107xx 参考手册）第 11 章 *Analog-to-digital converter (ADC)*，**11.3 节 ADC functional description**（里面有 ADC 的框图、采样与转换过程的描述） |
| 校准的流程和 CAL 位的用法 | RM0008 **第 11.4 节 Calibration** |
| 采样时间是"按通道"配置的（SMPR1/SMPR2） | RM0008 **第 11.6 节 Channel-by-channel programmable sample time** |
| 规则组 / 注入组的通道数、抢占关系、JDRx 寄存器 | RM0008 **第 11.3 节 ADC functional description** 里 regular/injected 两小节，以及 **第 11.13 节 ADC registers**（SR、CR1、CR2、SMPR1、SMPR2、SQR1~SQR3、JSQR、DR、JDR1~JDR4） |
| 内部温度传感器怎么用、接在哪个通道 | RM0008 **第 11.11 节 Temperature sensor** |
| **R_ADC = 1 kΩ、C_ADC = 8 pF**、VREF+ 的范围、输入电压的绝对最大额定值、内部温度传感器的 V25 = 1.43V 与 Avg_Slope = 4.3mV/℃、温度传感器采样时间不少于 17.1µs | **STM32F103x8/xB 数据手册**（DS5319 系列）里的电气特性表：*ADC characteristics*（R_ADC、C_ADC、t_STAB 这几行）、*Absolute maximum ratings*、*Temperature sensor characteristics* |
| 小封装（LQFP48）上 VREF+ 内部接 VDDA | 数据手册的引脚定义表 / 引脚描述（48 脚封装没有引出 VREF+） |
| 输入阻抗、采样时间与精度的关系（第 2.4 节那套推导的官方版本） | ST **应用笔记 AN2834** *How to get the best ADC accuracy in STM32 microcontrollers* |

按规范补一句：**上面我只写章节号和章节标题，没有写页码**——因为我手上的 PDF 版本和你看的版本页码不一定一样，写一个我记不清的页码反而会误导人。章节标题是各版本都一致的，按标题搜一定能定位到。

## 4. 完整代码

### 4.1 工程里有哪些文件，哪些是生成的

| 文件 | 谁写的 | 内容 |
|---|---|---|
| `Core/Src/main.c` | **CubeMX 生成框架 + 我改的** | 时钟树、三个外设初始化、主循环、ADC 校准与读取、串口打印 |
| `Core/Src/adc.c` / `adc.h` | CubeMX 生成（我改了采样时间） | `MX_ADC1_Init()`，两个通道的 Rank 和采样时间 |
| `Core/Src/usart.c` / `gpio.c` | CubeMX 生成，没动 | USART1 115200 8N1；PA1 被自动配成模拟输入 |
| `filter.h` / `filter.c` | **我自己写的** | 中值滤波 + 一阶低通 + 滑动平均，**与硬件无关的纯 C** |

我所有的代码都写在 CubeMX 的 `/* USER CODE BEGIN ... */` 和 `/* USER CODE END ... */` 之间——这样以后回 CubeMX 改个引脚、重新生成代码，我写的东西不会被冲掉。**这一点从一开始就得养成习惯**，我见过有人把所有代码写在生成区里，重新生成一次全没了。

`filter.c` 单独拆出来是有意为之：它不 include 任何 HAL 头文件，也不碰寄存器，所以能拿到 PC 上单独编译测试（第 4.4 节末尾写了编译命令）。控制算法和硬件驱动混在一个文件里，是后面做闭环时最难查的一类问题。

### 4.2 `main.c`

```c
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
```

### 4.3 `filter.h`

```c
/**
 * filter.h —— 三个常用的 ADC 软件滤波器（与硬件无关的纯 C 实现）
 *
 * 设计目标：
 *   1. 不 include 任何 HAL / STM32 头文件，只依赖 <stdint.h>。
 *      这样它既能直接丢进 STM32 工程，也能在 PC 上单独编译、喂数据做单元测试；
 *   2. 不用 malloc：嵌入式里运行期动态分配是能避就避的，全部状态都放在
 *      调用者自己的结构体里（想清楚内存归谁管）；
 *   3. 用 uint16_t / uint32_t 存 ADC 原始值而不是 float：
 *      F103 的 ADC 是 12 位，0~4095 用整数存最省也最准，
 *      只有到"算电压"和"低通滤波"这两步才转成 float。
 *
 * 三个滤波器各自对付什么：
 *
 *   MedianFilter_t  中值滤波   —— 脉冲噪声（电机换向、继电器动作打出来的单个尖峰）
 *   MovAvg_t        滑动平均   —— 随机噪声（热噪声、电源纹波引起的上下抖动）
 *   LowPass_t       一阶低通   —— 通用平滑，算力最省，且相位滞后可以算出来
 *
 * 注意：这三个都不是"越多越好"。每加一级都在往系统里塞延迟，
 * 具体怎么算延迟见 docs/07-ADC采样.md 第 2.7 节。
 */
#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================================================================
 * 1. 中值滤波（median filter）
 * ================================================================== */

/**
 * 窗口长度取奇数。
 *   5 是中位数滤波在单片机上的常用值：既能干掉连续 2 个点的尖峰，
 *   又只需要一个 5 元素的临时数组（10 字节栈空间）。
 * 取偶数会出现"中间两个数取哪个"的歧义，我没有理由给自己找麻烦。
 */
#define MEDIAN_WIN   5u

typedef struct {
    uint16_t buf[MEDIAN_WIN];   /* 环形缓冲：存最近 MEDIAN_WIN 个原始值 */
    uint8_t  idx;               /* 下一个要写入的位置 */
    uint8_t  count;             /* 已经填进去多少个（窗口没满时用它） */
} MedianFilter_t;

/* 初始化：把缓冲清零，count 归零。上电必须调一次 */
void     Median_Init(MedianFilter_t *f);

/**
 * 喂一个新采样值，返回窗口内的中位数。
 * 窗口没填满时，返回的是"已填部分的中间值"，不是 0 —— 这样上电前几个点也能用。
 */
uint16_t Median_Update(MedianFilter_t *f, uint16_t x);

/* ==================================================================
 * 2. 一阶低通滤波（first-order IIR low-pass）
 * ================================================================== */

typedef struct {
    float   alpha;      /* 滤波系数，范围 (0, 1]。越小平滑越强、滞后越大 */
    float   y;          /* 内部状态：上一次的输出 */
    uint8_t started;    /* 是否已经吃过第一个值 */
} LowPass_t;

/**
 * 用 alpha 直接初始化。
 *   alpha = 1.0f  →  不滤波，输出等于输入
 *   alpha = 0.2f  →  每次只把 20% 的新值混进来
 */
void  LowPass_Init(LowPass_t *f, float alpha);

/**
 * 用"时间常数"初始化，这是工程上更好用的写法。
 *   tau_s —— 时间常数 τ，单位秒。τ 越大越平滑、相位滞后越大。
 *   dt_s  —— 采样周期，单位秒。
 *
 *   alpha = dt / (τ + dt)
 *
 * 推导：一阶低通的连续形式是  τ·dy/dt + y = x，
 *       用后向差分 dy/dt ≈ (y[n] - y[n-1]) / dt 代进去整理，就得到
 *       y[n] = y[n-1] + α·(x[n] - y[n-1])，其中 α = dt/(τ+dt)。
 * 这是近似式，τ ≫ dt 时和精确式 α = 1 - e^(-dt/τ) 基本重合。
 */
void  LowPass_SetTau(LowPass_t *f, float tau_s, float dt_s);

/* 直接改 alpha（运行期调速用） */
void  LowPass_SetAlpha(LowPass_t *f, float alpha);

/**
 * 喂一个新值，返回滤波结果。
 *   y += α·(x − y)
 * 第一次调用直接把 x 当作 y 的初值，否则 y 从 0 往上爬，前几十个点是错的。
 */
float LowPass_Update(LowPass_t *f, float x);

/* 清状态（重新开始闭环、或者换量程时用） */
void  LowPass_Reset(LowPass_t *f);

/* ==================================================================
 * 3. 滑动平均（moving average，又叫算术平均滤波）
 * ================================================================== */

/**
 * 窗口长度取 2 的整数次幂。
 * 这样 % MOVAVG_WIN 会被编译器优化成一条按位与指令，
 * 在 72MHz 的 M3 上比真除法快十几倍（除法要做 2~12 个周期）。
 */
#define MOVAVG_WIN   8u

typedef struct {
    uint32_t buf[MOVAVG_WIN];   /* 环形缓冲 */
    uint32_t sum;               /* 窗口内所有值之和，靠它避免每次重新累加 */
    uint8_t  idx;               /* 下一个要覆盖的位置（窗口满时它指向最老的值） */
    uint8_t  count;
} MovAvg_t;

void     MovAvg_Init(MovAvg_t *f);

/**
 * 喂一个新值，返回当前窗口的平均值。
 * 用一个 sum 变量做"进一个、出一个"，每次 O(1)，
 * 不用像笨办法那样每次都把 8 个数重新加一遍。
 * 返回的是整数平均值（向下取整），省一次浮点除法。
 */
uint32_t MovAvg_Update(MovAvg_t *f, uint32_t x);

/* 只读当前平均值，不喂新数据 */
uint32_t MovAvg_Value(const MovAvg_t *f);

/* ==================================================================
 * 4. 组合：中值 → 滑动平均 → 低通
 * ==================================================================
 * 顺序不是随便排的：
 *   先中值，把脉冲尖峰整个"抠掉"（滑动平均对尖峰只能把它摊薄成 1/N，
 *   尖峰还在里面）；
 *   再滑动平均，压随机噪声；
 *   最后低通，给后面的控制环一个平滑量。
 * 如果顺序反了（先平均再中值），一个 200 LSB 的尖峰被平均成 25 LSB，
 * 中值滤波就再也认不出它是异常值了。
 */

typedef struct {
    MedianFilter_t med;
    MovAvg_t       avg;
    LowPass_t      lp;
} AdcFilterChain_t;

/**
 * 初始化整条链。
 *   tau_s —— 最后一级低通的时间常数，单位秒
 *   dt_s  —— 采样周期，单位秒
 */
void     AdcFilterChain_Init(AdcFilterChain_t *c, float tau_s, float dt_s);

/* 喂原始 ADC 值，返回滤波后的浮点值（单位还是"LSB"，换算成 mV 是调用者的事） */
float    AdcFilterChain_Update(AdcFilterChain_t *c, uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
```

### 4.4 `filter.c`

```c
/**
 * filter.c —— 中值滤波 / 一阶低通 / 滑动平均 的实现
 *
 * 这个文件里没有任何一行跟 STM32 有关的东西：没有寄存器、没有 HAL_、
 * 没有中断。所有状态都在调用者传进来的结构体指针里。
 *
 * 这么写就是为了能在 PC 上单独测试：
 *     gcc -Wall -Wextra -std=c99 -I. filter.c test_filter.c -o test_filter
 * 其中 test_filter.c 自己提供 main()，往里喂一串构造好的数据，
 * 检查输出是不是预期值（比如喂 100,100,100,4000,100 给中值滤波，
 * 输出必须还是 100 —— 那个 4000 是单个尖峰，应该被丢掉）。
 *
 * 🔬 上面这条编译命令我还没实际跑过（当时电脑上没装 gcc），
 *    先在注释里记下来，等后面补单元测试的时候一起验证。
 */
#include "filter.h"

/* ==================================================================
 * 1. 中值滤波
 * ================================================================== */

void Median_Init(MedianFilter_t *f)
{
    uint8_t i;

    if (f == 0) {
        return;
    }
    for (i = 0u; i < MEDIAN_WIN; i++) {
        f->buf[i] = 0u;
    }
    f->idx   = 0u;
    f->count = 0u;
}

uint16_t Median_Update(MedianFilter_t *f, uint16_t x)
{
    uint16_t tmp[MEDIAN_WIN];
    uint16_t key;
    uint8_t  i, j, n;

    if (f == 0) {
        return x;
    }

    /* 1) 新值写进环形缓冲，覆盖掉最老的那个 */
    f->buf[f->idx] = x;
    f->idx = (uint8_t)((f->idx + 1u) % (uint8_t)MEDIAN_WIN);
    if (f->count < (uint8_t)MEDIAN_WIN) {
        f->count++;
    }
    n = f->count;

    /* 2) 先复制一份出来再排序。
     *    不能直接对 f->buf 排序：环形缓冲里元素的"数组下标顺序"
     *    不代表"时间先后顺序"，原地排完就分不清谁新谁旧了，
     *    下一轮覆盖的位置也就错了。 */
    for (i = 0u; i < n; i++) {
        tmp[i] = f->buf[i];
    }

    /* 3) 插入排序。
     *    N = 5 时最坏 10 次比较，比调用通用快排快得多，
     *    而且不递归、不会吃掉几十字节的栈空间。 */
    for (i = 1u; i < n; i++) {
        key = tmp[i];
        j   = i;
        while ((j > 0u) && (tmp[j - 1u] > key)) {
            tmp[j] = tmp[j - 1u];
            j--;
        }
        tmp[j] = key;
    }

    /* 4) 取正中间那个。
     *    窗口取奇数就是为了这一步没有"取哪一个"的歧义。 */
    return tmp[n >> 1];
}

/* ==================================================================
 * 2. 一阶低通
 * ================================================================== */

void LowPass_Init(LowPass_t *f, float alpha)
{
    if (f == 0) {
        return;
    }
    LowPass_SetAlpha(f, alpha);
    f->y       = 0.0f;
    f->started = 0u;
}

void LowPass_SetAlpha(LowPass_t *f, float alpha)
{
    if (f == 0) {
        return;
    }
    /* 夹到 [0, 1]：alpha > 1 会让滤波器发散（输出越振越大），
     * alpha < 0 会让输出反号，两种都是灾难 */
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    f->alpha = alpha;
}

void LowPass_SetTau(LowPass_t *f, float tau_s, float dt_s)
{
    if (f == 0) {
        return;
    }
    if (tau_s <= 0.0f) {
        /* τ = 0 表示不要滤波，直接跟随输入 */
        LowPass_SetAlpha(f, 1.0f);
        return;
    }
    /* α = dt / (τ + dt) */
    LowPass_SetAlpha(f, dt_s / (tau_s + dt_s));
}

float LowPass_Update(LowPass_t *f, float x)
{
    if (f == 0) {
        return x;
    }

    if (f->started == 0u) {
        /* 第一个值直接当初始状态。
         * 如果这里让 y 从 0 开始爬，τ = 20ms 时要爬 3τ ≈ 60ms
         * 才接近真值，这段时间里输出全是错的。 */
        f->y       = x;
        f->started = 1u;
        return f->y;
    }

    /* 一行就是全部：新状态 = 老状态 + α × (新输入 − 老状态) */
    f->y += f->alpha * (x - f->y);
    return f->y;
}

void LowPass_Reset(LowPass_t *f)
{
    if (f == 0) {
        return;
    }
    f->y       = 0.0f;
    f->started = 0u;
}

/* ==================================================================
 * 3. 滑动平均
 * ================================================================== */

void MovAvg_Init(MovAvg_t *f)
{
    uint8_t i;

    if (f == 0) {
        return;
    }
    for (i = 0u; i < MOVAVG_WIN; i++) {
        f->buf[i] = 0u;
    }
    f->sum   = 0u;
    f->idx   = 0u;
    f->count = 0u;
}

uint32_t MovAvg_Update(MovAvg_t *f, uint32_t x)
{
    if (f == 0) {
        return x;
    }

    if (f->count < MOVAVG_WIN) {
        /* 窗口还没填满：只进不出 */
        f->buf[f->idx] = x;
        f->sum += x;
        f->count++;
    } else {
        /* 窗口满了：进一个新值、出一个最老的值。
         * 必须"先减后加"，否则如果 f->idx 刚好指到元素本身，
         * 加进去的东西会被下一次减法减掉，平均值会慢慢塌下去。 */
        f->sum -= f->buf[f->idx];
        f->buf[f->idx] = x;
        f->sum += x;
    }

    /* 2 的整数次幂 → 编译器把取模变成按位与 */
    f->idx = (uint8_t)((f->idx + 1u) % (uint8_t)MOVAVG_WIN);

    return f->sum / f->count;
}

uint32_t MovAvg_Value(const MovAvg_t *f)
{
    if ((f == 0) || (f->count == 0u)) {
        return 0u;
    }
    return f->sum / f->count;
}

/* ==================================================================
 * 4. 组合滤波链
 * ================================================================== */

void AdcFilterChain_Init(AdcFilterChain_t *c, float tau_s, float dt_s)
{
    if (c == 0) {
        return;
    }
    Median_Init(&c->med);
    MovAvg_Init(&c->avg);
    LowPass_Init(&c->lp, 1.0f);          /* 先置成"不滤波" */
    LowPass_SetTau(&c->lp, tau_s, dt_s); /* 再按 τ 算 α */
}

float AdcFilterChain_Update(AdcFilterChain_t *c, uint16_t raw)
{
    uint16_t med;
    uint32_t avg;

    if (c == 0) {
        return (float)raw;
    }

    med = Median_Update(&c->med, raw);           /* 第一级：抠掉脉冲尖峰 */
    avg = MovAvg_Update(&c->avg, (uint32_t)med); /* 第二级：压随机噪声 */
    return LowPass_Update(&c->lp, (float)avg);   /* 第三级：一阶低通 */
}
```

## 5. 逐段解释

### 5.1 为什么校准必须放在最前面，而且只能做一次

```c
ADC1_Calibrate();     /* 在 MX_ADC1_Init() 之后、进 while(1) 之前 */
```

校准做的是**消除内部电容网络和比较器的固有失调**：芯片内部那组电容不可能做得完全精确，总有几个 LSB 的固定偏差。校准就是让硬件自己测一遍这个偏差并记下来，之后每次转换都自动减掉。

三个必须记住的点：

1. **必须在 ADC 上电之后做**（`MX_ADC1_Init()` 里已经使能了 ADC 时钟，所以放在它后面就对了）；
2. **必须在第一次转换之前做**，做完再转换；
3. **只做一次就够**，不要放进主循环——校准本身要花 83 个 ADC 周期（手册里给的固定值），每轮都做纯属浪费。

不校准的后果是**一个固定的偏移**（第 6 节坑 5 会算成 mV）。它不会让读数来回跳，而是让所有读数整体偏一点——所以从串口上看起来"挺稳的"，反而更难发现。

### 5.2 为什么读电压要写成"乘 LSB"而不是"除 4096"

```c
static float ADC_RawToMv(uint16_t raw)
{
  return ((float)raw) * LSB_MV;      /* LSB_MV = 3300.0f / 4096.0f */
}
```

两个原因：

- **只有一个地方能写错分母。** `LSB_MV` 是个宏，改一处就等于改了全工程。要是我在十个地方各写一遍 `raw * 3.3f / 4096.0f`，迟早有一处会写成 `4095`，而且这种错很难看出来（读数只差 0.024%）。
- **乘比除快。** F103 没有 FPU，浮点除法是软件库里的几十到上百个周期，浮点乘法差不多是一半。这里每次循环都调用，能省就省。

### 5.3 为什么把电压换算成"µV 整数"再打印

```c
uv_raw = (uint32_t)(mv_raw * 1000.0f + 0.5f);
(void)snprintf(line, sizeof(line), "%lu, %u, %lu.%03lu, %lu.%03lu\r\n", ...);
```

`printf` 的 `%f` 在 Keil 的 MicroLIB 里是**一套相当占空间的浮点格式化代码**，而且我这次只需要 3 位小数。做法是把 mV 乘 1000 变成整数 µV，然后用 `%lu.%03lu` 手工打出小数点：

```text
mv_raw = 1650.0000 mV
  → uv_raw = 1650000 µV
  → 整数部分 1650000 / 1000 = 1650，小数部分 1650000 % 1000 = 000
  → 打印 "1650.000"
```

`+ 0.5f` 是四舍五入（`(uint32_t)` 强制转换是截断的）。12 位 ADC 满量程 3300mV，用 µV 表示最多 7 位数，`uint32_t` 装得下（上限 42.9 亿）。

顺带一提，这里也**顺便解决了第 2.3 节说的"SAR 向下取整"问题**：`uv_raw` 是按"最近"取的，但 raw 本身还是向下取整的结果。真要完全补偿，得在 raw 上先加 0.5 再乘 LSB，我在第 2.3 节写了那一行。

### 5.4 为什么打印要"4 次打一次"，而不是每次采样都打

这是我把数算完之后改的设计。115200 8N1 下一个字节 10 位（1 起始 + 8 数据 + 1 停止），一个字符的时间：

```text
1 位时间 = 1 / 115200 s = 8.6806 µs
1 个字节 = 10 位 = 86.806 µs
```

我这行输出是：

```text
"1234, 2048, 1650.000, 1649.512\r\n"
```

数一下大约 **30 个字符**，纯等待时间：

```text
30 × 86.806 µs = 2604.2 µs ≈ 2.60 ms
```

而我的采样周期是 **5ms**：

```text
2.60 ms / 5 ms = 52%
```

**每次采样都打印，一半以上的时间都在等串口!** 后果不是"慢一点"这么轻——采样周期会被不可控地拉长到 7.6ms 左右，滤波器的 α 就不再是我按 dt=5ms 算出来的那个值了，时间常数实际会偏大 50%。最后表现在数据上就是"滤波比我算的更迟钝"，但代码里一个错都找不到。

所以改成每 4 次采样打一次：

```text
采样周期依然是 5ms（滤波器按 5ms 算，参数没错）
打印周期 = 4 × 5ms = 20ms  →  50Hz 的曲线刷新率
打印占空比 = 2.60 ms / 20 ms = 13%
```

**50Hz 画曲线完全够用**（串口助手的绘图功能本身也就刷这么快）。要更密的曲线，把 `PRINT_DIV` 调小；要更准的时间基准，就得按注释里说的改用定时器触发采样。

### 5.5 `HAL_ADC_Start` / `PollForConversion` / `GetValue` 这三步在做什么

```c
HAL_ADC_Start(&hadc1);
if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK) {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);
}
HAL_ADC_Stop(&hadc1);
```

对应硬件上是三步：

| HAL 调用 | 底层干了什么 |
|---|---|
| `HAL_ADC_Start()` | 把 CR2 的 `ADON` 位置 1 给 ADC 上电，然后用 `SWSTART` 位软件触发一次转换（因为我把 `ExternalTrigConv` 配成了 `ADC_SOFTWARE_START`） |
| `HAL_ADC_PollForConversion()` | **死等** SR 寄存器的 `EOC`（End Of Conversion）标志位变 1，超时 10ms 就返回错误 |
| `HAL_ADC_GetValue()` | 读 `DR` 数据寄存器，同时（在 HAL 里）清掉 EOC 标志，为下一次转换做准备 |

**这里有个真实的陷阱**：`PollForConversion` 是**阻塞**的，一次转换 2.17µs（13.5 周期档），它就在那儿空转 2.17µs。在 5ms 的周期里这无所谓，但如果这段代码被放进 20kHz 的中断里，2.17µs 就占掉中断周期的 4.3%，而且 CPU 在这段时间里什么都干不了。

工程上的做法是：**慢速监测用轮询（就像我这样，代码最简单），和 PWM 同步的高速采样用定时器触发 + DMA 或注入中断**——CPU 只在转换完成时被打断一次，中间该干什么干什么。这也正好对应第 2.6 节说的"注入组是给电流采样用的"。

### 5.6 多通道扫描为什么能一个通道一个通道读

```c
HAL_ADC_Start(&hadc1);
for (i = 0u; i < n; i++) {
    if (HAL_ADC_PollForConversion(&hadc1, 10u) != HAL_OK) { break; }
    dst[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
}
HAL_ADC_Stop(&hadc1);
```

在扫描模式下，ADC 会按 `SQR1~SQR3` 里排好的顺序（Rank1 → Rank2 → …）自动往下转。F103 的 `EOC` 是**每转完一个通道置一次**，所以每 `PollForConversion` 成功一次，`DR` 里就是下一个通道的结果，依次读出来正好对上 `Rank` 的顺序。

**但这套写法有个前提**：`ADC_CH_NUM` 必须和 CubeMX 里配的 `NbrOfConversion`、以及 `MX_ADC1_Init()` 里 `ConfigChannel` 的次数**三者完全一致**。哪个地方多一个少一个，读出来的数组就整体错位——温度值被当成电位器电压这种事，从串口上看就是"电位器怎么不动"，能查一晚上。

**它也不是没有隐患**：每个通道之间都是阻塞轮询，CPU 全程陪着。通道一多（比如同时测电池、母线、三相电流），建议改成 DMA 搬运：

```text
CubeMX 里给 ADC1 加 DMA 请求，模式选 Circular（循环），
主循环只读 DMA 缓冲区里的数组 —— CPU 一秒都不用等。
（⚠️ Wokwi 的 STM32F103 仿真没有实现 DMA，这条只能上真板子验证。）
```

### 5.7 滤波链为什么要按"中值 → 滑动平均 → 低通"这个顺序

```c
med = Median_Update(&g_filter_pot.med, raw);
avg = MovAvg_Update(&g_filter_pot.avg, (uint32_t)med);
mv_filt = LowPass_Update(&g_filter_pot.lp, ADC_RawToMv((uint16_t)avg));
```

第 2.7 节讲过原则（先抠尖峰、再压随机、最后平滑），这里说两个实现上的细节：

**（1）中值必须先做，而且要在"原始 LSB"上做，不要先乘 LSB 变成 mV。**

两个理由：中值滤波比较的是**数值大小顺序**，整数比较没有误差，转成float 再比纯属给自己找麻烦；而且 `uint16_t` 的比较和交换比 float 快得多。所以 `Median_Update` 和 `MovAvg_Update` 都在整数域里跑，到低通那一级才 `ADC_RawToMv` 转成 mV。

**（2）整流和滤波的顺序不要反。**

```text
原始值 → 中值（整数） → 滑动平均（整数） → 换算成 mV（float） → 低通（float）
```

要是我把低通放到最前面，那 4000 的尖峰就会先在低通里混出一个"半高不低"的中间值，把后面中值滤波的判断也带歪。**一级一级各管各的，是这套组合能工作的前提。**

### 5.8 `LowPass_Update` 里那个 `started` 标志为什么不能省

```c
if (f->started == 0u) {
    f->y       = x;
    f->started = 1u;
    return f->y;
}
```

因为**一阶低通是有"记忆"的**：输出是从上一个状态慢慢挪过来的。如果 `y` 的初值留在 0，上电后第一秒的输出会是这样：

```text
τ = 20ms，dt = 5ms，α = 5/(20+5) = 0.2
从 0 开始，每次把差距的 20% 补上：
  第 1 拍：剩余差距 80%
  第 2 拍：80% × 0.8 = 64%
  ...
  第 n 拍：剩余 0.8^n
要让误差小于 1%（相当于小于 1/8 个 LSB 的目标），需要：
  0.8^n < 0.01  →  n > ln(0.01)/ln(0.8) = 4.605/0.223 = 20.6 拍
  = 20.6 × 5ms = 103 ms
```

**第一秒里有 100ms 的数据是逐步爬上去的假值。** 如果不做处理，上位机看到的曲线起始段是一段平滑的爬升——很容易被误当成"传感器响应慢"或者"电路有 RC 滤波"，其实是软件的问题。直接吃进第一个真值，就把这段假数据整个消掉了。

### 5.9 `MovAvg_Update` 里"先减后加"的顺序为什么不能反

```c
f->sum -= f->buf[f->idx];    /* 先减掉最老的 */
f->buf[f->idx] = x;          /* 再覆盖 */
f->sum += x;                 /* 最后加上新的 */
```

`f->idx` 指向的那个位置，就是我正要覆盖的位置，它里面装的**正好是最老的那个值**。所以顺序必须写成"减掉它 → 再覆盖 → 再加新值"。

要是反过来先覆盖再减：

```c
f->buf[f->idx] = x;
f->sum -= f->buf[f->idx];    /* 减掉的是刚写进去的 x，不是最老的值！ */
f->sum += x;                 /* 又加了一遍 x */
```

后果是**累加和每轮都会少掉一个最老值、多加一个新值**，`sum` 会越来越小，平均值慢慢"塌"下去。这个 bug 特别阴——数组本身是对的，只有平均值错，而且不是跳变，是缓慢漂移，肉眼看串口根本看不出来，得拿已知的固定输入（比如把 PA1 直接接 3V3，读数应该是 4095 附近）才对得出来。

**这也是我在文件头注释里写"要能在 PC 上单独测试"的原因**：这种错在 PC 上喂一组固定数据、跑 100 次比较输出，五秒钟就能发现；在板子上靠串口看，可能要盯一晚上。

### 5.10 `LSB_MV` 为什么用宏，而 ADC 的时间参数为什么写在注释里

```c
#define LSB_MV    (VREF_MV / ADC_FULL_SCALE)
```

`LSB_MV` 是**编译期常量**，编译器会直接把它折叠成一个数（甚至可能把整个 `raw * LSB_MV` 优化掉一部分），而且用括号包住整表达式，避免 `2 * LSB_MV` 之类的地方被运算符优先级坑到。

至于采样时间那类只能从固定档位里选、或者要跟手册对上的参数（比如 17.1µs、12.5 周期），我都写在注释里而不是写成宏。因为**它们的用途是"让我下次查手册时能一眼看到当初为什么这么选"**，而不是拿来计算的。宏一旦被用到代码里，改注释就改不动了。

## 6. 我踩的坑

**先说清楚这个前提**：这一章我还没上板焊接、没烧录过（状态标的是 🔬 待上机验证），所以下面这几条**不是"我测出来不对劲"的坑**，而是我在**写代码、算参数、查手册的过程中真实踩到的**——包括算错账、以及差点把板子烧掉的那一次。每条我都写清两件事：**错在哪**，以及**上板之后怎么验证它**。等我实测完会回来把实测数字补上。

### 坑 1：读数恒为 0（而且一度以为是电路接错了）

**现象**（🔬 预期会遇到的经典现象，也是我一开始照着网上代码写时的状态）：串口打出来的 `raw` 一直是 0，电压 0.000mV，拧电位器一点反应都没有。

**我一开始的思路**是去怀疑硬件：跳线松了？电位器坏了？万用表量了分压点明明有电压。

**原因有两层，都在软件**：

1. **ADC 时钟没开。** HAL 的 `HAL_ADC_Init()` 里面会开 ADC1 的时钟（`__HAL_RCC_ADC1_CLK_ENABLE()`），但如果直接操作寄存器、或者用的是自己把 CubeMX 代码抄错一行的版本，时钟没开的时候读写 ADC 寄存器**不会报错，也不会触发硬件错误，读回来就是 0**。这是最阴的一点：**外设时钟没开，寄存器访问是静默失败的。**
2. **忘了调用 `HAL_ADC_Start()`。** 轮询模式下没有 `Start` 就没有 `SWSTART`，ADC 根本没被触发，`DR` 一直是 0。`HAL_ADC_PollForConversion` 会等 10ms 然后超时——**如果没检查它的返回值**（我第一版就是直接忽略返回值），超时被悄悄吞掉，程序看起来一切正常，输出全是 0。

**怎么定位**：

```text
① 在 MX_ADC1_Init() 之后加一句 __HAL_RCC_ADC1_CLK_ENABLE(); 再读一次
   —— 如果读数活了，就是时钟问题
② 检查 HAL_ADC_PollForConversion 的返回值，超时就在串口打一行 "ADC timeout"
   —— 这是我后来固定加上的习惯：所有 HAL 的返回值都要看一眼
③ 用调试器看 ADC1->CR2 寄存器的 ADON 位是不是 1
```

**上板后的验证方法**：把 PA1 直接短接到 GND，读数应该稳定在 0~2 之间；短到 3V3，应该稳定在 4093~4095。**这两个数是"绝对锚点"**——不接任何东西的话，引脚悬空，读数是什么都有可能（悬空引脚是天线）。

### 坑 2：采样时间留在默认档，高阻分压的读数会严重偏低

**这是我算完之后才发现自己想错了的地方。** 我原本以为"采样时间设短一点，顶多让读数偏个百分之几"。

**实际情况**：用 **100kΩ + 10kΩ 分压**（很常见的配置，因为电阻大、不费电）接到 PA1，采样时间如果留在默认的 **1.5 个周期**（12MHz 下 = 125ns）：

```text
分压点的等效源阻抗 R_AIN = 100 kΩ ∥ 10 kΩ = (100 × 10) / (100 + 10) = 9.0909 kΩ

RC 时间常数 = (R_AIN + R_ADC) × C_ADC
            = (9090.9 Ω + 1000 Ω) × 8 pF
            = 10090.9 × 8×10⁻¹²
            = 8.0727×10⁻⁸ s
            = 80.73 ns

采样窗口 125 ns 内，采样电容充到的比例：
  t_s / RC = 125 ns / 80.73 ns = 1.5484
  1 − e^(−1.5484) = 1 − 0.2126 = 78.74%

即：输入 1.0000 V，采样电容上只有 0.7874 V
    读数偏低 (1 − 0.7874) / 1 = 21.3%
```

**偏低 21%**，而且这不是"静差可以标定掉"的那种误差——**它和输入电压的大小、温度、每一块板子的寄生电容都有关系，是变动的**，标定不了。

需要的采样时间按第 2.4 节的公式算：

```text
t_s ≥ 9.02 × (9090.9 + 1000) × 8 pF
    = 9.02 × 10090.9 × 8×10⁻¹²
    = 7.2816×10⁻⁷ s
    = 728.2 ns

12MHz 下需要 728.2 / 83.333 = 8.74 个周期
→ 必须选 13.5 周期（1125.0 ns）或更长
```

我第一次算的时候直接把电位器那种"低阻源"的经验套到分压电路上，选了 7.5 周期（625ns < 728.2ns），**差一点点不够**——这种"差一点"是最难发现的，因为读数只是轻微偏低，看着像是电阻精度问题。所以我现在一律**往上取一档**：算出来 8.74 就选 13.5，不选 7.5。

**而且这个误差不是线性的**，这一条我在第 2.4 节专门列了表：源阻抗 9.09kΩ 时偏低 21%，到了 50kΩ（两个 100kΩ 分压）就偏低 **73.6%**。**再往上加一点电阻，读数就直接奔着 0 去了。** 所以"分压电阻取大一点省电"这个看起来很自然的选择，必须和采样时间一起考虑。

**上板后的验证方法**：把同一个分压点分别用不同的采样时间读一遍（1.5 / 7.5 / 13.5 / 55.5 周期各测一组），对照万用表量出来的真实电压。**如果 1.5 周期那组的读数明显低于 55.5 周期那组，就说明第 2.4 节这套推导在真板子上是成立的。** 这是我上板后第一个要做的实验。

### 坑 3：差点把 12V 电池直接接到 ADC 引脚上（这次是真危险）

我一开始的想法很直接："要测 12V 电池，那就把电池正极接到 PA1 上，读出来乘 12 不就行了。"

**这个想法会烧板子。** 翻数据手册的绝对最大额定值表才看到：**模拟输入（analog input）的电压不能超过 VDDA + 0.3V**，我这块板子 VDDA = 3.3V，所以上限是 **3.6V**。

```text
12 V 比 3.6 V 超出 8.4 V

超出之后发生什么：引脚内部的 ESD 保护二极管会正向导通，
电流从引脚灌进 VDDA 这条 3.3V 网络。后果分两种情况：

情况 A：源阻抗是几欧姆（电池直接怼上去）
  电流 ≈ (12 V − 3.6 V) / 几欧姆 ≈ 几安培量级
  → 保护二极管和引脚金属化层瞬间烧断，这个脚就废了

情况 B：源阻抗是 100 kΩ（比如分压电阻上端虚焊，12V 通过 100kΩ 漏到引脚）
  电流 ≈ (12 V − 3.6 V) / 100 kΩ = 84 µA
  → 引脚可能不会马上坏，但这 84µA 灌进 VDDA 网络会把这根 3.3V 轨抬高，
    VREF 跟着变 → 【同一个 ADC 上其它所有通道的读数全都变错】
    这种"一个脚接错、全板读数都乱"的现象，特别难往"某个引脚过压"上想
```

**还有个容易被忽略的细节**：F103 的很多 GPIO 是 **FT（5V 耐受）** 的，看起来"接 5V 也没事"。但手册明确写了 **FT 只在数字模式下 5V 耐受，配成模拟输入就不享受这个待遇**。PA1 在 CubeMX 里被 ADC 占用时会被自动配成模拟模式，所以它的上限就是 VDDA + 0.3V = 3.6V，**不是 5V**。这个坑对 5V 供电的传感器（很多编码器、霍尔传感器是 5V 输出）尤其致命。

**我现在给分压测量定的规矩**（三条）：

```text
① 分压比留余量：最高电压 / 分压比 ≤ 3.0V，不要贴着 3.3V 设计
   （留 0.3V 给电阻误差、电源波动、上电瞬间的过冲）
② 分压点并一个 100nF 到地 + 一个 3.3V 稳压管/肖特基做钳位
③ 上电前先用万用表量一次分压点的电压，确认 ≤ 3.0V 再插 ST-Link
```

**验证方法**：上板时先只接分压网络（不接主控），用万用表量分压点电压，确认在 3.0V 以内，再接 PA1。**这一步花 30 秒，能省一块板子。**

### 坑 4：读数一直在跳，最后靠"硬件电容 + 软件滤波"两条腿才稳住

**现象**（🔬 预期现象，具体跳动幅度我上板后会用实测数据替换）：电位器不动，串口打出来的 `raw` 在 **±20 个 LSB** 左右来回跳，也就是电压在 ±16mV 左右抖。

**诊断顺序**（这一步比结论重要）：

| 先看什么 | 怎么看 | 说明 |
|---|---|---|
| ① 电源 | 示波器/万用表 AC 档看 3V3 轨的纹波 | 3.3V 脏，所有 ADC 通道都脏，这是根子 |
| ② 引脚悬空？ | 检查接线 | 悬空引脚是天线，读数跳是必然的 |
| ③ 接线阻抗 | 换短的、焊死的线试试 | 面包板 + 杜邦线的接触电阻每次插都不一样 |
| ④ 只剩信号本身的噪声 | 前三项都排除了 | 那就老老实实滤波 |

**我的两条处理，一手抓硬件、一手抓软件：**

**硬件侧：在电位器中间脚（= ADC 引脚）对地并一个 100nF 陶瓷电容。** 它干了两件不同的事，我把数都算了一遍：

```text
作用 1：变成 ADC 的"就近电荷仓库"
  ADC 每次采样要从引脚上取走的电荷 = C_ADC × 满量程
                                    = 8 pF × 3.3 V = 26.4 pC
  如果这些电荷由 100nF 直接提供（而 100nF 比 8pF 大 12500 倍）：
  100nF 上的瞬时下跌 = 26.4 pC / 100 nF = 0.264 mV
  换算成码值 = 0.264 mV / 0.805664 mV = 0.33 个 LSB
  —— 代价被压到 1/3 个 LSB，可以接受

作用 2：它把"采样时间不够"这个问题换成了"采样率不能太高"
  100nF 自己也要靠 R_AIN 充电，时间常数
    τ = R_AIN × 100 nF = 2.5 kΩ × 100 nF = 250 µs
  两次采样之间必须留够时间让它充满（一般取 5τ = 1.25 ms）
  我的采样周期是 5 ms ≫ 1.25 ms ✅
  ⚠️ 但如果哪天把采样率提到 1kHz（周期 1ms），这个电容就开始拖后腿了
     ——【它会把平均电压拉低，读数量偏低】，跟采样时间不够是同一个病
     （🔬 这条推论我要专门上板验证一下，因为它反直觉：
       加电容明明是为了让读数更稳，结果采样太快反而会偏低）
```

再配一个 RC 低通的截止频率，顺手能算出来：

```text
f_c = 1 / (2π × R_AIN × C) = 1 / (2π × 2500 Ω × 100 nF)
    = 1 / (1.5708×10⁻³ s)
    = 636.6 Hz
```

**软件侧：中值 + 滑动平均 + 一阶低通**（第 4 节那套）。中值负责抠掉偶发的单点尖峰，滑动平均把剩下的随机噪声压 √8 = 2.83 倍，低通给一个连续可调的最终平滑。

**这里我给自己划了一条线**：软件滤波的**代价是延迟**（这套组合总共 47.5ms，第 2.7 节算过）。所以**先用硬件把噪声压下去，剩下的才交给软件**——顺序反过来的话，就得靠加大滑动平均窗口，延迟会白白多出去几十毫秒。**能用 100nF 解决的问题，不要用滤波器解决。**

上板后的验证方法：先不加电容、不滤波，记录 `raw` 的峰峰值；再加上 100nF，记录一次；最后三层滤波全开，记录一次。**三组数据的对比才是这一节真正的结论**，我现在只有算式，没有数据。

### 坑 5：忘了校准，读数有一个固定的偏移（而且看起来还挺"稳"）

**原因**：芯片内部那组用于逐次逼近的电容阵列不可能做得绝对精确，加上比较器的输入失调，会带来一个**固定的偏移误差**。数据手册的 ADC 精度表把误差分成几项单列：总未调整误差 ET、偏移误差 EO、增益误差 EG、微分非线性 ED、积分非线性 EL，**量级都在 1~2 个 LSB**（具体数值以你手上版本的数据手册里那张精度表为准，我不凭印象写数字）。

```text
2 个 LSB 的偏移，换算成电压：
  2 × 0.805664 mV = 1.611 mV
```

**为什么这个错特别难发现**：它不表现为"读数乱跳"，而是**所有读数整体偏一个固定值**。串口上看起来数字稳稳的，一点都不像有问题。只有拿万用表量了分压点的真实电压去对，才会发现"怎么总是差 1.6mV"。

**解决**：`HAL_ADCEx_Calibration_Start(&hadc1)`，上电做一次。

**但校准不是万能的**，它消掉的主要是**偏移**那部分，对**增益误差**（第 2.5 节那个 VDDA 实际是 3.28V 的问题）基本无能为力——**增益误差要靠外部基准或者两点标定来治**。所以我的做法是：**校准是必做的基础动作，标定是想要更高精度时的进阶动作，两个是两回事。**

**上板后的验证方法**：读一个固定的低噪声基准（比如把 PA1 接一个 1.5V 的基准或者干脆接 3V3 分压出来的稳定电压），比较"校准前"和"校准后"两组的平均值，差多少 LSB 就是校准消掉的偏移。**（🔬 这个差值我上板后补。）**

## 7. 复现步骤

### 7.1 STM32CubeMX 配置

1. **新建工程**：`File → New Project` → 在 Part Number 里搜 `STM32F103C8`，选中 **STM32F103C8Tx** → 双击（或者点 Start Project）。
2. **配时钟源**：`System Core → RCC`，把 **High Speed Clock (HSE)** 设成 **Crystal/Ceramic Resonator**（Blue Pill 板上有 8MHz 晶振）。Low Speed Clock (LSE) 保持 Disable。
3. **配时钟树**：切到 `Clock Configuration` 标签页。
   - `Input frequency` 填 **8 MHz**；
   - `PLL Source Mux` 选 **HSE**，`PLLMul` 选 **×9**；
   - `System Clock Mux` 选 **PLLCLK**，于是 **HCLK = 72 MHz**；
   - `APB1 Prescaler` 选 **/2** → APB1 = 36MHz；`APB2 Prescaler` 选 **/1** → APB2 = 72MHz；
   - **找到时钟树右边/下边的 `ADC Prescaler`（有的版本叫 ADC 那一栏），选 `/6`** → ADC 时钟 = 72/6 = **12 MHz**（≤14MHz ✅）。
4. **开 ADC 通道**：左边 `Analog → ADC1`，在右侧的引脚图上点 **PA1**，勾上 **IN1**（PA1 会变成绿色，表示被 ADC1_IN1 占用）。
5. **配 ADC 参数**：在下面的 `Parameter Settings` 里，`ADC_Settings` 这一组：
   | 选项 | 填什么 | 为什么 |
   |---|---|---|
   | Mode | Independent mode | 只有一个 ADC 在工作 |
   | Data Alignment | Right alignment | 12 位右对齐，读出来就是 0~4095 |
   | Scan Conversion Mode | **Enabled** | 要一次扫多个通道 |
   | Continuous Conversion Mode | **Disabled** | 单次转换，什么时候读由主循环决定 |
   | Discontinuous Conversion Mode | Disabled | 不用间断模式 |
   | Number Of Conversion | **2** | ⚠️ 必须和代码里的 `ADC_CH_NUM` 一致 |
   | External Trigger Conversion Source | Regular Conversion launched by software | 用 `HAL_ADC_Start()` 触发 |
6. **配两个 Rank**：在 `Rank` 表格里：
   - **Rank 1**：Channel = **ADC1_IN1**，Sampling Time = **55.5 Cycles**；
   - **Rank 2**：Channel = **Temperature Sensor Channel**，Sampling Time = **239.5 Cycles**（⚠️ 温度传感器必须长采样时间，第 2.5 节算过）。
   - 点了 Temperature Sensor 之后，CubeMX 会自动把 `TSVREFE` 那一位置上，不用手动管。
7. **开串口**：`Connectivity → USART1` → Mode 选 **Asynchronous**；`Parameter Settings` 里 Baud Rate = **115200**，Word Length = **8 Bits**，Parity = **None**，Stop Bits = **1**。PA9/PA10 会自动分配。
8. **配工程**：`Project Manager` 标签页 → `Project Name` 填 `adc_demo`，`Toolchain/IDE` 选 **MDK-ARM V5** → 点 **GENERATE CODE**。
9. **加滤波模块**：把 `code/05-adc/filter.c` 和 `filter.h` 复制到工程目录（比如 `Core/Src/` 和 `Core/Inc/`），然后在 Keil 里 **右键 Source Group → Add Existing Files to Group** 把 `filter.c` 加进去。只加 `.c` 不用加 `.h`。
10. **Keil 里勾 MicroLIB**：`Options for Target → Target → 勾 Use MicroLIB`（第 05 章踩过这个坑）。
11. **把第 4 节的代码填进 `main.c` 的 USER CODE 区**，`MX_ADC1_Init()` 那一份对照第 4.2 节抄进 `adc.c`（注意采样时间要手改，CubeMX 的默认值不是这两个）。

> 说明：CubeMX 6.x 各个小版本的选项文字会有细微差别（比如 "Regular Conversion launched by software" 在有些版本里叫 "Software start"）。**认选项的含义，不要死记文字**。对不上的时候，对照第 3 节去 RM0008 里查这个位的本意。

### 7.2 硬件接线

| Blue Pill | 接到哪 | 备注 |
|---|---|---|
| **PA1** | 电位器**中间脚** | 这是被测信号 |
| **3V3** | 电位器一端 | 电位器两端接 3V3 和 GND，中间脚才是分压输出 |
| **GND** | 电位器另一端 | 和板子共地 |
| **PA1 对 GND** | **100nF 陶瓷电容** | 坑 4 说的那个电容，就近放，引脚越短越好 |
| **PA9 (TX)** | USB-TTL 的 **RXD** | TX→RX 交叉接 |
| **PA10 (RX)** | USB-TTL 的 **TXD** | 这一章只用发送，但一起接上方便以后加命令 |
| **GND** | USB-TTL 的 **GND** | **必须共地**，否则串口全是乱码 |

### 7.3 串口助手设置

| 项 | 值 |
|---|---|
| 端口 | USB-TTL 对应的 COM 口（设备管理器里看） |
| 波特率 | **115200** |
| 数据位 / 校验 / 停止位 | **8 / None / 1** |
| 接收区 | 文本模式，**关掉自动换行**（每一行自带 `\r\n`） |

### 7.4 预期现象 🔬

> **下面全是预期现象，不是我测到的数据。** 我上板之后会回来把实测值替换掉。

1. 复位后串口先出两行表头：

   ```text
   [07] ADC1 ready: 12-bit, VREF=3.3V, 1LSB=0.806mV
   sample, raw, volt_mV, filt_mV
   ```

2. 数据行每 **20ms** 出一行（`PRINT_DIV = 4`，采样周期 5ms），长这样（数值是举例，不是实测）：

   ```text
   0, 2048, 1650.000, 1649.512
   1, 2049, 1650.806, 1649.623
   ```

3. **电位器拧到两端**，`raw` 应该分别稳定在 **4095 附近** 和 **0 附近**（不一定是 4095 和 0，因为电位器两端到电源地之间有接触电阻，加上 ADC 本身的偏移误差，两端各差几个 LSB 是正常的）。**如果两端差得超过 20 个 LSB，先去查接线和电位器。**
4. **电位器拧到中间**，`raw` ≈ 2048，`volt_mV` ≈ **1650.000**。
5. **`raw` 和 `volt_mV` 的抖动幅度**：🔬 待测。按坑 4 的分析，加上 100nF 之后应该比不加小很多——具体数字我等实测。
6. **`filt_mV` 变化时明显比 `volt_mV` 慢**，估算的滞后是 **47.5ms**（第 2.7 节算的：中值 10ms + 滑动平均 17.5ms + 低通 20ms）。快速来回拧电位器时，两列的差会拉到最大。
   - **验证这个数的方法**：用串口助手的波形功能，看 `volt_mV` 跳变和 `filt_mV` 跟到位之间差几个采样点。20ms 一个点，47.5ms 应该差 **2.4 个点**左右。对不上就说明采样周期不是我以为的 5ms（`HAL_Delay` 的抖动 + 打印开销，第 5.4 节讲过）。
7. 每秒额外出一行温度：

   ```text
   TEMP, 25.0, C, raw=1775
   ```

   ⚠️ **这个温度基本不能当真。** 两个原因：
   - 25℃ 时温度传感器输出典型值 1.43V，对应 `raw = 1430 / 3300 × 4096 = 1774.9`，ADC 会读到 **1775**，**这个数可以拿来验证通道有没有读对**；
   - 但手册给的 V25 = 1.43V 是**典型值**，个体差异能到 ±0.1V 量级，换算成温度就是：

     ```text
     ±100 mV / 4.3 mV/℃ = ±23.3 ℃
     ```

     也就是说，**这颗芯片报出来的"25℃"完全可能是 45℃ 或者 5℃**。所以它只能看"有没有在变、往哪边变"（比如用手按住芯片，读数应该往上走），**绝对不能当温度计用**。

### 7.5 三个必须做的验证实验

这三个实验合起来，才算是把这一章的原理在真板子上验证了一遍。**我做完全部补实测数据。**

| 实验 | 怎么改 | 应该看到什么 |
|---|---|---|
| **A. 验证采样时间的影响**（对应坑 2、第 2.4 节） | 把 PA1 改成接 **100kΩ + 10kΩ 分压**（分压点 1.09V @12V，或者用 3V3 分压出 1.5V），然后采样时间分别设 **1.5 / 7.5 / 13.5 / 55.5 周期**各记一组 | 1.5 周期那组应该明显偏低（算出来偏低 **21.3%**），13.5 及以上应该基本一致。**如果四组数一样，说明我第 2.4 节的推导在真板子上不成立，那我就得回去查哪儿想错了** |
| **B. 验证滤波链的延迟**（对应第 2.7 节） | 电位器从一头快速拧到另一头，看串口波形 | `filt_mV` 应该滞后 `volt_mV` 约 **2.4 个采样点（47.5ms）**。然后把 `PRINT_DIV` 改成 1 再试一次，看是不是只有打印周期变了、滞后不变 |
| **C. 验证 100nF 的作用**（对应坑 4） | 把 100nF 拆掉再测一次，记 `raw` 的峰峰值；装回去再记一次 | 不装电容时峰峰值应该明显更大。**这是"硬件滤波比软件滤波划算"这条结论的直接证据** |

### 7.6 没有硬件怎么办（Wokwi 能做什么、不能做什么）

我买板子之前用 [Wokwi](https://wokwi.com/) 在线仿真先把逻辑跑通过。对 ADC 这一章，**它的能力边界必须说清楚**：

| 能力 | 状态 | 能不能验证本章的内容 |
|---|---|---|
| ADC1 基础转换 | 🟡 只实现了 ADC1 的基础转换 | ✅ 可以验证：`raw` 读取、LSB 换算、串口打印格式、滤波算法的逻辑对不对 |
| 多通道扫描 | 🟡 支持有限 | 🔶 可以试试，但顺序对不对要上真板子确认 |
| **DMA** | ❌ **未实现** | ❌ 第 5.6 节说的 DMA 方案在仿真里跑不了 |
| **采样时间 / 源阻抗的影响** | ❌ 仿真里是理想器件，没有 RC 充电过程 | ❌ **第 2.4 节和坑 2 那套结论，仿真永远验证不了** |
| 内部温度传感器 / VREFINT | ❌ 仿真里没有真实的模拟前端 | ❌ 验证不了 |
| CAN / RTC / IWDG / PWR | ❌ 未实现 | — |

**所以我的做法是**：在 Wokwi 里验证**数字逻辑**（读数换算、滤波代码、打印格式），**模拟特性（采样时间、源阻抗、噪声、100nF 的作用）一律上真板子**。这个分界线我觉得挺重要——仿真能告诉你"代码跑得通"，但它永远不会告诉你"这个数准不准"。

## 8. 自测题（面试可能这么问）

**Q1. 12 位 ADC、参考电压 3.3V，1 个 LSB 是多少毫伏？如果读到 1024，电压是多少？**

**答**：

```text
LSB = 3300 mV / 2^12 = 3300 / 4096 = 0.805664 mV ≈ 0.806 mV

V = 1024 × 0.805664 mV = 825.00 mV = 0.825 V
```

**Q2. LSB 为什么除以 4096，而不是 4095？两者的区别是什么？**

**答**：除以 4096（`VREF / 2^n`）算的是**每个台阶的高度**——ADC 的真实行为是把 0~VREF 分成 4096 个等宽的输入区间，码值 k 代表输入落在 `[k·LSB, (k+1)·LSB)` 里，所以这个除法和硬件行为是对得上的。除以 4095（`VREF / (2^n − 1)`）是把**两个端点**（码值 0 和 4095）分别钉在 0V 和 VREF 上，算的是端点到端点的平均斜率——它的问题在于码值 4095 对应的输入区间其实一直延伸到 4096·LSB，并不等于 VREF。

两者差 `1/4096 ≈ 0.0244%`，也就是最后一位的权重，在 12 位上折算成 0.0002mV，很多时候无所谓。**我的原则是：算分辨率、算量化误差、算噪声折算，一律用 4096；要求"满量程时显示成 VREF"的显示换算可以用 4095。** 真正要准，就不该在这两个分母之间纠结，而是去做两点标定。

**Q3.（计算题）用 100kΩ 和 10kΩ 分压测 12V 电池（分压比 1/11），接到 PA1。采样时间该设多少？如果不设够会发生什么？**

**答**：

第一步，确认电压安全：

```text
分压点电压 = 12 V × 10 / (100 + 10) = 12 / 11 = 1.0909 V
1.0909 V < 3.3 V ✅ 不会损坏引脚
（但注意：如果电池充满是 13V，分压点 = 1.18V，也安全；
  真正危险的是"分压电阻虚焊"这种情况 —— 见坑 3）
```

第二步，算等效源阻抗（从分压点看进去，两个电阻是并联关系）：

```text
R_AIN = 100 kΩ ∥ 10 kΩ = (100 × 10) / (100 + 10) kΩ = 9.0909 kΩ
```

第三步，算需要的采样时间：

```text
t_s ≥ 9.02 × (R_AIN + R_ADC) × C_ADC
    = 9.02 × (9090.9 Ω + 1000 Ω) × 8 pF
    = 9.02 × 10090.9 × 8×10⁻¹²
    = 7.2816×10⁻⁷ s
    = 728.2 ns
```

第四步，换算成可选档位（ADC 时钟 12MHz，1 周期 = 83.333ns）：

```text
需要的周期数 = 728.2 / 83.333 = 8.74 个周期
可选档位：1.5 / 7.5 / 13.5 / 28.5 / 41.5 / 55.5 / 71.5 / 239.5
7.5 周期 = 625.0 ns < 728.2 ns ❌ 不够
13.5 周期 = 1125.0 ns ≥ 728.2 ns ✅
→ 选 13.5 周期（保守一点选 28.5 也没问题，代价只是慢 1.25µs）
```

**如果采样时间不够**（比如留在默认的 1.5 周期 = 125ns）：

```text
RC 时间常数 = 10090.9 Ω × 8 pF = 80.73 ns
t_s / RC = 125 / 80.73 = 1.5484
采样电容只充到 1 − e^(−1.5484) = 1 − 0.2126 = 78.74%

→ 读数偏低 21.3%
   分压点真实 1.0909 V，ADC 会算出 ≈ 0.859 V
   换算成电池电压：以为是 9.45V，其实是 12V —— 差了 2.5V
```

**为什么这个错比"读数不准"更严重**：电池电压监测是**低电量保护**的输入。偏低 21% 意味着：电池实际还有 12.0V（大约剩 50% 电）的时候，程序就认为掉到 9.45V 了，直接触发低压保护停车。**这不是精度问题，是功能问题。** 而且这个偏低量会随温度、随每一块板子的寄生参数变化，标定都标不干净。

（对照第 2.4 节那张表：9.09kΩ 的源阻抗夹在 8.7kΩ（偏低 20.0%）和 10kΩ（偏低 24.2%）中间，算出来 21.3%，对得上。）

**Q4. 三种软件滤波各自对付什么噪声？为什么中值滤波写不出传递函数？**

**答**：

| 滤波 | 对付的噪声 | 传递函数 |
|---|---|---|
| 滑动平均 | 随机噪声（白噪声），N 个点平均噪声降 **√N** 倍（N=8 → 2.83 倍 = 9dB） | `H(z) = (1/N)(1 − z⁻ᴺ)/(1 − z⁻¹)` |
| 一阶低通 | 随机噪声，高频段 **−20dB/十倍频**，延迟**连续可调** | `H(z) = α / (1 − (1−α)z⁻¹)` |
| 中值滤波 | **脉冲噪声**（电机换向、继电器动作的单个尖峰） | **写不出来** |

中值写不出传递函数，是因为**它是非线性运算**：中值不是输入的线性加权组合（输出取决于输入值的大小顺序，而不是它们的加权和），所以"线性时不变系统"那一整套频域分析（包括传递函数）对它不适用。**它只能从统计特性（能干掉几个连续异常点）和延迟上描述。** 这也是为什么工程上不能像分析线性滤波器那样去推中值滤波的截止频率。

顺带一个关键对比：**滑动平均对脉冲噪声是无能为力的**——一个 4000 的尖峰平均进 5 个点里还剩 900，它只能"摊薄"，不能"消除"。这就是中值滤波不可替代的地方。

**Q5. 一阶低通的 α 怎么算？τ = 20ms、采样周期 dt = 5ms 时 α 是多少？截止频率大概多少？**

**答**：

```text
α = dt / (τ + dt) = 5 ms / (20 ms + 5 ms) = 5 / 25 = 0.2

（这就是代码里 LowPass_SetTau() 做的事：
  alpha = dt_s / (tau_s + dt_s)）

截止频率（α 较小时的近似式）：
f_c ≈ 1 / (2π·τ) = 1 / (2π × 0.02 s) = 1 / 0.12566 s = 7.96 Hz
```

推导来源：连续形式是 `τ·dy/dt + y = x`，用后向差分 `dy/dt ≈ (y[n]−y[n−1])/dt` 代入整理得 `y[n] = y[n−1] + α(x[n] − y[n−1])`。

**补充一点更重要的**：这个滤波器的**延迟就是 τ 本身（20ms）**。所以"把 τ 调大"和"延迟变大"是同一件事，调参的时候必须一起看。

**Q6. N = 8 的滑动平均，噪声能降多少？代价是多少延迟？**

**答**：

```text
噪声：N 个独立样本求平均，标准差变成 1/√N
      √8 = 2.828 倍，换算成信噪比改善 = 20·log10(2.828) = 9.03 dB

延迟：群延迟 = (N − 1) / 2 = 3.5 个采样周期
      在我的配置里 dt = 5 ms → 3.5 × 5 ms = 17.5 ms
```

**为什么延迟是 (N−1)/2 而不是 N/2**：滑动平均是"当前点加上之前 N−1 个点"的算术平均，N 个点的中心在 `(N−1)/2` 个采样周期之前。所以对"输入发生阶跃变化"这件事，输出要过 `(N−1)/2` 拍才完全反映出来。

**Q7. ADC 时钟为什么不能超过 14MHz？我配的是多少？**

**答**：F103 手册规定 ADC 时钟上限 **14MHz**。它的来源是 **PCLK2 再分频**（可选 /2 /4 /6 /8），不是直接等于系统时钟。

```text
我的配置：PCLK2 = 72 MHz，分频 = /6
ADC 时钟 = 72 / 6 = 12 MHz ≤ 14 MHz ✅
（如果分频选 /2：72/2 = 36 MHz > 14 MHz ❌）
```

**为什么有这个上限**：逐次逼近的每一次比较，都要经过"内部 DAC 建立 → 比较器判决 → 结果写回"这一串**模拟过程**，这些过程需要时间。时钟给太快，比较器还没判完就被要求进入下一步，结果的低位就是错的。**这是模拟电路的建立时间限制，不是数字逻辑的时序限制**——所以它不能通过"加点等待周期"来绕过，只能降频。

**Q8. 规则组和注入组有什么区别？RM 上的电流采样为什么用注入组？**

**答**：

| | 规则组 | 注入组 |
|---|---|---|
| 通道数 | 最多 16 个 | 最多 **4 个** |
| 结果存哪 | 全部覆盖同一个 `DR` | **4 个独立寄存器 JDR1~JDR4** |
| 取数据 | 需要 DMA 或者及时逐个读 | 转完直接读，不用 DMA |
| 优先级 | 低 | **高，可以打断规则组的转换** |

RM 上电流采样用注入组的三个理由：

1. **必须和 PWM 同步**。相电流在 PWM 周期内是变化的，采样点必须落在纹波最小、和 PWM 严格对齐的时刻（通常用 TIM1 在中心对齐模式的下溢事件触发注入组）。不同步的话每次采到的相位都不一样，采出来的值没有物理意义。
2. **不能被慢速转换挡住**。电流环是全车最快的环，采样延迟直接吃稳定裕度。注入组能抢占正在进行的规则组转换（比如后台慢慢扫的电池电压），保证金电流的采样点准时。
3. **4 个独立结果寄存器刚好够用**：三相电流 Ia/Ib/Ic 加一个母线电压或采样电阻温度，正好 4 个，还不用 DMA。

**本章为什么只用规则组**：因为这一章的重点是"把电压换算和精度这件事弄明白"，用规则组轮询最简单、最容易看懂。注入组这条路我记在这里，等做电流环的时候按这条线走。

**Q9. 校准（calibration）解决了什么误差？它解决不了什么？**

**答**：`HAL_ADCEx_Calibration_Start()` 做的是让芯片自己测一遍内部电容网络和比较器的**固有失调**并记下来，之后每次转换自动减掉。它解决的是**偏移误差（offset error）**，量级在 1~2 个 LSB（约 0.8~1.6mV）。

它解决不了的是**增益误差（gain error）**——也就是第 2.5 节那个问题：我按 VREF = 3.300V 算，但 VDDA 实际是 3.28V，于是**整个量程的所有读数都按 0.606% 的比例偏**。校准不碰这个比例。

治增益误差的两条路：**用外部基准芯片**（把 VREF+ 接到独立的 3.300V 基准），或者**两点标定**（拿万用表量两个已知电压，反推出实际的 LSB 和偏移，把它们做成常数写进代码）。小封装的 Blue Pill 没有引出 VREF+ 引脚，所以只能走标定这条路——或者用内部 VREFINT 反推 VDDA 来消掉"随电源飘"的那部分。





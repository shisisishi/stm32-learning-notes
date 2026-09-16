# 01-blink · 板载 LED + 外接 LED 非阻塞闪烁

**所属章节**：[02 · GPIO 与第一个点灯程序](../../docs/02-GPIO与点灯.md)
**状态**：🔬 代码已写完整，**尚未在真板上烧录运行**（下面所有"现象"都是预期现象）

---

## 1. 用到的引脚

| 引脚 | 配置 | 点亮电平 | 闪烁半周期 | 完整周期 | 限流电阻 |
|---|---|---|---|---|---|
| PC13 | GPIO 推挽输出，2 MHz 档 | **低电平点亮** | 500 ms | 1000 ms（1 Hz） | 板载自带，不加外接电阻 |
| PA5 | GPIO 推挽输出，2 MHz 档 | 高电平点亮 | 100 ms | 200 ms（5 Hz） | 外接 330 Ω（电流约 3.9 mA） |

系统时钟 **72 MHz**（HSE 8 MHz 晶振 × PLL9）。

### 接线

```text
外接 LED（高电平点亮，"吐"电流）：
    PA5 ──── 330 Ω ──── LED 阳极 ├▶| 阴极 ──── GND

板载 LED（低电平点亮，"吸"电流，板子上已经接好）：
    3.3 V ──── 板载电阻 ──── LED 阳极 ├▶| 阴极 ──── PC13
```

> PC13 属于后备域（backup domain），数据手册限制它的输出电流不超过 **3 mA**，
> 所以板载 LED 才设计成阳极接 3.3 V、阴极接 PC13 —— 让引脚"吸"电流，
> 而不是从引脚"吐"电流去驱动 LED。原因见章节第 1、2 节。

---

## 2. CubeMX 需要怎么配

| 位置 | 设置项 | 值 |
|---|---|---|
| Pinout & Configuration → System Core → SYS | Debug | Serial Wire |
| Pinout & Configuration → System Core → RCC | High Speed Clock (HSE) | Crystal/Ceramic Resonator |
| Clock Configuration | HSE → PLL Source | HSE |
| Clock Configuration | PLLMul | ×9 |
| Clock Configuration | System Clock Mux | PLLCLK |
| Clock Configuration | APB1 Prescaler | /2（= 36 MHz） |
| Clock Configuration | APB2 Prescaler | /1（= 72 MHz） |
| 芯片图上点 PC13 | GPIO_Output | Output level: **High**（低电平点亮，所以初始要写高 = 灭） |
| 芯片图上点 PA5 | GPIO_Output | Output level: **Low** |
| 两个引脚各自的 GPIO 参数 | GPIO mode | Output Push Pull |
| 两个引脚各自的 GPIO 参数 | GPIO Pull-up/Pull-down | No pull-up and no pull-down |
| 两个引脚各自的 GPIO 参数 | Maximum output speed | **Low** |
| 两个引脚各自的 GPIO 参数 | User Label | `LED_BUILTIN` / `LED_EXT` |
| Project Manager → Project | Toolchain/IDE | MDK-ARM V5 |

点 **GENERATE CODE** 生成工程，然后在 Keil 里把 `main.c` 的
`/* USER CODE BEGIN 1 */`、`/* USER CODE BEGIN 2 */`、`/* USER CODE BEGIN 3 */`
三处替换成本仓库 `main.c` 里对应的内容（`MX_GPIO_Init()` 由 CubeMX 生成，不用手写）。

---

## 3. 预期现象 🔬

| 观察点 | 预期现象 |
|---|---|
| 板载 LED（PC13，板子上的小灯） | 亮 500 ms、灭 500 ms 交替，1 秒一个完整周期 |
| 外接 LED（PA5） | 亮 100 ms、灭 100 ms 交替，0.2 秒一个完整周期，肉眼看着像"快闪" |
| 上电瞬间 | 两个灯都是**灭**的，等到第一个半周期结束才第一次点亮 |
| 两个灯的关系 | 互不干扰：PA5 闪 5 次的时间里 PC13 只翻转 1 次 |

---

## 4. 如何验证

1. **肉眼 + 秒表**：盯着 PC13 的灯，从一次"亮起"开始数 10 秒，预期数到 10 次亮起；
   PA5 同理数 10 秒，预期约 50 次（肉眼数不准，只能判断量级对不对）。
2. **手机慢动作录像**：120 fps 慢动作拍 PA5，逐帧数"亮/灭"各占几帧，
   按 120 fps 换算，亮的时间应该在 100 ms 量级。
3. **逻辑分析仪 / 示波器**（如果手上有）：测 PA5 波形，
   一个完整方波周期应该是 **200 ms**，占空比接近 **50 %**；
   测 PC13 应该是 **1000 ms**、50 %。这是最硬的证据，比肉眼可靠。
4. **Keil 调试器 Watch 窗口**：把 `tick_pc13`、`tick_pa5`、`now` 加进 Watch，
   全速运行后暂停，三个变量的差值应该分别接近 500 和 100（单位 ms）；
   再单步看 `HAL_GPIO_TogglePin()` 前后 `GPIOC->ODR` 的 bit13 是否翻转。
5. **反例验证（故意改坏，确认自己真的理解了）**：
   - 把 `__HAL_RCC_GPIOC_CLK_ENABLE();` 注释掉 → 预期 PC13 完全不动，PA5 正常。🔬
   - 把 `HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);` 删掉 →
     预期上电瞬间板载 LED 会先亮一下才进入正常闪烁。🔬

---

## 5. 文件说明

| 文件 | 说明 |
|---|---|
| `main.c` | 按 CubeMX 6.x 的 `main.c` 骨架整理，内核逻辑都在 `USER CODE` 区内 |
| `README.md` | 本文件 |

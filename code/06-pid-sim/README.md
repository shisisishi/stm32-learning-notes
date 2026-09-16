# 06 · PID 整定仿真

脱离硬件，在 PC 上把 PID 三个参数的作用跑明白。
对应章节：[`docs/09-PID控制.md`](../../docs/09-PID控制.md)

## 这个目录里有什么

| 文件 | 作用 | 依赖 |
|---|---|---|
| `pid.h` / `pid.c` | PID 控制器本体（位置式 + 输出限幅 + 积分限幅 + 微分先行 + 微分低通） | **无**，可直接搬进 STM32 工程 |
| `plant.h` / `plant.c` | 有刷直流电机的电气+机械双状态模型 | 仅 `math.h` |
| `sim.c` | 仿真主程序：18 组场景、指标统计、CSV 输出 | 仅标准库 |
| `plot.py` | 读 CSV 画成 SVG | **仅 Python 标准库**，无需 matplotlib |
| `data/` | 仿真输出的波形与指标（已提交，就是文中的证据） | — |

## 怎么跑

### Windows (PowerShell)

```powershell
.\build.ps1          # 编译 + 运行 + 画图，一步到位
```

### Linux / macOS

```bash
bash build.sh
```

### 手动

```bash
gcc -O2 -std=c99 -Wall -Wextra -o pidsim sim.c pid.c plant.c -lm
./pidsim
python plot.py          # 生成 SVG 到 ../../assets/
```

`sim.c` 和 `pid.c` 是**严格 C99、零警告**编译的（`-Wall -Wextra`）。

## 仿真配置

| 项目 | 值 | 说明 |
|---|---|---|
| 控制周期 | 1 ms (1 kHz) | RM 上常见的控制频率 |
| 物理积分步长 | 20 µs | 必须远小于电气时间常数 τe = L/R = 0.5 ms |
| 仿真时长 | 0.5 s | 500 个控制周期 |
| 目标转速 | 200 rad/s | 约 1910 rpm |
| 电源限幅 | ±12 V | |
| 电机 | R=1Ω, L=0.5mH, Ke=Kt=0.0318, J=5.5e-5, B=1e-6 | 12V 带减速箱小型直流电机 |
| 派生常数 | τe = 0.50 ms, τm = 54.3 ms | 程序会自动打印自检 |

## 输出指标说明

| 指标 | 含义 |
|---|---|
| 上升时间 | 10% → 90% 的时间 |
| 超调量 | (峰值 − 目标) / 目标 × 100% |
| 稳态误差 | 最后 10% 时间内的平均偏差 |
| 调节时间 | 最后一次超出 ±2% 带之后的时间 |
| 输出纹波 | 稳态输出的峰峰值（看振荡） |
| 输出抖动 | 相邻采样输出变化量的平均绝对值（看执行器有多"躁"） |

## 想自己改实验

`sim.c` 里的 `CASES[]` 表，一行一个场景：

```c
{ "场景名", kp, ki, kd, d_tau, i_max, use_ilimit, quant, noise },
```

改完重新 `build.ps1` / `build.sh` 即可。`plot.py` 里的 `fig_*` 函数决定画哪几组。

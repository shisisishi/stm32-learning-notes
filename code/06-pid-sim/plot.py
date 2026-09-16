#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot.py —— 把 sim.c 跑出来的 CSV 画成 SVG 曲线图

为什么自己手写 SVG，而不用 matplotlib？
  1. 不用装任何第三方库，python plot.py 直接就能跑 —— 复现门槛最低；
  2. SVG 是纯文本，能直接塞进 GitHub 仓库里显示，还能 diff；
  3. 顺带把 SVG 的坐标变换搞明白，以后画上位机波形也有用。

用法（在 code/06-pid-sim 目录下）：
    python plot.py            # 读取 data/*.csv，输出到 ../../assets/*.svg
"""

import csv
import os
import sys

# Windows 的 Python 默认拿 ANSI 代码页（简体中文机器是 cp936、英文机器是 cp1252）
# 去编码 print 的内容，一打中文就抛 UnicodeEncodeError。
# 这里强制把标准输出切成 UTF-8 —— 这是个只有 Windows 上才会踩到的坑，
# 我第一版就是这么崩的，写在这里省得你也踩一遍。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ---------------------------------------------------------------- 数据读取

def read_csv(path):
    """读一个波形 CSV，返回 dict：{'t': [...], 'omega': [...], 'u': [...], 'setpoint': [...]}"""
    cols = {}
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for k in r.fieldnames:
            cols[k] = []
        for row in r:
            for k in r.fieldnames:
                cols[k].append(float(row[k]))
    return cols


def load(name):
    p = os.path.join("data", name + ".csv")
    if not os.path.exists(p):
        print(f"  ! 缺少 {p}，先运行 pidsim 生成数据")
        return None
    return read_csv(p)


# ---------------------------------------------------------------- SVG 绘图

PALETTE = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]
FONT = "Microsoft YaHei, PingFang SC, Hiragino Sans GB, Noto Sans CJK SC, Arial, sans-serif"


def nice_range(vmin, vmax, nticks=5):
    """把 [vmin,vmax] 扩到好看的范围，并返回合适的刻度步长"""
    if vmax <= vmin:
        vmax = vmin + 1.0
    span = vmax - vmin
    raw = span / max(1, nticks - 1)
    mag = 10.0 ** int(_floor_log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    lo = _floor(vmin / step) * step
    hi = _ceil(vmax / step) * step
    return lo, hi, step


def _floor_log10(x):
    import math
    return math.floor(math.log10(x)) if x > 0 else 0


def _floor(x):
    import math
    return math.floor(x)


def _ceil(x):
    import math
    return math.ceil(x)


def fmt_tick(v, step):
    if abs(v) < 1e-12:
        return "0"
    if step >= 1:
        return f"{v:.0f}"
    if step >= 0.1:
        return f"{v:.1f}"
    if step >= 0.01:
        return f"{v:.2f}"
    return f"{v:.3f}"


class Panel:
    """一个子图：带自己的标题、轴标签、曲线列表"""

    def __init__(self, title, ylabel, height=300, ymin=None, ymax=None,
                 xlabel="时间 (ms)", legend_pos="top-right"):
        self.title = title
        self.ylabel = ylabel
        self.xlabel = xlabel
        self.height = height
        self.series = []          # (label, xs, ys, color)
        self.ymin = ymin
        self.ymax = ymax
        self.legend_pos = legend_pos

    def add(self, label, xs, ys, color=None):
        if color is None:
            color = PALETTE[len(self.series) % len(PALETTE)]
        self.series.append((label, xs, ys, color))
        return self

    def data_bounds(self):
        lo, hi = 1e300, -1e300
        for _, _, ys, _ in self.series:
            for v in ys:
                if v < lo:
                    lo = v
                if v > hi:
                    hi = v
        return lo, hi


def render(panels, out_path, width=940):
    """把若干 Panel 竖着拼成一张 SVG"""
    ml, mr, mt, mb = 66, 210, 34, 40     # 左、右、上、下边距（右侧留给图例）
    parts = []
    y_cursor = 8
    total_h = 8

    for p in panels:
        total_h += p.height + 46
    total_h += 8

    for p in panels:
        # ---- 计算 y 轴范围 ----
        dlo, dhi = p.data_bounds()
        pad = (dhi - dlo) * 0.08 if dhi > dlo else 1.0
        lo, hi, _ = nice_range(dlo - pad, dhi + pad, 6)
        if p.ymin is not None:
            lo = p.ymin
        if p.ymax is not None:
            hi = p.ymax
        # 强制过范围之后必须重算步长，否则刻度会落在 -13 / -8 / -3 这种丑位置上
        _, _, step = nice_range(lo, hi, 6)

        # ---- x 轴：统一用第一组数据的 t ----
        xs_ref = p.series[0][1] if p.series else [0.0, 1.0]
        xlo, xhi = min(xs_ref), max(xs_ref)
        if xhi <= xlo:
            xhi = xlo + 1.0

        px0, px1 = ml, width - mr
        py0, py1 = y_cursor + 30, y_cursor + 30 + p.height

        def sx(x):
            return px0 + (x - xlo) / (xhi - xlo) * (px1 - px0)

        def sy(y):
            return py1 - (y - lo) / (hi - lo) * (py1 - py0)

        # ---- 标题 ----
        parts.append(
            f'<text x="{px0}" y="{y_cursor + 18}" font-size="15" font-weight="600" '
            f'fill="#111">{_esc(p.title)}</text>'
        )

        # ---- 横向网格 + y 刻度 ----
        v = lo
        while v <= hi + step * 0.001:
            y = sy(v)
            parts.append(
                f'<line x1="{px0:.1f}" y1="{y:.1f}" x2="{px1:.1f}" y2="{y:.1f}" '
                f'stroke="#e8e8e8" stroke-width="1"/>'
            )
            parts.append(
                f'<text x="{px0 - 8}" y="{y + 4:.1f}" font-size="11" fill="#666" '
                f'text-anchor="end">{fmt_tick(v, step)}</text>'
            )
            v += step

        # ---- 纵向网格 + x 刻度 ----
        n_x = 5
        for i in range(n_x + 1):
            xv = xlo + (xhi - xlo) * i / n_x
            x = sx(xv)
            parts.append(
                f'<line x1="{x:.1f}" y1="{py0}" x2="{x:.1f}" y2="{py1}" '
                f'stroke="#f2f2f2" stroke-width="1"/>'
            )
            parts.append(
                f'<text x="{x:.1f}" y="{py1 + 18}" font-size="11" fill="#666" '
                f'text-anchor="middle">{xv:.0f}</text>'
            )

        # ---- 坐标轴 ----
        parts.append(
            f'<rect x="{px0}" y="{py0}" width="{px1 - px0}" height="{py1 - py0}" '
            f'fill="none" stroke="#999" stroke-width="1"/>'
        )

        # ---- 曲线 ----
        for label, xs, ys, color in p.series:
            d = []
            for i in range(len(xs)):
                cmd = "M" if i == 0 else "L"
                d.append(f"{cmd}{sx(xs[i]):.2f},{sy(ys[i]):.2f}")
            parts.append(
                f'<path d="{" ".join(d)}" fill="none" stroke="{color}" '
                f'stroke-width="1.8" stroke-linejoin="round"/>'
            )

        # ---- 图例 ----
        lx = px1 + 14
        ly = py0 + 8
        for label, _, _, color in p.series:
            parts.append(
                f'<line x1="{lx}" y1="{ly - 4}" x2="{lx + 18}" y2="{ly - 4}" '
                f'stroke="{color}" stroke-width="2.4"/>'
            )
            parts.append(
                f'<text x="{lx + 24}" y="{ly}" font-size="11.5" fill="#333">{_esc(label)}</text>'
            )
            ly += 19

        # ---- 轴标签 ----
        parts.append(
            f'<text x="{(px0 + px1) / 2:.0f}" y="{py1 + 35}" font-size="11.5" '
            f'fill="#555" text-anchor="middle">{_esc(p.xlabel)}</text>'
        )
        parts.append(
            f'<text x="16" y="{(py0 + py1) / 2:.0f}" font-size="11.5" fill="#555" '
            f'text-anchor="middle" transform="rotate(-90 16 {(py0 + py1) / 2:.0f})">'
            f'{_esc(p.ylabel)}</text>'
        )

        y_cursor = py1 + 46

    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{total_h}" '
        f'viewBox="0 0 {width} {total_h}" font-family="{FONT}">\n'
        f'<rect width="{width}" height="{total_h}" fill="#ffffff"/>\n'
        + "\n".join(parts)
        + "\n</svg>\n"
    )

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"  写出 {out_path}  ({len(svg) / 1024:.1f} KB)")


def _esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def downsample(xs, ys, maxpts=400):
    """点太多时抽稀，SVG 体积可控、形状不失真"""
    n = len(xs)
    if n <= maxpts:
        return xs, ys
    stride = max(1, n // maxpts)
    return xs[::stride], ys[::stride]


def ms(xs):
    return [x * 1000.0 for x in xs]


# ---------------------------------------------------------------- 各张图

SETPOINT = 200.0


def fig_p(paths_out):
    cases = [("P_kp0.3", "Kp = 0.3"), ("P_kp1", "Kp = 1"),
             ("P_kp3", "Kp = 3"), ("P_kp8", "Kp = 8（开始振）")]
    p = Panel("实验一：只有比例项 P —— 目标 200 rad/s",
              "转速 ω (rad/s)", ymin=0, ymax=260)
    p.add("设定值 200", [0, 0.5], [SETPOINT, SETPOINT], "#bbbbbb")
    for name, label in cases:
        d = load(name)
        if d:
            xs, ys = downsample(d["t"], d["omega"])
            p.add(label, ms(xs), ys)
    render([p], os.path.join(paths_out, "pid_p_kp.svg"))


def fig_i(paths_out):
    cases = [("PI_ki0", "Ki = 0（纯 P）"), ("PI_ki2", "Ki = 2"),
             ("PI_ki10", "Ki = 10"), ("PI_ki50", "Ki = 50")]
    p = Panel("实验二：固定 Kp = 1，加积分项 I", "转速 ω (rad/s)", ymin=0, ymax=260)
    p.add("设定值 200", [0, 0.5], [SETPOINT, SETPOINT], "#bbbbbb")
    for name, label in cases:
        d = load(name)
        if d:
            xs, ys = downsample(d["t"], d["omega"])
            p.add(label, ms(xs), ys)
    render([p], os.path.join(paths_out, "pid_i_ki.svg"))


def fig_d(paths_out):
    cases = [("PID_kd0", "Kd = 0"), ("PID_kd0005", "Kd = 0.0005"),
             ("PID_kd001", "Kd = 0.001"), ("PID_kd002", "Kd = 0.002")]
    p = Panel("实验三：固定 Kp = 1, Ki = 10，加微分项 D",
              "转速 ω (rad/s)", ymin=180, ymax=215)
    p.add("设定值 200", [0, 0.5], [SETPOINT, SETPOINT], "#bbbbbb")
    for name, label in cases:
        d = load(name)
        if d:
            xs, ys = downsample(d["t"], d["omega"])
            p.add(label, ms(xs), ys)
    render([p], os.path.join(paths_out, "pid_d_kd.svg"))


def fig_windup(paths_out):
    p1 = Panel("实验四：积分饱和 —— Ki 都等于 50，只差一个积分限幅",
               "转速 ω (rad/s)", height=270, ymin=0, ymax=320)
    p2 = Panel("同一次实验的控制输出（被电源 ±12V 限住）",
               "输出电压 u (V)", height=240, ymin=-13, ymax=13)
    for name, label in [("windup_off", "无积分限幅 · 超调 44%"),
                        ("windup_on", "有积分限幅 · 超调 1.1%")]:
        d = load(name)
        if d:
            xs, ys = downsample(d["t"], d["omega"])
            p1.add(label, ms(xs), ys)
            xs2, ys2 = downsample(d["t"], d["u"])
            p2.add(label, ms(xs2), ys2)
    p1.add("设定值 200", [0, 0.5], [SETPOINT, SETPOINT], "#bbbbbb")
    render([p1, p2], os.path.join(paths_out, "pid_windup.svg"))


def fig_noise(paths_out):
    p1 = Panel("实验五：测量噪声（±2 rad/s）下的转速",
               "转速 ω (rad/s)", height=250, ymin=185, ymax=215)
    p2 = Panel("同一实验的控制输出 —— 抖动在这里看得最清楚",
               "输出电压 u (V)", height=250, ymin=0, ymax=14)
    for name, label in [("noise_nod", "不加 D · 抖动 1.69 V"),
                        ("noise_d", "加 D 不滤波 · 抖动 4.73 V"),
                        ("noise_d_f10", "加 D + 10ms 低通 · 抖动 1.88 V")]:
        d = load(name)
        if d:
            xs, ys = downsample(d["t"], d["omega"])
            p1.add(label, ms(xs), ys)
            xs2, ys2 = downsample(d["t"], d["u"])
            p2.add(label, ms(xs2), ys2)
    p1.add("设定值 200", [0, 0.5], [SETPOINT, SETPOINT], "#bbbbbb")
    render([p1, p2], os.path.join(paths_out, "pid_noise.svg"))


def main():
    if not os.path.isdir("data"):
        print("请在 code/06-pid-sim 目录下运行本脚本")
        return 1
    out = os.path.join("..", "..", "assets")
    print("生成曲线图：")
    fig_p(out)
    fig_i(out)
    fig_d(out)
    fig_windup(out)
    fig_noise(out)
    print("完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

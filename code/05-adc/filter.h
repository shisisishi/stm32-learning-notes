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

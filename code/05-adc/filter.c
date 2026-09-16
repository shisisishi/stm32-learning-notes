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

    med = Median_Update(&c->med, raw);          /* 第一级：抠掉脉冲尖峰 */
    avg = MovAvg_Update(&c->avg, (uint32_t)med); /* 第二级：压随机噪声 */
    return LowPass_Update(&c->lp, (float)avg);   /* 第三级：一阶低通 */
}

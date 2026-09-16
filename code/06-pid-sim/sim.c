/**
 * sim.c —— PID 整定仿真：脱离硬件，在 PC 上把参数影响看清楚
 *
 * 为什么先做仿真而不是直接上板子？
 *   1. 上板子一次只能试一组参数，改一次要重编译下载，一组 30 秒，
 *      试 16 组就是 8 分钟；仿真 16 组只要 0.1 秒。
 *   2. 仿真里被控对象是固定的，于是 Kp 变化引起的差异一定来自 Kp，
 *      而不是"这次电池电压低了"。这叫控制变量法，是做实验的基本功。
 *   3. 参数扫出大致范围之后再上板子微调，效率高得多。
 *
 * 编译运行：
 *     gcc -O2 -std=c99 -Wall -Wextra -o pidsim sim.c pid.c plant.c -lm
 *     ./pidsim            （在 06-pid-sim 目录下运行，需要 data/ 目录存在）
 *
 * 输出：
 *     data/<场景名>.csv     每次控制周期的 t / setpoint / omega / u
 *     data/metrics.csv      16 组场景的性能指标汇总
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pid.h"
#include "plant.h"

/* ---------------- 仿真配置 ---------------- */
#define PLANT_DT    2e-5f    /* 物理积分步长 20µs，必须 << L/R = 0.5ms */
#define CTRL_DT     1e-3f    /* 控制周期 1ms，对应 1kHz 控制频率（RM 常见值） */
#define PLANT_SUB   50       /* 每个控制周期做多少次物理积分 = 1ms / 20µs */
#define T_END       0.5f     /* 仿真时长 0.5s */
/* 步数 = T_END / CTRL_DT = 0.5 / 0.001 = 500。
 * 必须写成整数字面量：C99 里 (int)(浮点表达式) 不是「整数常量表达式」，
 * 当数组维度会报 "storage size isn't constant"。这个坑是真编译的时候才发现的。 */
#define N_STEPS     500

#define SETPOINT    200.0f   /* 目标转速 rad/s（约 1910 rpm） */
#define U_LIMIT     12.0f    /* 电源电压限幅 ±12V */

/* ---------------- 电机参数 ----------------
 * 对应一台 12V 带减速箱的小型直流电机（输出轴侧折算）：
 *   R  = 1Ω          电枢电阻
 *   L  = 0.5mH       电枢电感
 *   Ke = Kt ≈ 0.0318 V·s/rad   （空载约 300 rpm/V）
 *   J  = 5.5e-5 kg·m²         折算转动惯量
 *   B  = 1e-6 N·m·s/rad       粘性摩擦
 *
 * 由此得到两个关键时间常数（这是整定 PID 前必须算的东西）：
 *   电气时间常数 τe = L / R              = 0.5 ms
 *   机械时间常数 τm = J·R / (Kt·Ke)      = 54  ms
 *
 * τm 比 τe 大 100 倍 —— 这就是"转速环可以当成一阶惯性环节来调"的根据。
 * 如果偷懒把物理步长取成 1ms，dt/τe = 2 正好踩在欧拉法稳定边界上，仿真会直接炸。
 */
#define MOTOR_R     1.0f
#define MOTOR_L     0.0005f
#define MOTOR_J     5.5e-5f
#define MOTOR_B     1e-6f
#define MOTOR_RPMV  300.0f

typedef struct {
    const char *name;
    float kp, ki, kd;
    float d_tau;        /* 微分低通滤波时间常数 [s]，0 = 不滤波 */
    float i_max;        /* 积分限幅 */
    int   use_ilimit;   /* 0 = 关掉积分限幅，用来演示积分饱和 */
    float quant;        /* 测量量化步长 rad/s，0 = 理想编码器 */
    float noise;        /* 测量噪声幅度 rad/s，0 = 无噪声 */
} Case_t;

/* ---------------- 实验设计 ----------------
 * 规则：每一组只改一个变量，其余保持和对照组完全一致。 */
static const Case_t CASES[] = {
    /* 实验一：只有比例项 P —— 看 Kp 对上升速度和稳态误差的影响，
     *         以及 Kp 太大为什么会振 */
    { "P_kp0.3",      0.3f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "P_kp1",        1.0f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "P_kp3",        3.0f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "P_kp8",        8.0f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },

    /* 实验二：固定 Kp=1，加积分项 I —— 看稳态误差怎么被消掉、超调怎么来的 */
    { "PI_ki0",       1.0f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PI_ki2",       1.0f,  2.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PI_ki10",      1.0f, 10.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PI_ki50",      1.0f, 50.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 0.0f },

    /* 实验三：固定 Kp=1、Ki=10（这组本身超调就有 3%，才看得出 D 的作用），
     *         加微分项 D —— 看超调怎么被压下去。
     * 注意 Kd 的量纲是"秒"（因为微分项要除以 dt），
     * 所以数值很小是正常的，不是"参数没调大"，量纲决定它就该是这个数量级。 */
    { "PID_kd0",      1.0f, 10.0f, 0.0000f,  0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PID_kd0005",   1.0f, 10.0f, 0.0005f,  0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PID_kd001",    1.0f, 10.0f, 0.0010f,  0.0f,   12.0f, 1, 0.0f, 0.0f },
    { "PID_kd002",    1.0f, 10.0f, 0.0020f,  0.0f,   12.0f, 1, 0.0f, 0.0f },

    /* 实验四：抗积分饱和。ki 给很大 + 输出被 ±12V 顶住，
     *         对比"关掉积分限幅"和"打开积分限幅" */
    { "windup_off",   1.0f, 50.0f, 0.0f,     0.0f,    1e9f, 0, 0.0f, 0.0f },
    { "windup_on",    1.0f, 50.0f, 0.0f,     0.0f,    8.0f, 1, 0.0f, 0.0f },

    /* 实验五：测量噪声下标量微分的灾难，以及低通滤波能救回多少
     * —— 这一组是我被数据教育之后补的实验。
     * 基准特意取 ki=0：这样纹波里就没有"积分项对噪声随机游走"的成分，
     * 剩下的差异才是微分项一个人的锅。 */
    { "noise_nod",    1.0f,  0.0f, 0.0f,     0.0f,   12.0f, 1, 0.0f, 2.0f },
    { "noise_d",      1.0f,  0.0f, 0.0010f,  0.0f,   12.0f, 1, 0.0f, 2.0f },
    { "noise_d_f2",   1.0f,  0.0f, 0.0010f,  0.002f, 12.0f, 1, 0.0f, 2.0f },
    { "noise_d_f10",  1.0f,  0.0f, 0.0010f,  0.010f, 12.0f, 1, 0.0f, 2.0f },
};
#define N_CASES ((int)(sizeof(CASES) / sizeof(CASES[0])))

/* 简单可复现的伪随机数（不用 rand()，保证每次跑结果完全一样，方便对比） */
static unsigned int g_seed = 12345u;
static float frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return (float)((g_seed >> 16) & 0x7FFF) / 32767.0f - 0.5f;   /* [-0.5, 0.5) */
}

typedef struct {
    float rise_time;     /* 10% → 90% 上升时间 (s)，达不到则 -1 */
    float overshoot;     /* 超调量 (%) */
    float sse;           /* 稳态误差 (rad/s)，取最后 10% 时间的平均偏差 */
    float settle_time;   /* 调节时间 (s)，进入并保持在 ±2% 带内，达不到则 -1 */
    float u_max;         /* 输出峰值 (V) */
    float ripple;        /* 稳态输出波动峰峰值 (V)，看抖动 */
    float jitter;        /* 输出抖动量 (V/次)，相邻采样输出变化量的平均绝对值。
                          * 比峰峰值更贴切：它直接反映"执行器每毫秒要变多少"，
                          * 也就是电机听到的电流噪声有多大。 */
} Metrics_t;

static void Analyze(const float *t, const float *y, const float *u, int n,
                    float sp, Metrics_t *m)
{
    int i;
    float ymax = -1e30f;
    float band = 0.02f * sp;
    float tol10 = 0.10f * sp, tol90 = 0.90f * sp;
    int t10 = -1, t90 = -1, last_out = -1;
    float sum = 0.0f; int cnt = 0;
    float umin_s = 1e30f, umax_s = -1e30f;

    for (i = 0; i < n; i++) {
        if (y[i] > ymax) ymax = y[i];
        if (t10 < 0 && y[i] >= tol10) t10 = i;
        if (t90 < 0 && y[i] >= tol90) t90 = i;
        if (fabsf(y[i] - sp) > band) last_out = i;
    }

    m->rise_time   = (t10 >= 0 && t90 >= 0) ? (t[t90] - t[t10]) : -1.0f;
    m->overshoot   = (ymax > sp) ? (ymax - sp) / sp * 100.0f : 0.0f;
    m->settle_time = (last_out >= 0 && last_out < n - 1) ? t[last_out + 1] : -1.0f;

    /* 稳态误差、输出波动、输出抖动：都取最后 10% 时间 */
    {
        float jsum = 0.0f; int jcnt = 0;
        for (i = n - n / 10; i < n; i++) {
            sum += (sp - y[i]);
            cnt++;
            if (u[i] < umin_s) umin_s = u[i];
            if (u[i] > umax_s) umax_s = u[i];
            if (i > n - n / 10) { jsum += fabsf(u[i] - u[i - 1]); jcnt++; }
        }
        m->jitter = (jcnt > 0) ? jsum / (float)jcnt : 0.0f;
    }
    m->sse    = (cnt > 0) ? sum / (float)cnt : 0.0f;
    m->ripple = umax_s - umin_s;

    m->u_max = 0.0f;
    for (i = 0; i < n; i++) if (fabsf(u[i]) > m->u_max) m->u_max = fabsf(u[i]);
}

int main(void)
{
    static float t[N_STEPS], ys[N_STEPS], us[N_STEPS];
    FILE *fm;
    int c, k;

    fm = fopen("data/metrics.csv", "w");
    if (!fm) { fprintf(stderr, "打不开 data/metrics.csv，请先建好 data 目录\n"); return 1; }
    fprintf(fm, "case,kp,ki,kd,d_tau,rise_time_s,overshoot_pct,sse_rad_s,settle_time_s,u_peak_V,ripple_V,jitter_V\n");

    printf("%-15s %5s %5s %8s | %9s %8s %10s %10s %9s %10s\n",
           "场景", "Kp", "Ki", "Kd", "上升时间", "超调%", "稳态误差", "调节时间", "输出纹波", "输出抖动");
    printf("---------------------------------------------------------------------------------------------------\n");

    for (c = 0; c < N_CASES; c++) {
        const Case_t *cs = &CASES[c];
        Motor_t motor;
        PID_t   pid;
        Metrics_t m;
        char path[256];
        FILE *fp;
        int step;
        float tnow = 0.0f;
        char st[32];

        Motor_InitFromRpmPerVolt(&motor, MOTOR_RPMV, MOTOR_R, MOTOR_L, MOTOR_J, MOTOR_B);
        PID_Init(&pid, cs->kp, cs->ki, cs->kd);
        PID_SetOutputLimit(&pid, -U_LIMIT, U_LIMIT);
        PID_SetIntegralLimit(&pid, cs->use_ilimit ? cs->i_max : 1e9f);
        if (cs->d_tau > 0.0f) PID_SetDerivativeFilter(&pid, cs->d_tau);
        PID_SetSetpoint(&pid, SETPOINT);

        g_seed = 12345u;   /* 每个场景用同一串随机数，保证可比 */

        for (step = 0; step < N_STEPS; step++) {
            int sub;
            float meas;

            /* --- 测量：真实转速 + 量化 + 噪声 --- */
            meas = motor.omega;
            if (cs->quant > 0.0f) meas = floorf(meas / cs->quant + 0.5f) * cs->quant;
            if (cs->noise > 0.0f) meas += frand() * 2.0f * cs->noise;

            /* --- 控制器：每个控制周期算一次 --- */
            motor.u = PID_Update(&pid, meas, CTRL_DT);

            /* --- 被控对象：控制周期内做 50 次物理积分 --- */
            for (sub = 0; sub < PLANT_SUB; sub++) Motor_Step(&motor, PLANT_DT);

            t[step]  = tnow;
            ys[step] = motor.omega;
            us[step] = motor.u;
            tnow += CTRL_DT;
        }

        Analyze(t, ys, us, N_STEPS, SETPOINT, &m);

        snprintf(path, sizeof(path), "data/%s.csv", cs->name);
        fp = fopen(path, "w");
        if (!fp) { fprintf(stderr, "打不开 %s\n", path); fclose(fm); return 1; }
        fprintf(fp, "t,setpoint,omega,u\n");
        for (k = 0; k < N_STEPS; k++)
            fprintf(fp, "%.4f,%.4f,%.4f,%.4f\n", t[k], SETPOINT, ys[k], us[k]);
        fclose(fp);

        if (m.settle_time >= 0.0f) snprintf(st, sizeof(st), "%8.1fms", m.settle_time * 1000.0f);
        else                       snprintf(st, sizeof(st), "%10s", "未进带");

        printf("%-15s %5.2f %5.1f %8.4f | %7.1fms %7.1f%% %9.2f %10s %8.2fV %8.3fV\n",
               cs->name, cs->kp, cs->ki, cs->kd,
               m.rise_time * 1000.0f, m.overshoot, m.sse, st, m.ripple, m.jitter);

        fprintf(fm, "%s,%.4f,%.4f,%.6f,%.5f,%.5f,%.3f,%.4f,%.5f,%.3f,%.4f,%.4f\n",
                cs->name, cs->kp, cs->ki, cs->kd, cs->d_tau,
                m.rise_time, m.overshoot, m.sse, m.settle_time, m.u_max, m.ripple, m.jitter);
    }

    fclose(fm);

    /* 模型自检：理论稳态转速应当和仿真的终值对得上，对不上说明模型写错了 */
    {
        Motor_t tmp;
        float w24;
        Motor_InitFromRpmPerVolt(&tmp, MOTOR_RPMV, MOTOR_R, MOTOR_L, MOTOR_J, MOTOR_B);
        w24 = Motor_SteadyStateOmega(&tmp, U_LIMIT);
        printf("\n[模型自检] %gV 空载理论稳态转速 = %.1f rad/s (%.0f rpm)\n",
               (double)U_LIMIT, w24, w24 * 60.0f / 6.2831853f);
        printf("[模型自检] 电气时间常数 τe = %.2f ms，机械时间常数 τm = %.1f ms\n",
               MOTOR_L / MOTOR_R * 1000.0f,
               MOTOR_J * MOTOR_R / (0.031831f * 0.031831f) * 1000.0f);
    }

    printf("\n已写出 data/metrics.csv 与 %d 个波形文件。\n", N_CASES);
    return 0;
}

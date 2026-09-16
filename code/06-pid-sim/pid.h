/**
 * pid.h —— 位置式 PID 控制器（与硬件无关的纯 C 实现）
 *
 * 设计目标：
 *   1. 不依赖任何 STM32 / HAL 头文件，PC 上也能编译测试；
 *   2. 同一份代码既能跑在 PC 仿真里，也能直接丢进 STM32 工程用；
 *   3. 带工程上真正需要的四个东西：输出限幅、积分限幅、微分先行、微分低通滤波。
 *
 * 用 float 而非 double：STM32F1 是 Cortex-M3，没有硬件双精度 FPU，
 * 用 float 由软件浮点库处理，比 double 少一半运算量。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ---- 需要用户配置的参数 ---- */
    float kp;             /* 比例系数     [V/(rad/s)] */
    float ki;             /* 积分系数     [V/(rad/s)/s] = [V/rad] */
    float kd;             /* 微分系数     [V·s/(rad/s)] = [V·s²/rad] —— 量纲是"秒" */
    float d_tau;          /* 微分低通滤波时间常数 [s]，0 表示不滤波 */
    float out_min;        /* 输出下限，例如 -12.0f */
    float out_max;        /* 输出上限，例如  12.0f */
    float i_max;          /* 积分项单独限幅，抗积分饱和的关键 */

    /* ---- 运行期状态 ---- */
    float setpoint;       /* 目标值 */
    float integral;       /* 积分累加器 */
    float prev_measure;   /* 上一次的测量值（微分先行用） */
    float d_filtered;     /* 滤波后的微分项 */
    float out;            /* 最近一次输出 */
    int   first_run;      /* 首次运行标志：第一次不做微分 */
} PID_t;

/* 初始化：把参数和状态都置成确定值，避免上电跑飞 */
void  PID_Init(PID_t *pid, float kp, float ki, float kd);

/* 设置输出限幅（对应执行器的物理能力，例如电源 ±12V） */
void  PID_SetOutputLimit(PID_t *pid, float min, float max);

/* 设置积分限幅。i_max 一般取 (out_max 的量级) 再收一收 */
void  PID_SetIntegralLimit(PID_t *pid, float i_max);

/**
 * 打开微分项的一阶低通滤波。
 *   tau —— 滤波时间常数，单位秒。tau 越大越平滑、但相位滞后越多。
 *          经验取法：tau 取采样周期的 3~10 倍。
 * 物理含义：把 "理想的纯微分" 换成 "带惯性的微分"，
 *          这样对测量噪声的高频增益不再是无穷大。
 */
void  PID_SetDerivativeFilter(PID_t *pid, float tau);

/* 修改目标值。注意：不在这里清积分，否则每次改目标都会丢失积分作用 */
void  PID_SetSetpoint(PID_t *pid, float setpoint);

/* 清空积分与历史，用于重新启动闭环 */
void  PID_Reset(PID_t *pid);

/**
 * 执行一次 PID 计算。
 *   measurement —— 本次采样到的被控量（例如编码器测到的转速）
 *   dt          —— 距上次调用的时间间隔，单位秒
 * 返回本次的控制输出（已被限幅到 [out_min, out_max]）。
 */
float PID_Update(PID_t *pid, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */

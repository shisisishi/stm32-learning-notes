/**
 * pid.c —— 位置式 PID 实现
 *
 * 四个"教科书上不写、工程上必须加"的东西，这里全加了：
 *
 *   1) 输出限幅      —— 执行器有物理上限（PWM 占空比 0~100%、电源 ±12V）。
 *                       不限幅的话积分项会继续乱涨，回到范围内要很久才恢复。
 *
 *   2) 积分限幅      —— 抗积分饱和（anti-windup）最常用也最省事的一招。
 *                       原理：一旦输出已经顶到上限、误差却还在同方向，
 *                       积分项就会越滚越大，这叫"饱和"；等误差反向时，
 *                       必须先把这一大坨积分"还清"才能真正反向，
 *                       表现就是超调巨大、恢复极慢。
 *                       本仓库的 sim.c 里有 windup_off / windup_on 两组对比数据。
 *
 *   3) 微分先行      —— 不对误差 e 微分，而是对测量值 y 微分（再加负号）。
 *                       因为设定值突变时 e 会跳变，de/dt 理论上无穷大，
 *                       实际表现是输出瞬间打满、电机"咔"一下。
 *                       对 y 微分就不会有这个冲击（y 是连续变化的物理量）。
 *
 *   4) 微分低通滤波  —— 这一条是本仿真里"被数据打脸"才加上的。
 *                       一开始我没加滤波，结果 Kd 稍微给大一点，
 *                       输出就在 ±12V 之间疯狂跳（见 noise_d 场景）。
 *                       原因：微分项 = Kd·Δy/dt，dt 只有 1ms，
 *                       等于把测量噪声放大了 1000 倍再乘以 Kd。
 *                       解法：串一个一阶惯性环节把高频压下去。
 */
#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->d_tau = 0.0f;         /* 默认不开滤波 */

    pid->out_min = -1.0f;      /* 默认限幅 ±1，用之前一定要改成实际范围 */
    pid->out_max =  1.0f;
    pid->i_max   =  1.0f;

    PID_Reset(pid);
}

void PID_SetOutputLimit(PID_t *pid, float min, float max)
{
    pid->out_min = min;
    pid->out_max = max;
}

void PID_SetIntegralLimit(PID_t *pid, float i_max)
{
    pid->i_max = (i_max < 0.0f) ? -i_max : i_max;   /* 取绝对值，防调用方传负数 */
}

void PID_SetDerivativeFilter(PID_t *pid, float tau)
{
    pid->d_tau = (tau < 0.0f) ? 0.0f : tau;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Reset(PID_t *pid)
{
    pid->setpoint     = 0.0f;
    pid->integral     = 0.0f;
    pid->prev_measure = 0.0f;
    pid->d_filtered   = 0.0f;
    pid->out          = 0.0f;
    pid->first_run    = 1;
}

float PID_Update(PID_t *pid, float measurement, float dt)
{
    float error, p, d_raw, d, out;

    /* 防御：dt 为 0 或负数会让除法和积分出错；上限防采样间隔异常 */
    if (dt <= 0.0f)   dt = 1e-3f;
    if (dt >  0.5f)   dt = 0.5f;

    error = pid->setpoint - measurement;

    /* ---------- 比例项 P ---------- */
    p = pid->kp * error;

    /* ---------- 积分项 I ---------- */
    /* 写成 ki·error·dt 累加，这样 ki 的量纲是 1/s，换采样率不用重调参数 */
    pid->integral += pid->ki * error * dt;

    /* 积分限幅：最朴素的抗饱和 */
    if (pid->integral >  pid->i_max) pid->integral =  pid->i_max;
    if (pid->integral < -pid->i_max) pid->integral = -pid->i_max;

    /* ---------- 微分项 D ---------- */
    d = 0.0f;
    if (pid->first_run) {
        /* 第一次运行没有"上一次测量值"，强行微分会算出一个假的巨大冲击值 */
        pid->prev_measure = measurement;
        pid->d_filtered   = 0.0f;
        pid->first_run    = 0;
    } else {
        /* 微分先行：对测量值微分。y 变大说明系统正在朝目标走，要往回压，取负号 */
        d_raw = -pid->kd * (measurement - pid->prev_measure) / dt;

        if (pid->d_tau > 0.0f) {
            /* 一阶低通： y[k] = y[k-1] + α·(x[k] - y[k-1])，α = dt/(τ+dt)
             * 这是最省算力的一阶 IIR，一个乘加就够了，非常适合单片机。 */
            float alpha = dt / (pid->d_tau + dt);
            pid->d_filtered += alpha * (d_raw - pid->d_filtered);
            d = pid->d_filtered;
        } else {
            pid->d_filtered = d_raw;
            d = d_raw;
        }
    }
    pid->prev_measure = measurement;

    /* ---------- 求和并限幅 ---------- */
    out = p + pid->integral + d;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->out = out;
    return out;
}

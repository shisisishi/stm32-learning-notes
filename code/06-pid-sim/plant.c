/**
 * plant.c —— 直流电机模型实现（前向欧拉离散化）
 *
 * 关于"为什么这里可以用前向欧拉"：
 * 前向欧拉法是条件稳定的，要求步长 dt < 2·τ（τ 为最快的时间常数）。
 * 本模型里最快的时间常数是电气时间常数 τe = L/R。
 * 以 L=0.5mH、R=1Ω 为例，τe=0.5ms，所以 dt 取 20µs 时
 * dt/τe = 0.04，离稳定边界差得很远，精度也足够。
 *
 * 反过来，如果偷懒直接用 dt=1ms，dt/τe = 2，正好踩在稳定边界上，
 * 仿真会直接发散（数值炸掉）。这是很多同学写仿真时"结果乱飞"的原因。
 */
#include <math.h>
#include "plant.h"

void Motor_Init(Motor_t *m, float R, float L, float Ke, float Kt, float J, float B)
{
    m->R = R;
    m->L = L;
    m->Ke = Ke;
    m->Kt = Kt;
    m->J = J;
    m->B = B;
    Motor_Reset(m);
}

void Motor_InitFromRpmPerVolt(Motor_t *m, float rpm_per_volt, float R, float L, float J, float B)
{
    /* 空载时 ω ≈ u/Ke（忽略摩擦），所以 Ke = u/ω。
     * rpm → rad/s：乘 2π/60。于是 Ke = 60/(2π·rpm_per_volt) */
    float Ke = 60.0f / (6.2831853f * rpm_per_volt);
    Motor_Init(m, R, L, Ke, Ke, J, B);   /* 国际单位制下 Kt == Ke */
}

void Motor_Reset(Motor_t *m)
{
    m->i = 0.0f;
    m->omega = 0.0f;
    m->u = 0.0f;
    m->T_load = 0.0f;
}

void Motor_Step(Motor_t *m, float dt)
{
    /* 先把两个导数都算出来，再一起更新 —— 这叫"同步更新"。
     * 如果写完 i 再拿新的 i 去算 omega，就变成了半隐式欧拉，
     * 虽然更稳定，但和后面要对齐的数学推导就不一致了，容易讲不清。 */
    float di     = (m->u - m->R * m->i - m->Ke * m->omega) / m->L;
    float domega = (m->Kt * m->i - m->B * m->omega - m->T_load) / m->J;

    m->i     += di * dt;
    m->omega += domega * dt;
}

float Motor_SteadyStateOmega(const Motor_t *m, float u)
{
    /* 稳态：di/dt=0 且 domega/dt=0
     *   → i = (u - Ke·ω)/R
     *   → Kt·i = B·ω
     * 联立解得 ω = Kt·u / (Kt·Ke + B·R) */
    return (m->Kt * u) / (m->Kt * m->Ke + m->B * m->R);
}

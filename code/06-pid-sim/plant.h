/**
 * plant.h —— 被控对象模型：有刷直流电机的电气+机械双状态模型
 *
 * 为什么不用"一阶惯性环节"糊弄过去？
 * 因为一阶环节里"给电压→转速立刻开始涨"，看不出电感带来的电流滞后；
 * 而真实电机调电流环时，电感 L 是最要命的参数。所以老老实实写两阶：
 *
 *     电气方程： L·di/dt = u - R·i - Ke·ω
 *     机械方程： J·dω/dt = Kt·i - B·ω - T_load
 *
 *   其中  u      电枢电压 (V)
 *         i      电枢电流 (A)
 *         ω      转子角速度 (rad/s)
 *         R, L   电枢电阻(Ω) / 电感(H)
 *         Ke    反电动势常数 (V·s/rad)
 *         Kt    转矩常数   (N·m/A)   —— 理想电机 Kt = Ke（国际单位制下）
 *         J     转动惯量   (kg·m²)
 *         B     粘性摩擦系数 (N·m·s/rad)
 *         T_load 负载转矩 (N·m)
 *
 * 注意 Ke 的单位：工程上常写 rpm/V，这里统一换算成国际单位 rad/s/V。
 * 这是初学者最容易搞混的地方之一（换算见 plant.c 的 Motor_InitFromRpmPerVolt）。
 */
#ifndef PLANT_H
#define PLANT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 电机参数 */
    float R, L, Ke, Kt, J, B;

    /* 状态量 */
    float i;        /* 电枢电流 A */
    float omega;    /* 角速度 rad/s */

    /* 外部输入 */
    float u;        /* 电枢电压 V（由 PID 送来） */
    float T_load;   /* 负载转矩 N·m */
} Motor_t;

/** 直接按国际单位制初始化 */
void  Motor_Init(Motor_t *m, float R, float L, float Ke, float Kt, float J, float B);

/** 按工程习惯初始化：空载转速 rpm@电压V 反推 Ke */
void  Motor_InitFromRpmPerVolt(Motor_t *m, float rpm_per_volt, float R, float L, float J, float B);

/** 复位到静止状态 */
void  Motor_Reset(Motor_t *m);

/** 用前向欧拉法推进一步，dt 为积分步长（秒），必须远小于 L/R */
void  Motor_Step(Motor_t *m, float dt);

/** 理论空载稳态转速 = u / Ke，用来验证仿真结果对不对 */
float Motor_SteadyStateOmega(const Motor_t *m, float u);

#ifdef __cplusplus
}
#endif

#endif /* PLANT_H */

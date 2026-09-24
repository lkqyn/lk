#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

/*
 * 电机与编码器的已确认参数。
 * 所有宏名称都携带单位，控制算法不得在其他文件重复定义这些数值。
 */
#define MOTOR_MODEL_NAME                         "STP-59D5129"
#define MOTOR_STEP_ANGLE_DEG                     (1.8f)
#define MOTOR_FULL_STEPS_PER_REVOLUTION          (200U)
#define MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION   (50U)
#define MOTOR_HOLDING_TORQUE_NM                  (2.0f)
#define MOTOR_RATED_PHASE_CURRENT_A              (5.0f)
#define MOTOR_PHASE_RESISTANCE_OHM               (0.45f)
/* 铭牌相电感，已由静止20kHz电流阶跃的逐点di/dt结果验证。 */
#define MOTOR_PHASE_INDUCTANCE_H                 (0.00185f)
/*
 * 当前测试电流范围内，d/q轴初始小信号电感均与铭牌相电感一致。
 * 未经独立测量不得使用高速指令电压反推的等效值替代物理电感。
 */
#define MOTOR_D_AXIS_INDUCTANCE_H                (0.00185f)
#define MOTOR_Q_AXIS_INDUCTANCE_H                (0.00185f)
/*
 * 磁链已在Id约为0的200/300/400rpm实时点分别辨识；
 * 三档共同区间为0.007105~0.007468Wb，采用区间内原值。
 */
#define MOTOR_FLUX_LINKAGE_WB                    (0.007407f)
/*
 * d轴弱磁有效系数不等同于静止小信号电感：它综合反映绕组增量电感、
 * 逆变器非理想、采样/PWM延迟及尚未单独标定的相位误差。
 * 初值来自此前高速实时工作点的电压方程拟合，只允许用于弱磁规划与
 * q轴反电动势前馈，不得用于电流PI整定或磁阻转矩计算。
 *
 * q轴交叉耦合项必须使用已验证的物理Lq。此前由小Iq稳态点拟合出的
 * 5.597mH包含其他误差，放大到启动大Iq时会严重过补偿并导致电压饱和。
 */
#define MOTOR_FW_D_AXIS_FLUX_COEFFICIENT_H       (0.003223f)
#define MOTOR_ROTOR_INERTIA_KG_M2                (0.000047f)

#define MOTOR_ENCODER_CPR_PER_CHANNEL            (1000U)
#define MOTOR_ENCODER_COUNTS_PER_REVOLUTION      (4000L)
#define MOTOR_ENCODER_COUNTS_PER_ELECTRICAL_CYCLE \
    (MOTOR_ENCODER_COUNTS_PER_REVOLUTION / MOTOR_ELECTRICAL_CYCLES_PER_REVOLUTION)
/*
 * TIM2原始计数增加时，从电机输出轴端观察为顺时针。
 * 项目统一采用“从输出轴端观察，逆时针为正”的机械坐标系，
 * 因此在编码器边界将原始计数方向取反。
 */
#define MOTOR_ENCODER_OUTPUT_CCW_DIRECTION_SIGN  (-1L)

#endif

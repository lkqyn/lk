#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

/*
 * 1 kHz 速度 PI 与弱磁工作点规划模块。
 * 将机械速度误差转换为 Id/Iq 请求，并交由 20 kHz d/q 电流环执行。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 速度环基准参数：1kHz执行频率、10ms滑动窗口测速。
 * Kp按机械阻尼试验确定：空载400/700/900/1000rpm均已验证；
 * 8000uA/rpm在深弱磁区会失稳，因此默认值保留明确裕量。
 */
/* 已完成0~3000rpm闭环验证的默认速度PI。 */
#define SPEED_LOOP_DEFAULT_KP_UA_PER_RPM       (3800L)
#define SPEED_LOOP_DEFAULT_KI_UA_PER_RPM_S     (70000L)

/*
 * 弱磁规划电压比例支持停机时在75%~90%之间调整，默认90%。
 * 使用千分数保存，避免控制路径依赖文本浮点解析。
 * 逐整步电角度校准已经完成并通过正反向验证。自动弱磁允许搜索至
 * -2.5A，但该值只是规划边界；实际Id仍由实时速度、电压预算和3.5A
 * 电流圆共同决定，不允许把边界值作为固定弱磁电流使用。
 */
#define SPEED_LOOP_FIELD_WEAKENING_DEFAULT_RATIO_PERMILLE (900U)
#define SPEED_LOOP_FIELD_WEAKENING_MINIMUM_RATIO_PERMILLE (750U)
#define SPEED_LOOP_FIELD_WEAKENING_MAXIMUM_RATIO_PERMILLE (900U)
#define SPEED_LOOP_FIELD_WEAKENING_MINIMUM_ID_MA (-2500L)
/*
 * 进入深弱磁后，以电压椭圆与电流圆交点规划Id，保证可用转矩裕量；
 * 不使用瞬时Iq，避免速度PI的正常纹波调制弱磁电流。
 */
#define SPEED_LOOP_FULL_TORQUE_WEAKENING_START_MRPM (2400000L)
/* 实测2700rpm以上需先建立完整弱磁，避免高速掉速时撤弱磁。 */
#define SPEED_LOOP_HIGH_SPEED_PREWEAKENING_MRPM (2700000L)
/* 实测电压余量闭环的最低规划电压，避免修正器过度弱磁。 */
#define SPEED_LOOP_FIELD_WEAKENING_MIN_VOLTAGE_RATIO (0.75f)

typedef enum
{
    SPEED_LOOP_FAULT_NONE = 0,
    SPEED_LOOP_FAULT_START_FAILED,
    SPEED_LOOP_FAULT_TARGET_UPDATE,
    SPEED_LOOP_FAULT_CURRENT_LOOP,
    SPEED_LOOP_FAULT_BUS_VOLTAGE,
    SPEED_LOOP_FAULT_DIRECTION,
    SPEED_LOOP_FAULT_OVERSPEED,
    SPEED_LOOP_FAULT_HOLD_SPEED_RANGE,
    SPEED_LOOP_FAULT_TIMEOUT
} SpeedLoop_Fault_t;

typedef struct
{
    /* Kp的单位为uA/rpm。 */
    int32_t kp_ua_per_rpm;
    /* Ki的单位为uA/(rpm·s)。 */
    int32_t ki_ua_per_rpm_s;
    /* d/q目标电流矢量的幅值上限，单位mA。 */
    int32_t current_vector_limit_ma;
    int32_t maximum_speed_mrpm;
    /* 最大运行毫秒数；UINT32_MAX为持续运行，0无效。 */
    uint32_t maximum_run_time_ms;
    int8_t torque_direction_sign;
    /* 恒速模式检测实际速度是否长期反向；位置模式允许末端反向制动。 */
    uint8_t direction_check_enabled;
    /*
     * 定电流诊断：到达指定时刻后锁存物理Id/Iq，短时间旁路速度PI和弱磁调度。
     * hold_start_ms或hold_duration_ms为0时禁用，不影响正常速度模式。
     */
    uint32_t hold_start_ms;
    uint32_t hold_duration_ms;
    int32_t hold_minimum_abs_speed_mrpm;
    int32_t hold_maximum_abs_speed_mrpm;
} SpeedLoop_Config_t;

typedef struct
{
    int32_t target_speed_mrpm;
    int32_t measured_speed_mrpm;
    int32_t speed_error_mrpm;
    int32_t proportional_output_ma;
    int32_t integral_output_ma;
    int32_t effective_kp_ua_per_rpm;
    /* 速度PI输出的Id=0等效转矩电流，单位mA。 */
    int32_t logical_iq_target_ma;
    int32_t current_loop_id_target_ma;
    int32_t current_loop_iq_target_ma;
    /* 按磁阻转矩折算到Id=0工况的等效Iq，单位mA。 */
    int32_t torque_equivalent_iq_ma;
    uint32_t filtered_bus_voltage_mv;
    /* 母线越界故障触发瞬间锁存，进入故障后仍保留供串口诊断。 */
    uint32_t fault_bus_voltage_mv;
    uint16_t fault_bus_voltage_raw;
    /* 0=无锁存，1=低于下限，2=高于上限。 */
    uint8_t fault_bus_voltage_limit;
    int32_t current_voltage_limit_mv;
    /* 电压余量闭环从解析弱磁预算中扣除的电压，单位mV。 */
    int32_t field_weakening_voltage_correction_mv;
    uint16_t field_weakening_voltage_ratio_permille;
    int16_t fixed_weakening_id_ma;
    uint32_t elapsed_ms;
    SpeedLoop_Fault_t fault;
    uint8_t output_saturated;
    uint8_t field_weakening_active;
    uint8_t fixed_weakening_enabled;
    uint8_t current_hold_enabled;
    uint8_t current_hold_active;
    uint8_t current_hold_completed;
    uint8_t running;
} SpeedLoop_State_t;

/**
 * @brief 速度环1ms实时诊断样本。
 * @note 仅用于整定阶段的JustFloat逐点观测，不参与闭环计算。
 */
typedef struct
{
    uint32_t elapsed_ms;
    int32_t target_speed_mrpm;
    int32_t adaptive_speed_mrpm;
    int32_t fixed_5ms_speed_mrpm;
    int32_t fixed_10ms_speed_mrpm;
    int32_t encoder_delta_count;
    int32_t iq_target_ma;
    int32_t iq_measured_ma;
    int32_t id_measured_ma;
    int32_t electrical_count;
    /* 机械一圈内的编码器位置，范围0~3999计数。 */
    int32_t mechanical_count_in_revolution;
    int32_t output_d_mv;
    int32_t output_q_mv;
    int32_t feedforward_d_mv;
    int32_t feedforward_q_mv;
    uint32_t voltage_saturated_sample_count;
    uint32_t d_axis_priority_sample_count;
    int32_t id_target_ma;
    int32_t current_loop_iq_target_ma;
    int32_t torque_equivalent_iq_ma;
    int32_t integral_output_ma;
    int32_t effective_kp_ua_per_rpm;
    int32_t field_weakening_voltage_correction_mv;
    /* 原始角度减观测角度，已折算到-180000~180000毫电角度。 */
    int32_t observer_phase_error_mdeg;
} SpeedLoop_TraceSample_t;

/* 初始化速度环状态和诊断队列，不启动电流环。 */
void SpeedLoop_Init(void);

/**
 * @brief 启动1kHz速度环和20kHz内层电流环。
 * @param config 速度PI、安全限制和方向配置。
 * @param target_speed_mrpm 目标机械转速，逆时针为正。
 */
uint8_t SpeedLoop_Start(const SpeedLoop_Config_t *config,
                        int32_t target_speed_mrpm);
/* 在线更新机械速度目标，单位 mrpm。 */
uint8_t SpeedLoop_SetTargetSpeedMrpm(int32_t target_speed_mrpm);

/* 清除速度PI积分，仅用于外部脉冲末端制动的状态切换。 */
uint8_t SpeedLoop_ResetIntegrator(void);

/* 仅限制“转矩方向与当前转速相反”的回馈制动电流；0表示关闭限制。 */
uint8_t SpeedLoop_SetExternalBrakingCurrentLimitMa(int32_t limit_ma);

/*
 * 外部位置伺服的转矩通道：直接指定等效Iq，旁路速度PI，
 * 但仍经过速度环的电流限幅、母线保护与20kHz电流环。
 */
uint8_t SpeedLoop_SetExternalTorqueOverride(int32_t torque_iq_ma,
                                            uint8_t enabled);

/*
 * @brief 将已经启用的外部直接转矩命令立即下发到d/q电流环。
 * @note 仅供4kHz外部STEP位置控制使用；弱磁、电流圆及电压前馈仍复用
 *       速度环内部的同一套工作点计算，1kHz速度环继续负责母线保护。
 */
uint8_t SpeedLoop_ApplyExternalTorqueOverrideFast(void);

/**
 * @brief 原子更新速度PI参数，支持调试期间在线整定。
 * @note Ki设为0时同时清空积分器，保证纯P试验不受历史积分影响。
 */
uint8_t SpeedLoop_SetTunings(int32_t kp_ua_per_rpm,
                             int32_t ki_ua_per_rpm_s);

/**
 * @brief 设置弱磁规划电压占电流环电压上限的千分比。
 * @note 只允许在速度环停止时设置，范围750~900。
 */
uint8_t SpeedLoop_SetFieldWeakeningVoltageRatioPermille(
    uint32_t ratio_permille);

/**
 * @brief 启用固定Id弱磁诊断，隔离自动弱磁工作点调度。
 * @note 只允许停机设置，Id范围-2500~-100mA。
 */
uint8_t SpeedLoop_SetFixedWeakeningIdMa(int32_t fixed_id_ma);

/**
 * @brief 退出固定Id诊断并恢复自动弱磁规划。
 */
uint8_t SpeedLoop_DisableFixedWeakening(void);

/* 停止速度环和其拥有的 d/q 电流环。 */
void SpeedLoop_Stop(void);
/* 1 kHz 速度 PI、弱磁、母线保护和工作点更新。 */
void SpeedLoop_Tick1ms(void);
/* 获取速度环的只读状态快照。 */
const volatile SpeedLoop_State_t *SpeedLoop_GetState(void);

/**
 * @brief 从1kHz实时诊断队列读取一个样本。
 * @return 1表示读取成功，0表示当前没有样本。
 */
uint8_t SpeedLoop_PopTraceSample(SpeedLoop_TraceSample_t *sample);

/**
 * @brief 获取因串口来不及发送而被覆盖的诊断样本数量。
 */
uint32_t SpeedLoop_GetTraceDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif

/*
 * 集中保存大小机的几何、驱动映射、补偿表、遥控和LQR/RL参数。
 * 编译机型及节拍/功能开关在头文件，具体数值在machine_table；物理标定值须由作者台架确认。
 */

#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include "motion_guard.h"
#include <stdint.h>
#define MACHINE_BALANCE_USE_MPC 0u
#define MACHINE_ROLL_HEIGHT_USE_MPC 0u

#define MACHINE_MPC_REFERENCE_PREDICT 1u /* 四态真实长度斜坡预览；不改变十态参考生成。 */
#define MACHINE_MPC_INPUT_CONSTRAINT 1u
#if MACHINE_ROLL_HEIGHT_USE_MPC && !MACHINE_MPC_INPUT_CONSTRAINT
#error "MPC requires input constraints"
#endif

#define MACHINE_LATENCY_CAPTURE_ENABLE              0u /* 完整时序采集；0编译移除测点/缓冲/新增IRQ。 */
#define MACHINE_MOTION_GUARD_ENABLE                 1u /* 0：原偏航参考；1：大机NORMAL防翻参考约束，改后重新编译烧录 */
#define MACHINE_NORMAL_LEG_CONTROL                  1u  /* 0：原PD；1：NORMAL四态后端（LQR/MPC），控制腿长和roll */
#define MACHINE_ROLL_HEIGHT_OBSERVER_ENABLE         0u /* 4态leso观测器 */
/* 轴向力补偿，*/
#define MACHINE_HEIGHT_TRIM_ENABLE                  1u
#define MACHINE_HEIGHT_TRIM_LOW_N (-70.0f)
#define MACHINE_HEIGHT_TRIM_HIGH_N (-8.0f)

/* 机器编号 */
#define MACHINE_ID_BIG_WHEELLEG    0u
#define MACHINE_ID_SMALL_WHEELLEG  1u

#define MACHINE_NUM               2u

/* 两份机器配置表同时编译，各自引用机械腿长范围，单位m。 */
#define MACHINE_BIG_LEG_LEN_MIN   0.14f
#define MACHINE_BIG_LEG_LEN_MAX   0.33f

#define MACHINE_SMALL_LEG_LEN_MIN 0.13f
#define MACHINE_SMALL_LEG_LEN_MAX 0.23f
/* 电机数量 (须与 dm.c / dji.c 的枚举一致) */
#define MACHINE_LEG_NUM           4u
#define MACHINE_WHEEL_NUM         2u

/* 编译选机，参数与 K 表同步绑定。 */
#ifndef MACHINE_DEFAULT
#define MACHINE_DEFAULT           MACHINE_ID_BIG_WHEELLEG
#endif

#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
#define MACHINE_TIM6_PERIOD       999u    /* 1 kHz */
#define MACHINE_TICK_DT           0.001f
#define MACHINE_RL_CTRL_DIV       2u      /* RL实体PD/反馈采样/正常下发：500Hz */
#define MACHINE_POLICY_DIV        10u     /* 8000 v2：25/125双输入，100Hz策略/500Hz PD */
#define MACHINE_DM_OFFLINE_MS     50u
#define MACHINE_VOFA_PORT         8u      /* UART8 */
#define MACHINE_FDCAN13_DATA_PRESCALER  1u
#define MACHINE_FDCAN13_DATA_SEG1       4u
#define MACHINE_FDCAN13_DATA_SEG2       1u
#elif MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
#define MACHINE_TIM6_PERIOD       999u    /* 1 kHz */
#define MACHINE_TICK_DT           0.001f
#define MACHINE_RL_CTRL_DIV       1u      /* 小机兼容 */
#define MACHINE_DM_OFFLINE_MS     10u
#define MACHINE_POLICY_DIV        10u     /* 策略节拍 1000/10 = 100 Hz */
#define MACHINE_VOFA_PORT         1u      /* UART1 */
#define MACHINE_FDCAN13_DATA_PRESCALER  3u
#define MACHINE_FDCAN13_DATA_SEG1       5u
#define MACHINE_FDCAN13_DATA_SEG2       2u
#else
#error "Unsupported MACHINE_DEFAULT"
#endif

#define MACHINE_LQR_DT            MACHINE_TICK_DT
#define MACHINE_RL_CTRL_DT        (MACHINE_TICK_DT * (float)MACHINE_RL_CTRL_DIV)
/* 训练XML直立目标换算：轮接地半径减髋轴相对base_link高度，非实机标定。 */
#define MACHINE_RL_LEG_TO_HEIGHT  (0.06f - 0.02352f)
#define MACHINE_POLICY_DT         (MACHINE_TICK_DT * (float)MACHINE_POLICY_DIV)


/* 一路电机的极性 */
typedef struct {
    int8_t fb;      /* 反馈极性 */
    int8_t out;     /* 输出极性 */
} motor_sign_t;

/* IMU 安装: 机体三轴各取模块哪一路、乘什么符号 (输出 = 原始值 × 符号) */
typedef struct {
    uint8_t eul_src[3];   /* 欧拉角来源, 按机体 俯仰/横滚/偏航 序; 值 = 模块 0 Roll / 1 Pitch / 2 Yaw */
    int8_t  eul_sign[3];  /* 欧拉角符号, 同序 */
    uint8_t gyr_src[3];   /* 角速度来源, 按机体 X/Y/Z 序 */
    int8_t  gyr_sign[3];  /* 角速度符号, 输出 X/Y/Z */
    uint8_t acc_src[3];   /* 加速度来源, 按机体 X/Y/Z 序 */
    int8_t  acc_sign[3];  /* 加速度符号, 输出 X/Y/Z */
    uint8_t quat_src[3];  /* 四元数输出 X/Y/Z 分别取模块 X/Y/Z 哪一路 */
    int8_t  quat_sign[3]; /* 四元数输出 X/Y/Z 极性, 独立于 quat_src */
} imu_cfg_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_limit;
} machine_pid_cfg_t;

typedef struct {
    uint16_t deadband_vel; /* DR16原始计数 */
    uint16_t deadband_yaw;
    uint16_t deadband_len;
    float vel_max; /* m/s，前进上限 */
    float vel_max_rev; /* m/s，倒车幅值上限 */
    float yaw_max; /* rad/s */
    float len_init; /* m */
    float len_rate; /* m/s */
} machine_rc_cfg_t;

/* 8000 v2：先缩放，当前帧整体裁剪；历史保存裁剪前帧。 */
typedef struct {
    float gyro_scale;          /* 机体角速度观测缩放 */
    float height_scale;        /* 目标高度命令观测缩放 */
    float joint_vel_scale;     /* 六路电机速度观测缩放 */
    float obs_clip;            /* 缩放后完整当前帧裁剪幅值 */
    float velocity_command_scale; /* 前向速度命令缩放 */
} machine_rl_observation_cfg_t;
typedef struct {
    float pos_scale;           /* action→相对名义角偏移，rad/action */
    float wheel_vel_scale;     /* action→轮轴速度目标，rad/s/action */
    float p_gains[4];          /* 实体电机位置比例，Nm/rad */
    float d_gains[4];          /* 实体电机速度阻尼，Nm/(rad/s) */
    float wheel_gain[2];       /* 轮轴速度比例，Nm/(rad/s) */
    float position_limit;     /* q_reference两侧的目标范围，rad；不限制反馈或环绕角差 */
    float wheel_speed_limit;  /* 轮轴速度目标幅值，rad/s */
    float leg_torque_limit;   /* RL腿力矩限幅，Nm；与MIT编码刻度分开 */
    float wheel_torque_limit; /* RL轮力矩限幅，Nm；与公共满电流限幅分开 */
} machine_rl_torque_cfg_t;
/* 实体顺序LF前/后、RF前/后、左轮/右轮。
 * 电机反馈已包含驱动机械标定；motor_zero基于pos_continuous_rad，不再叠加旧虚拟关节零位。
 * 配对映射将公共驱动坐标转换至RL坐标，不能照网络action顺序填写。 */
/* 公共起立/恢复过程 */
typedef struct {
    float terminal_gravity_z; /* 模型倾倒终止：机体系单位重力Z大于此值。 */
    uint32_t terminal_ms; /* 100Hz连续超过100步：1010ms，非起立动作超时。 */
} machine_rl_recovery_cfg_t;

/* RL动作新鲜度期限，ms */
typedef struct {
    uint32_t action_timeout_ms; /* 已发布动作停止更新的故障期限 */
    uint32_t ready_timeout_ms; /* 实际投入后动作持续不可用的故障期限 */
} machine_rl_timing_cfg_t;

typedef struct {

    int8_t motor_sign[6];      /* 固件↔模型配对方向，输入角/速与输出力矩共用 */
    float motor_zero[4];       /* 固件标定连续角至模型零位的额外偏移，rad */
    float nominal_pos[4];      /* q_nominal：训练默认站立原生角，观测减此值、目标加此值 */
    float position_limit_center[4];    /* q_reference：XML的qpos0，目标限幅中心，非实机零偏 */
    machine_rl_observation_cfg_t observation;
    machine_rl_torque_cfg_t torque;


    machine_rl_recovery_cfg_t recovery; /* RL独立倾倒终止，非自起动作 */

    machine_rl_timing_cfg_t timing; /* RL动作新鲜度期限，ms */
} machine_rl_cfg_t;

typedef struct {
    float mass; /* 向心补偿采用的机身质量kg；0不补偿 */
    float track; /* 左右轮接地点全轮距m */
    float height_offset; /* 轮半径与机身质心相对髋轴高度之和m */
    float gain; /* 向心补偿比例 */
} machine_turn_comp_cfg_t;

/* 十态姿态参考及偏航反馈开关 */
typedef struct {
    float leg_trim[2];
    float pitch_trim;
    uint8_t yaw_hold;
    uint8_t yaw_rate_hold;
} machine_lqr_reference_cfg_t;

/* 停车位置保持；m、m/s、rad */
typedef struct {
    float pos_target;
    float pos_reset_dist; /* m */
    float pos_arm_vel; /* m/s */
    float pos_leg_tol; /* rad */
    uint8_t pos_hold;
} machine_lqr_position_hold_cfg_t;

/* 状态低通，顺序见结构体 */
typedef struct {
    float lpf_alpha[4]; /* [0]pitch角速度 [1]yaw角速度 [2]roll角速度 [3]高度变化速度；系数越小越平滑、延迟越大 */
} machine_lqr_filter_cfg_t;

/* 四态轴向/高度控制；N、m/s² */
typedef struct {
    float leg_force_min; /* 四态单腿轴向力下限N */
    float length_ref_accel_max; /* NORMAL长度参考加速度上限m/s²，不影响起立/翻身 */
    float roll_height_bias_common_max; /* 观测器平均偏置上限N/腿，不是腿力输出限幅 */
    float roll_height_bias_diff_max; /* 左右偏置之差上限N，不是单腿力输出限幅 */
    float support_force[2];
    struct {
        uint16_t max_iter; /* MPC有限迭代上限；模型/rho/窗口由生成数据管理。 */
        float primal_tolerance; /* 输入共识残差N。 */
        float dual_tolerance; /* rho加权对偶残差，求解器坐标。 */
        uint16_t solve_div; /* MPC重算/输入保持拍数，模型周期须等于基础周期*solve_div。 */
    } mpc;
} machine_lqr_roll_height_cfg_t;

/* 原腿长/roll PD；参数口径保持 */
typedef struct {
    machine_pid_cfg_t leg_len[2];
    machine_pid_cfg_t roll;
} machine_lqr_pd_cfg_t;

/* 十态输出通道选择 */
typedef struct {
    uint8_t wheel_enable;
    uint8_t leg_swing_enable;
} machine_lqr_output_cfg_t;

typedef struct {


    machine_turn_comp_cfg_t turn_comp;
    motion_guard_param_t motion_guard; /* NORMAL偏航参考约束策略 */


    machine_lqr_reference_cfg_t reference; /* 十态姿态参考及偏航反馈开关 */

    machine_lqr_position_hold_cfg_t position_hold; /* 停车位置保持；m、m/s、rad */

    machine_lqr_filter_cfg_t filter; /* 状态低通，顺序见结构体 */

    machine_lqr_roll_height_cfg_t roll_height; /* 四态轴向/高度控制；N、m/s² */

    machine_lqr_pd_cfg_t pd; /* 原腿长/roll PD；参数口径保持 */

    machine_lqr_output_cfg_t output; /* 十态输出通道选择 */
} machine_lqr_cfg_t;

typedef struct {
    float len_min;
    float len_max;
    float bar_sum_sq;
    float bar_prod2;
    float anchor_sum_sq;
    float anchor_prod2;
    float phase;
    float numerator_scale;
    float denominator_scale;
    float force_n;
} machine_spring_cfg_t;

extern const machine_spring_cfg_t machine_spring_leg3;

typedef struct {
    float length;
    float offset;
    float radius;
} machine_gravity_node_t;

typedef struct {
    float mass;
    float gravity;
    const machine_gravity_node_t *nodes;
    uint8_t count;
} machine_gravity_cfg_t;

/* 一台机器的全部参数 */
/* 轮驱动：型号、传动、限矩、总线和标定 */
typedef struct {
    uint8_t     dji_type;
    float       dji_gear_ratio;
    float       dji_trq_clamp;
    float       wheel_r;
    motor_sign_t dji_sign[MACHINE_WHEEL_NUM];
    uint8_t     dji_bus;
} machine_wheel_cfg_t;

/* 髋驱动：MIT刻度、限矩、总线和标定；不改物理值 */
typedef struct {
    float       dm_pos_max;
    float       dm_vel_max;
    float       dm_trq_max;
    float       dm_trq_clamp;
    motor_sign_t dm_sign[MACHINE_LEG_NUM];
    uint8_t     dm_bus[MACHINE_LEG_NUM];
    float       dm_zero[MACHINE_LEG_NUM];
} machine_hip_cfg_t;

/* 机构几何及机械工作范围 */
typedef struct {
    float       leg_lu;
    float       leg_lg;
    float       leg_len_min;
    float       leg_len_max;
    float       leg_off_phi0[2];
} machine_leg_cfg_t;

/* 公共物理补偿模型 */
typedef struct {
    const machine_spring_cfg_t *spring;
    const machine_gravity_cfg_t *gravity;
} machine_compensation_cfg_t;

/* 公共起立/恢复过程 */
typedef struct {
    float fallen_support_angle_range; /* 起立支撑权重角范围rad */
} machine_recovery_cfg_t;

/* 普通自起/台阶共用参数源；状态实例仍由各自过程持有。 */
typedef struct {
    float recovery_settle_time; /* 倒置稳定等待时间s。 */
    float recovery_len; /* 翻身准备/扫腿目标长度m。 */
    float recovery_front_angle; /* 倒置专用前点rad，与普通front_angle独立 */
    float angle_rate; /* 普通自起准备位及摆正角目标速率rad/s。 */
    float recovery_angle_rate; /* 倒置前点对齐及翻身扫角速率rad/s。 */
    float recovery_pitch_tol; /* 翻身回正俯仰窗口rad。 */
    float recovery_stall_time; /* 翻身卡滞允许持续时间s。 */
    uint8_t recovery_retry_max; /* 允许消耗的地面恢复重试次数上限。 */
    float retry_wait; /* 任务层撤力后重新初判前等待时间s。 */
    float length_rate; /* 腿长m/s */
    float extend_len; /* 普通起立伸腿准备长度m。 */
    float length_tol; /* 伸/收腿到位容差m。 */
    float length_timeout; /* 伸/收腿阶段最长时间s。 */
    float front_angle; /* 普通自起前侧准备角rad，几何角表示 */
    float normal_swing_window; /* 普通入口可直接收短/摆正的角区间rad，不是最终到位容差 */
    float rear_angle; /* 普通后侧准备位几何角rad。 */
    float angle_tol; /* 普通准备位/摆正及倒置前点对齐共用到位容差rad */
    float angle_timeout; /* 准备位对齐/普通摆正阶段最长时间s。 */
    float retract_len; /* 普通起立收腿目标m。 */
    float angle_pos_kp; /* 摆角外环比例系数，具体离散用法见PID初始化。 */
    float angle_pos_kd; /* 摆角外环误差差分系数，不等于物理速度阻尼Kd。 */
    float angle_speed_kp; /* 角速度内环比例系数。 */
    float angle_speed_kd; /* 角速度内环误差差分系数。 */
    float angle_speed_max; /* 摆角外环输出的角速度目标上限rad/s。 */
    float stable_time; /* 到位条件连续确认时间s。 */
    float support_time; /* 支撑前馈从0渐入到1的时间s。 */
} fallen_process_param_t;

extern fallen_process_param_t fallen_process_param;

enum {
    STAIR_MODE_OFF = 0, /* 不检测、不接管 */
    STAIR_MODE_OBSERVE, /* 记录特征/确认后的候选，不接管 */
    STAIR_MODE_AUTO, /* 本机参数确认后允许自动接管 */
};

typedef struct {
    uint8_t confirmed; /* 本机确认后允许AUTO消费事件；不阻止OBSERVE计算试用候选 */
    float minimum_length; /* 双腿实测长度均不小于此值m才允许检测 */
    float minimum_forward_speed; /* 最近有效LQR融合前向速度严格大于此值m/s才允许检测 */
    float angle_threshold; /* 两腿世界角偏差都小于此负阈值rad */
    float acceleration_threshold; /* 任一侧世界角加速度不大于此负阈值rad/s² */
    float acceleration_tau; /* 角速度差分的一阶低通时间常数s */
    float confirm_time; /* 两个判据联合成立的连续确认时间s */
} stair_detection_param_t;

typedef struct {
    uint8_t confirmed; /* 作者允许试用动作后置1，不代表实机越阶已验收 */
    float angle_rate; /* 台阶后摆及委托摆正角目标速率rad/s */
    float angle_tolerance; /* 后摆到位容差rad；收腿/摆正容差归普通自起 */
    float confirm_time; /* 双腿实际/指令到位连续确认时间s */
    float stall_angle_progress; /* 后摆卡滞窗口内最小角进度rad */
    uint32_t swing_timeout_ms; /* 后摆阶段真实期限ms，收腿/摆正期限归普通自起 */
    uint32_t total_timeout_ms, stall_timeout_ms, wait_ms; /* 总期限、卡滞窗口及失败撤力等待ms */
} stair_process_param_t;

typedef struct {
    uint8_t mode; /* STAIR_MODE_* */
    stair_detection_param_t detection;
    stair_process_param_t process;
} machine_stair_cfg_t;

typedef struct {

    const char *name;

    machine_rc_cfg_t rc;        /* 公共量程 */
    machine_lqr_cfg_t lqr;
    machine_stair_cfg_t stair; /* 台阶检测/动作，参数及状态不归自起 */

    /* 轮: 型号(0=M2006, 1=M3508) + 总传动比 + 满限幅力矩(Nm) */

    /* 腿: MIT 三个满量程 + 满限幅力矩(Nm) */

    /* 极性: 腿 4 台 (前左/后左/前右/后右), 轮 2 个 (左/右) */


    /* 总线号: 1=FDCAN1 2=FDCAN2 3=FDCAN3 */


    /* 电机零点: 4 台腿 (前左/后左/前右/后右), 在 dm.c 解码时叠加 */

    /* 腿几何: 杆长 + 腿长工作区间 + 腿摆角零位 (左/右) */

    /* IMU 安装极性 */
    imu_cfg_t   imu;
    /* RL 默认配置 */
    machine_rl_cfg_t rl;

    machine_wheel_cfg_t wheel; /* 轮驱动：型号、传动、限矩、总线和标定 */

    machine_hip_cfg_t hip; /* 髋驱动：MIT刻度、限矩、总线和标定；不改物理值 */

    machine_leg_cfg_t leg; /* 机构几何及机械工作范围 */

    machine_compensation_cfg_t compensation; /* 公共物理补偿模型 */

    machine_recovery_cfg_t recovery; /* 公共起立/恢复过程 */
} machine_cfg_t;

extern const machine_cfg_t machine_table[MACHINE_NUM];
extern const machine_cfg_t *const machine; /* 编译期选定的机器 */


#endif

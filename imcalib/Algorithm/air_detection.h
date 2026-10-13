/*
 * 文件职责：离地/落地状态估计（air_detection.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：执行任务传入的双腿几何、四髋反馈力矩与IMU。
 * 输出/去向：发布支持力、armed/valid/flight及压缩诊断，执行层决定是否切换控制。
 * 关键边界：按固定执行周期处理缓存反馈；无效观测保留flight。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef AIR_DETECTION_H
#define AIR_DETECTION_H

#include "leg_solver.h"
#include "imu_state.h"
#include "leg_compensation.h"

/* 参数数值初始化时检查；检测、控制接入和落地开关各自独立。 */
typedef struct {
    uint8_t enabled; /* 启用观察器；关闭会走检测复位 */
    uint8_t control_enabled; /* 允许执行层使用AIR反馈选择，不等同于检测有效 */
    uint8_t landing_enabled; /* 允许AIR退出；关闭则保持离地并继续观测 */
    uint8_t compression_enabled; /* 1用相对峰值压缩落地；0用原支持力落地 */
    float wheel_mass; /* 每侧轮组模型质量kg */
    float force_off; /* 均值低于此值才累计离地时间，N */
    float force_on; /* 初次支撑/原落地支持力门槛，N */
    float time_off; /* 离地条件需连续满足的时间s */
    float time_on; /* 初次支撑及原支持力落地持续时间s */
    float derivative_tau; /* 腿竖直加速度差分低通时间常数s */
    float force_tau; /* 支持力低通时间常数s */
    float hip_max; /* AIR摆矩限幅N·m，执行层使用 */
    float compression_on; /* 峰值减实测腿长的压缩门槛m */
    float compression_time; /* 单腿压缩条件连续持续时间s */
} air_param_t;

/* 执行层提供的本次快照：数组第一维左右腿，第二维同腿前/后髋。 */
typedef struct {
    const imu_state_t *imu; /* 已映射/归一化的IMU测量，只读引用 */
    const leg_state_t *leg[2]; /* 左右腿几何/速度/雅可比，只读引用 */
    const leg_compensation_values_t *compensation; /* 本拍模型值 */
    float torque[2][2]; /* 四髋物理反馈力矩N·m，已处理驱动极性 */
    uint8_t joint_ready; /* 四髋可用 */
    uint8_t permit; /* 当前模式/使能/自起条件是否允许检测 */
} air_input_t;

/* 单腿估计中间量：电机贡献、实体模型、投影与轮重项分开保存便于判断。 */
typedef struct {
    float motor_force; /* 由反馈反解的电机轴向力N，不含实体弹簧 */
    float motor_torque; /* 由反馈反解的虚拟腿摆矩N·m */
    float spring_force; /* 实体弹簧模型伸腿力N，释放时仍存在 */
    float gravity_torque; /* 估计中扣除的完整自重摆矩N·m */
    float pressure; /* 虚拟腿力投影后的竖直下压力N */
    float wheel_acceleration; /* 机体竖直加速度减腿投影加速度，m/s² */
    float force_raw; /* 含轮重动态项的未滤波FN，N */
    float force; /* 滤波后的FN，进入左右均值判据 */
    float height_velocity; /* 腿长/摆角投影形成的竖直速度m/s */
    float height_acceleration; /* 投影速度差分再滤波的加速度m/s² */
} air_leg_t;

/* 每腿独立压缩判据历史：重新离地播种；落地后保留末态作诊断。 */
typedef struct {
    float peak_length; /* 本次有效AIR反馈中的实测腿长最大值m */
    float compression; /* peak_length减当前实测腿长，m */
    float elapsed; /* 本腿连续达到压缩门槛的时间s，中断清零 */
} air_landing_t;

/* 检测发布状态：有效性与接触状态独立，不得将valid=0解释成自动落地。 */
typedef struct {
    air_leg_t leg[2]; /* 左右支持力估计 */
    air_landing_t landing[2]; /* 左右峰值/压缩/独立持续时间 */
    float mean_force; /* 左右滤波FN的均值N */
    float elapsed; /* 当前条件计时s；压缩AIR时为两腿计时最大值 */
    uint8_t flight; /* 1判定离地，0接地；短暂无效可保留上次值 */
    uint8_t armed; /* 已连续确认首次支撑，才允许判离地 */
    uint8_t valid; /* 最近估计是否有效，不表示电机执行成功 */
    uint8_t fault; /* 本拍估计失败，不等同ctrl_fault */
    uint8_t applied; /* 执行层是否选中AIR控制，不证明实体执行 */
} air_detection_t;

extern air_param_t air_param;
extern air_detection_t air_detection;
extern volatile air_detection_t air_debug;
uint8_t Air_Detection_Init(air_detection_t *state);
void Air_Detection_Reset(air_detection_t *state);
void Air_Detection_Update(air_detection_t *state, const air_input_t *input);
uint8_t Air_Detection_Active(const air_detection_t *state);

#endif

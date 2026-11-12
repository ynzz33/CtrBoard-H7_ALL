/* 跳跃单次动作及独立退出阶段；蓄力委托调用方普通LQR，不访问任务全局、AIR或自起状态。当前未接入控制流程。 */
#ifndef JUMP_PROCESS_H
#define JUMP_PROCESS_H

#include "machine_config.h"
#include "leg_solver.h"
#include "leg_compensation.h"
#include "torque_output.h"
#include "simple-function.h"

enum {
    JUMP_PROCESS_IDLE = 0, /* 无跳跃动作 */
    JUMP_PROCESS_CROUCH, /* 蓄力：不归属输出，调用方普通LQR跟踪length_target */
    JUMP_PROCESS_THRUST, /* 蹬伸：随腿长的静态支撑力+爬升推力，摆矩/轮矩沿用普通候选 */
    JUMP_PROCESS_FLIGHT, /* 空中：从实测起步收腿，腿角PD，轮矩0 */
    JUMP_PROCESS_LAND, /* 落地：从实测起步低刚度高阻尼缓冲，支撑渐入 */
    JUMP_PROCESS_HANDOFF, /* 保持落地输出，等待调用方LQR交接成功后Complete */
    JUMP_PROCESS_WAIT, /* 失败撤力，独立等待 */
    JUMP_PROCESS_RECHECK, /* 零输出，等待任务层重新初判 */
};

enum {
    JUMP_RESULT_IDLE = 0, JUMP_RESULT_RUNNING, JUMP_RESULT_DONE,
    JUMP_RESULT_FAILED, JUMP_RESULT_CANCELLED,
};

enum {
    JUMP_EXIT_NONE = 0, JUMP_EXIT_COMPLETE, JUMP_EXIT_CANCELLED,
    JUMP_EXIT_INPUT, JUMP_EXIT_MAPPING, JUMP_EXIT_POSE,
    JUMP_EXIT_CROUCH_TIMEOUT, JUMP_EXIT_THRUST_TIMEOUT,
    JUMP_EXIT_HANDOFF_TIMEOUT, JUMP_EXIT_TOTAL_TIMEOUT,
};

/* 着地确认当拍成立的判据位 */
enum {
    JUMP_TOUCH_FORCE = 1u, JUMP_TOUCH_COMPRESSION = 2u,
    JUMP_TOUCH_SPEED = 4u, JUMP_TOUCH_TIMEOUT = 8u,
};

/* 本次动作内锁存的诊断位 */
enum {
    JUMP_FLAG_UNLOADED = 1u, /* 支持力已确认卸载 */
    JUMP_FLAG_STROKE_END = 2u, /* 蹬伸由行程保护结束，而非起飞确认 */
    JUMP_FLAG_FALLBACK = 4u, /* 需要普通候选时缺失，改用腿角PD且轮矩0 */
    JUMP_FLAG_WHEEL_LIMIT = 8u, /* 普通候选轮矩达到轮限矩 */
    JUMP_FLAG_RETRACTED = 16u, /* 收腿给定已到retract_len且腿速已降下，几何着地判据生效 */
};

typedef struct {
    const leg_state_t *leg[2]; /* 本拍左右腿，已发布几何、速度及力映射 */
    const leg_compensation_values_t *compensation; /* 本拍弹簧/重力补偿缓存 */
    float pitch, pitch_rate; /* 与LQR同口径机身俯仰rad、rad/s；世界腿角=几何腿角-pitch */
    float roll, roll_rate; /* 与LQR同口径机身横滚rad、rad/s */
    float support[2]; /* 左右支持力估计N，来自跳跃期间持续运行的AIR观测 */
    uint8_t support_valid; /* 支持力估计本拍有效；无效时起飞只靠行程、着地只靠几何/速度 */
    float static_force[2]; /* 实测腿长下普通控制静态支撑力N（support_force+height trim），由调用方按普通控制同一公式提供 */
    float normal_torque[2]; /* 本拍普通控制实际采用的虚拟腿摆矩Tp N·m */
    float normal_wheel[2]; /* 本拍普通控制实际采用的左右轮矩N·m */
    uint8_t normal_valid; /* 普通候选有效；蹬伸/落地/交接据此沿用Tp和轮矩 */
    uint32_t now_ms; /* 真实时钟，允许uint32回绕 */
    uint8_t permit; /* 基础出力、四髋可用、LQR策略及投入许可；不检查AIR/FALLEN */
} jump_process_input_t;

typedef struct {
    uint8_t phase, result, exit_reason;
    uint8_t touchdown_source; /* JUMP_TOUCH_*位 */
    uint8_t flags; /* JUMP_FLAG_*位 */
    uint8_t recheck_pending; /* 失败退出后只请求一次检测历史复位 */
    uint32_t sequence, begin_ms, phase_begin_ms;
    float elapsed, total_elapsed; /* 阶段、动作累计时间s */
    float confirm_elapsed; /* 当前阶段转段条件连续成立时间s */
    float thrust_scale; /* 推力系数0～1 */
    float land_scale; /* 落地支撑渐入0～1 */
    float length_target[2]; /* 本拍腿长指令m：蓄力交普通LQR，蹬伸为行程保护，空中/落地为斜坡给定 */
    ramp_t length_ramp[2]; /* 收腿/缓冲给定，转段时从实测腿长起步 */
    float force[2], torque[2], wheel[2]; /* 基础轴向力N、摆矩N·m、轮矩N·m，腿部不含物理补偿 */
    compensation_debug_t compensation;
} jump_process_t;

extern jump_process_t jump_process;

void Jump_Process_Init(jump_process_t *state);
void Jump_Process_Reset(jump_process_t *state);
/* 蹬伸起至RECHECK归属输出；蓄力期间仍由普通LQR出力。 */
uint8_t Jump_Process_Owns_Output(const jump_process_t *state);
/* 仅IDLE、参数已确认且许可成立时进入蓄力。 */
uint8_t Jump_Process_Begin(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input);
/* 蓄力直接释放回IDLE；蹬伸～交接转WAIT撤力。 */
void Jump_Process_Abort(jump_process_t *state, uint8_t reason, uint32_t now_ms);
/* 仅交接阶段、调用方已生成有效LQR候选后记录完成并释放归属。 */
void Jump_Process_Complete(jump_process_t *state, uint32_t now_ms);
/* WAIT到RECHECK只返回一次1，调用者据此复位公共检测历史。 */
uint8_t Jump_Process_Advance_Exit(jump_process_t *state, const jump_process_param_t *param, uint32_t now_ms);
/* 每执行拍一次。蓄力不修改output，调用方沿用普通候选；接管阶段先清候选，成功才发布有效结果，中止当拍保持零候选。 */
void Jump_Process_Update(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input, float dt, torque_output_t *output);

#endif

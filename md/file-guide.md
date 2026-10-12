# 工程文件职责与阅读顺序

2026-10-11：按作者要求补充源码说明，并删除工程内Python文件。本文解释文件职责和调用关系；实时参数和当前行为以源码为准。自动生成/第三方库的用途在本文说明，模型权重和厂商算法不改写。

## 先看主控制链

1. [main.c](../Core/Src/main.c)：上电、时钟/外设、通信与控制初始化，启动调度。
2. [freertos.c](../Core/Src/freertos.c)：创建任务和安排循环/信号量等待；具体业务在imcalib/task。
3. [task_imu.c](../imcalib/task/task_imu.c)与[task_comm.c](../imcalib/task/task_comm.c)：分别准备IMU和电机/腿/遥控反馈，经[feedback_snapshot.c](../imcalib/task/feedback_snapshot.c)发布。
4. [task_actuation.c](../imcalib/task/task_actuation.c)：取快照、选择LQR/RL/补偿台架、求力矩、最终检查许可并发送。这里是唯一策略仲裁和力矩输出出口。
5. [task_policy.c](../imcalib/task/task_policy.c)：仅RL使用，准备观测/历史、推理、发布动作；执行任务再把动作转成力矩。
6. [background_service.c](../imcalib/task/background_service.c)：低优先级调用VOFA与灯效，USB初始化仍在默认任务入口。

| 路径 | 主要数据流 |
| --- | --- |
| LQR平衡 | IMU/腿/轮反馈 → lqr_balance状态 → slip融合速度 → LQR状态反馈 → leg_balance长度/roll/支撑力 → 公共补偿 → 执行出口 |
| 离地/落地 | 同拍反馈快照 → air_detection估力与判据 → task_actuation选择AIR/释放或有效落地 → 腿力控暂停/恢复与一次目标复位 |
| 自起/自救 | standup阶段判定 → 按阶段生成目标 → 独立PID → 公共映射/补偿 → PREPARE/BALANCE/STOP路由 |
| RL | 反馈/指令 → 训练角色观测 → 历史 → CubeAI推理 → 动作发布 → rl_torque → 执行出口 |
| 通信 | 硬件中断 → BSP取帧/字节 → DM/DJI/DR16/HI229协议驱动 → 任务更新/发布物理反馈 |
| 观测 | 共享状态只读快照 → vofa_debug页布局 → Vofa_Send打包 → 所选UART DMA |

## 离地功能怎么读

先看[air_detection.h](../imcalib/Algorithm/air_detection.h)的字段注释，再按[air_detection.c](../imcalib/Algorithm/air_detection.c)的Air_Detection_Update → Air_Estimate → Air_State_Update阅读。armed是首次支撑确认，valid是观测有效，flight是接触状态，三个标志不能互相替代；输入无效不会自动清flight，新反馈才推进连续条件时间。

随后看执行层Control_Air_Update的有效落地事件与Leg_Balance_Compute的释放分支：检测只发布状态，执行层才决定控制切换。最终腿长目标leg_len_tgt、轨迹斜坡输出/长度PID的set和实测腿长是三个不同量。新增每拍实测起点方案已彻底删除，当前只有原轨迹斜坡；页4的通道37保留为固定0。

原斜坡首次对齐实测，之后限制给定轨迹的变化速率，并不保证实际腿长以该速率运动。落地当前仍直接恢复完整地面反馈、轮速修正、支撑前馈和弹簧抵消；注释补充不等于本轮已修复落地翻倒。观测下标见[vofa-debug.md](vofa-debug.md)。

## 文件逐项索引

同名.c与.h分别承担实现和数据/接口声明；只有类型或内联函数的.h不需要独立.c。表中每个文件单独列出，避免把历史候选表、固件部署表和公共增益绑定入口混淆。

### imcalib/Algorithm

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/Algorithm/air_detection.c](../imcalib/Algorithm/air_detection.c) | 离地/落地状态估计。发布支持力、armed/valid/flight及压缩诊断，执行层决定是否切换控制。新反馈才推进计时；输入无效保留flight并清连续计时；不直接改腿长目标或电机输出。 |
| [imcalib/Algorithm/air_detection.h](../imcalib/Algorithm/air_detection.h) | 离地/落地状态估计。发布支持力、armed/valid/flight及压缩诊断，执行层决定是否切换控制。新反馈才推进计时；输入无效保留flight并清连续计时；不直接改腿长目标或电机输出。 |
| [imcalib/Algorithm/Attitude_Algorithm.c](../imcalib/Algorithm/Attitude_Algorithm.c) | 姿态归一化与竖直加速度计算。输出归一化姿态、rad角度和世界竖直加速度，供LQR/离地估计使用。本文件不解析HI229字节；两机竖直映射沿用源码，不在此重新标定符号。 |
| [imcalib/Algorithm/Attitude_Algorithm.h](../imcalib/Algorithm/Attitude_Algorithm.h) | 姿态归一化与竖直加速度计算。输出归一化姿态、rad角度和世界竖直加速度，供LQR/离地估计使用。本文件不解析HI229字节；两机竖直映射沿用源码，不在此重新标定符号。 |
| [imcalib/Algorithm/gas_spring.c](../imcalib/Algorithm/gas_spring.c) | 气弹簧伸腿力模型与电机侧抵消。估计实体弹簧伸腿力；将抵消力转换为四髋补偿力矩。Gas_Spring_Force是检测用物理模型值；Gas_Spring_Apply是命令补偿，两者含义不同。 |
| [imcalib/Algorithm/gas_spring.h](../imcalib/Algorithm/gas_spring.h) | 气弹簧伸腿力模型与电机侧抵消。估计实体弹簧伸腿力；将抵消力转换为四髋补偿力矩。Gas_Spring_Force是检测用物理模型值；Gas_Spring_Apply是命令补偿，两者含义不同。 |
| [imcalib/Algorithm/gravity_comp.c](../imcalib/Algorithm/gravity_comp.c) | 整条腿自重模型查询与摆矩补偿。插值质心半径/偏角，计算重力摆矩并经五连杆映射到电机。只负责重力，不叠加弹簧或最终限幅；超出表域夹到边界并记录clamped。 |
| [imcalib/Algorithm/gravity_comp.h](../imcalib/Algorithm/gravity_comp.h) | 整条腿自重模型查询与摆矩补偿。插值质心半径/偏角，计算重力摆矩并经五连杆映射到电机。只负责重力，不叠加弹簧或最终限幅；超出表域夹到边界并记录clamped。 |
| [imcalib/Algorithm/imu_state.h](../imcalib/Algorithm/imu_state.h) | 公共IMU状态数据结构。向控制快照提供四元数、欧拉角、角速度、比力及pitch_world有效性。欧拉角同时保存deg/rad；pitch_world是倒置判别补角，不能混同原LQR pitch。 |
| [imcalib/Algorithm/leg_balance.c](../imcalib/Algorithm/leg_balance.c) | LQR腿长/横滚PID及腿部输出组合。生成左右轴向力/摆矩，经公共补偿与限幅后填torque_output_t。AIR释放时轴向主动出力为零；非释放时用原轨迹斜坡；只求解，不发送CAN。 |
| [imcalib/Algorithm/leg_balance.h](../imcalib/Algorithm/leg_balance.h) | LQR腿长/横滚PID及腿部输出组合。生成左右轴向力/摆矩，经公共补偿与限幅后填torque_output_t。AIR释放时轴向主动出力为零；非释放时用原轨迹斜坡；只求解，不发送CAN。 |
| [imcalib/Algorithm/leg_compensation.c](../imcalib/Algorithm/leg_compensation.c) | 弹簧和完整腿自重补偿的组合入口。先加弹簧抵消，再加重力摆矩，最后逐电机限幅并发布补偿诊断。spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。 |
| [imcalib/Algorithm/leg_compensation.h](../imcalib/Algorithm/leg_compensation.h) | 弹簧和完整腿自重补偿的组合入口。先加弹簧抵消，再加重力摆矩，最后逐电机限幅并发布补偿诊断。spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。 |
| [imcalib/Algorithm/leg_solver.c](../imcalib/Algorithm/leg_solver.c) | 五连杆运动学、速度和力映射。求虚拟腿长/腿角、关节速度、雅可比，并在轴向力/摆矩和两髋力矩间转换。几何有效与力映射有效分开；奇异/非有限输入拒绝映射；实体极性和零点在驱动/配置层处理。 |
| [imcalib/Algorithm/leg_solver.h](../imcalib/Algorithm/leg_solver.h) | 五连杆运动学、速度和力映射。求虚拟腿长/腿角、关节速度、雅可比，并在轴向力/摆矩和两髋力矩间转换。几何有效与力映射有效分开；奇异/非有限输入拒绝映射；实体极性和零点在驱动/配置层处理。 |
| [imcalib/Algorithm/lqr_balance.c](../imcalib/Algorithm/lqr_balance.c) | LQR状态构建、遥控目标和状态反馈。形成10维状态/目标；按实测腿长求K，输出双轮和左右腿摆矩。腿长PID由leg_balance负责；AIR只使用腿角/角速度四列、轮矩为零；本文件不发报文。 |
| [imcalib/Algorithm/lqr_balance.h](../imcalib/Algorithm/lqr_balance.h) | LQR状态构建、遥控目标和状态反馈。形成10维状态/目标；按实测腿长求K，输出双轮和左右腿摆矩。腿长PID由leg_balance负责；AIR只使用腿角/角速度四列、轮矩为零；本文件不发报文。 |
| [imcalib/Algorithm/lqr_gain_big.c](../imcalib/Algorithm/lqr_gain_big.c) | 大机器生成的LQR增益多项式。提供大机器lqr_gain_desc_t及K(lL,lR)求值函数。生成物：调参入口在tools/matlab/run_all.m；注释补充不改变任何系数或机器标识。 |
| [imcalib/Algorithm/lqr_gain_small.c](../imcalib/Algorithm/lqr_gain_small.c) | 小机器生成的LQR增益多项式。提供小机器lqr_gain_desc_t及K(lL,lR)求值函数。状态序/输出序以本文件生成头和公共描述结构为准；不能当作大机器表。 |
| [imcalib/Algorithm/lqr_gain_table.c](../imcalib/Algorithm/lqr_gain_table.c) | 两机LQR增益表选择、兼容校验与求值。校验周期/几何/状态约定/工作域，再求40个状态优先排列的K系数。不能把某机K用于另一机；只夹增益求值域，不把实测腿长改成边界值。 |
| [imcalib/Algorithm/lqr_gain_table.h](../imcalib/Algorithm/lqr_gain_table.h) | 两机LQR增益表选择、兼容校验与求值。校验周期/几何/状态约定/工作域，再求40个状态优先排列的K系数。不能把某机K用于另一机；只夹增益求值域，不把实测腿长改成边界值。 |
| [imcalib/Algorithm/rl_observation.c](../imcalib/Algorithm/rl_observation.c) | 训练侧观测构建和历史队列。构建25维观测、5帧125维历史；完成实体角色与训练角色互逆转换。缩放、裁剪、默认角和历史顺序需与训练一致；这里不推理、不选策略、不输出电机。 |
| [imcalib/Algorithm/rl_observation.h](../imcalib/Algorithm/rl_observation.h) | 训练侧观测构建和历史队列。构建25维观测、5帧125维历史；完成实体角色与训练角色互逆转换。缩放、裁剪、默认角和历史顺序需与训练一致；这里不推理、不选策略、不输出电机。 |
| [imcalib/Algorithm/rl_policy.c](../imcalib/Algorithm/rl_policy.c) | CubeAI networkzn1模型生命周期与推理。管理网络句柄/工作区，输出6维动作和运行有效性/耗时。仅封装网络调用；遥控预热、动作发布与电机执行分别由任务层/rl_torque负责。 |
| [imcalib/Algorithm/rl_policy.h](../imcalib/Algorithm/rl_policy.h) | CubeAI networkzn1模型生命周期与推理。管理网络句柄/工作区，输出6维动作和运行有效性/耗时。仅封装网络调用；遥控预热、动作发布与电机执行分别由任务层/rl_torque负责。 |
| [imcalib/Algorithm/rl_torque.c](../imcalib/Algorithm/rl_torque.c) | RL动作到虚拟关节及实体电机力矩。构造关节目标、计算控制力矩、映射髋电机并补弹簧/限幅，输出torque_output_t。动作是训练角色量而非直接CAN值；轮组使用本路径自己的速度控制，不套用LQR输出。 |
| [imcalib/Algorithm/rl_torque.h](../imcalib/Algorithm/rl_torque.h) | RL动作到虚拟关节及实体电机力矩。构造关节目标、计算控制力矩、映射髋电机并补弹簧/限幅，输出torque_output_t。动作是训练角色量而非直接CAN值；轮组使用本路径自己的速度控制，不套用LQR输出。 |
| [imcalib/Algorithm/slip.c](../imcalib/Algorithm/slip.c) | 带加速度状态的速度融合与异常轮速降权。维护速度/加速度及协方差；AIR停用轮速修正但保留预测/IMU修正。不发布独立打滑状态；降权不是主动轮速制动，接触标志由执行层传入。 |
| [imcalib/Algorithm/slip.h](../imcalib/Algorithm/slip.h) | 带加速度状态的速度融合与异常轮速降权。维护速度/加速度及协方差；AIR停用轮速修正但保留预测/IMU修正。不发布独立打滑状态；降权不是主动轮速制动，接触标志由执行层传入。 |
| [imcalib/Algorithm/standup.c](../imcalib/Algorithm/standup.c) | 自起准备与倒置自救状态机。按阶段设置长度/摆角目标，运行独立PID，输出准备力矩及BALANCE/PREPARE/STOP路由。阶段切换、目标生成、PID和映射分层；DONE才交给LQR；轮组准备输出保持零。 |
| [imcalib/Algorithm/standup.h](../imcalib/Algorithm/standup.h) | 自起准备与倒置自救状态机。按阶段设置长度/摆角目标，运行独立PID，输出准备力矩及BALANCE/PREPARE/STOP路由。阶段切换、目标生成、PID和映射分层；DONE才交给LQR；轮组准备输出保持零。 |
| [imcalib/Algorithm/torque_output.h](../imcalib/Algorithm/torque_output.h) | 策略无关的统一力矩结果。保存四髋/双轮力矩及valid，交唯一执行出口检查和发送。valid只说明求解结果可用；驱动发送结果和实体执行成功是另外的状态。 |

### imcalib/task

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/task/background_service.c](../imcalib/task/background_service.c) | 默认任务后台入口。调用普通VOFA发送与WS2812灯效。通信/控制计算不放在此低优先级后台；USB初始化仍由defaultTask入口执行。 |
| [imcalib/task/feedback_snapshot.c](../imcalib/task/feedback_snapshot.c) | 跨任务反馈发布/读取与元数据。临界区发布，执行/策略任务一次读取结构与接收/发布时间戳。发布时刻不等于传感器采样或CAN接收时刻；sequence表示软件发布，旧样本不能伪装成新反馈。 |
| [imcalib/task/inc/feedback_snapshot.h](../imcalib/task/inc/feedback_snapshot.h) | 跨任务反馈发布/读取与元数据。临界区发布，执行/策略任务一次读取结构与接收/发布时间戳。发布时刻不等于传感器采样或CAN接收时刻；sequence表示软件发布，旧样本不能伪装成新反馈。 |
| [imcalib/task/inc/rc_command.h](../imcalib/task/inc/rc_command.h) | 遥控死区和统一归一化指令。commTask每拍生成唯一rc_command，供执行和策略读取。这里只处理通道量；拨杆模式语义在task_actuation；长度通道表示调节输入，不是实测腿长。 |
| [imcalib/task/inc/robot_control.h](../imcalib/task/inc/robot_control.h) | 共享控制状态定义与全机控制初始化。初始化几何/LQR/RL/自起/离地与信号量，提供动作清零和模型切换。共享变量在这里定义；策略仲裁和发送在task_actuation；不在初始化中重标定物理量。 |
| [imcalib/task/inc/robot_state.h](../imcalib/task/inc/robot_state.h) | 跨任务共享状态的声明集合。给各任务提供统一extern声明、动作/机器人/输入状态类型。该头包含共享对象，算法层应接收参数而不是包含这个任务头；同帧读写使用快照接口。 |
| [imcalib/task/inc/vofa_debug.h](../imcalib/task/inc/vofa_debug.h) | 普通VOFA状态快照与39通道打包。按air_param.vofa_page构造JustFloat帧，按发送周期/忙状态提交传输。只读观测，不推进检测或修改目标；异步来源同帧不等于同一物理采样时刻，通道表见md/vofa-debug.md。 |
| [imcalib/task/pitch_world.c](../imcalib/task/pitch_world.c) | 利用重力方向区分正常/倒置的俯仰补角。返回[-pi,pi)内pitch_world及成功标志，供自起/跌倒相关处理。先按最大分量缩放再归一化；重力z决定补角；不重写LQR原pitch或IMU零点。 |
| [imcalib/task/pitch_world.h](../imcalib/task/pitch_world.h) | 利用重力方向区分正常/倒置的俯仰补角。返回[-pi,pi)内pitch_world及成功标志，供自起/跌倒相关处理。先按最大分量缩放再归一化；重力z决定补角；不重写LQR原pitch或IMU零点。 |
| [imcalib/task/rc_command.c](../imcalib/task/rc_command.c) | 遥控死区和统一归一化指令。commTask每拍生成唯一rc_command，供执行和策略读取。这里只处理通道量；拨杆模式语义在task_actuation；长度通道表示调节输入，不是实测腿长。 |
| [imcalib/task/robot_control.c](../imcalib/task/robot_control.c) | 共享控制状态定义与全机控制初始化。初始化几何/LQR/RL/自起/离地与信号量，提供动作清零和模型切换。共享变量在这里定义；策略仲裁和发送在task_actuation；不在初始化中重标定物理量。 |
| [imcalib/task/task_actuation.c](../imcalib/task/task_actuation.c) | 策略仲裁、控制求解及唯一电机输出出口。选择补偿台架/LQR自起/RL/失能，统一门控后发送四髋/双轮力矩。离地检测只发布状态，本文件决定释放/有效落地；零输出/故障在最终出口仍需复查。 |
| [imcalib/task/task_comm.c](../imcalib/task/task_comm.c) | 通信刷新、腿解算、故障/跌倒/使能管理。更新物理电机量、解算双腿、发布遥控/电机反馈并维护许可。协议解析在驱动，运动学在leg_solver；本任务管理许可，不承担策略力矩求解。 |
| [imcalib/task/task_imu.c](../imcalib/task/task_imu.c) | IMU任务的单周期测量处理。去重、取轴/乘符号、单位换算、姿态归一化/补角后发布IMU反馈。断流发布online=0和补角无效；不在这里计算电机力矩或替控制器改姿态。 |
| [imcalib/task/task_policy.c](../imcalib/task/task_policy.c) | RL观测准备、预热、推理与动作发布。构建训练角色观测/历史，按策略节拍推理并发布可执行动作。无效/未就绪时清动作；只发布动作，不直接发送DM/DJI报文。 |
| [imcalib/task/vofa_debug.c](../imcalib/task/vofa_debug.c) | 普通VOFA状态快照与39通道打包。按air_param.vofa_page构造JustFloat帧，按发送周期/忙状态提交传输。只读观测，不推进检测或修改目标；异步来源同帧不等于同一物理采样时刻，通道表见md/vofa-debug.md。 |

### imcalib/bsp

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/bsp/BMI088Middleware.c](../imcalib/bsp/BMI088Middleware.c) | BMI088的板级SPI/GPIO与延时适配。经HAL实现传感器访问的硬件操作。只是底层适配；当前主初始化未启用BMI088主测量链，运行姿态来源以HI229为准。 |
| [imcalib/bsp/BMI088Middleware.h](../imcalib/bsp/BMI088Middleware.h) | BMI088的板级SPI/GPIO与延时适配。经HAL实现传感器访问的硬件操作。只是底层适配；当前主初始化未启用BMI088主测量链，运行姿态来源以HI229为准。 |
| [imcalib/bsp/can_bus.c](../imcalib/bsp/can_bus.c) | FDCAN公共注册、接收分发与发送服务。HAL中断读取帧并派发协议回调；统一构造发送帧、统计接收和错误。本层不解释DM/DJI物理协议或策略意图；HAL_OK仅表明发送入队调用成功。 |
| [imcalib/bsp/can_bus.h](../imcalib/bsp/can_bus.h) | FDCAN公共注册、接收分发与发送服务。HAL中断读取帧并派发协议回调；统一构造发送帧、统计接收和错误。本层不解释DM/DJI物理协议或策略意图；HAL_OK仅表明发送入队调用成功。 |
| [imcalib/bsp/dma_cache.h](../imcalib/bsp/dma_cache.h) | M7 DMA收发缓存一致性辅助接口。发送前clean，接收读取前invalidate，统一处理地址对齐。仅保证CPU cache与DMA内存可见性，不负责缓冲所有权、协议解析或任务同步。 |
| [imcalib/bsp/mono_ns.c](../imcalib/bsp/mono_ns.c) | DWT周期扩展的单调纳秒时间。给反馈接收/发布、观测采集和时间预算提供64位单调时刻。读取/扩展保留原中断屏蔽状态；这是上电相对时间，不是墙上日期，复位后从头计数。 |
| [imcalib/bsp/mono_ns.h](../imcalib/bsp/mono_ns.h) | DWT周期扩展的单调纳秒时间。给反馈接收/发布、观测采集和时间预算提供64位单调时刻。读取/扩展保留原中断屏蔽状态；这是上电相对时间，不是墙上日期，复位后从头计数。 |
| [imcalib/bsp/uart_idle.c](../imcalib/bsp/uart_idle.c) | 串口DMA环形接收与IDLE缓存交接。复制本次新字节及时间戳，由任务侧Take后交给协议解析。ISR负责取新字节与发布缓存，不做控制求解；跨环尾分两段复制，M7缓存由dma_cache维护。 |
| [imcalib/bsp/uart_idle.h](../imcalib/bsp/uart_idle.h) | 串口DMA环形接收与IDLE缓存交接。复制本次新字节及时间戳，由任务侧Take后交给协议解析。ISR负责取新字节与发布缓存，不做控制求解；跨环尾分两段复制，M7缓存由dma_cache维护。 |
| [imcalib/bsp/Vofa_send.c](../imcalib/bsp/Vofa_send.c) | 串口JustFloat发送实现。按MACHINE_VOFA_PORT选UART，保留DMA忙保护/静态缓冲/cache维护；不再提供UART/USB切换状态。 |
| [imcalib/bsp/Vofa_send.h](../imcalib/bsp/Vofa_send.h) | 串口JustFloat发送接口。按MACHINE_VOFA_PORT选UART，保留DMA忙保护/静态缓冲/cache维护；不再提供UART/USB切换状态。 |

### imcalib/drivers

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/drivers/BMI088driver.c](../imcalib/drivers/BMI088driver.c) | BMI088加速度/陀螺仪寄存器驱动。初始化芯片并读取原驱动约定的加速度、角速度和温度。保留原寄存器量程/单位约定；是否启用由main初始化决定，不替代HI229通信任务。 |
| [imcalib/drivers/BMI088driver.h](../imcalib/drivers/BMI088driver.h) | BMI088加速度/陀螺仪寄存器驱动。初始化芯片并读取原驱动约定的加速度、角速度和温度。保留原寄存器量程/单位约定；是否启用由main初始化决定，不替代HI229通信任务。 |
| [imcalib/drivers/BMI088reg.h](../imcalib/drivers/BMI088reg.h) | BMI088寄存器地址与位定义。给BMI088driver提供ID、配置/数据寄存器和位掩码。本文件是常量表，无任务或控制流程；量程位与换算需作为一个整体核对。 |
| [imcalib/drivers/dji.c](../imcalib/drivers/dji.c) | DJI轮电机反馈与力矩到电流发送。解析连续角和轮速；换算力矩对应raw电流，按反馈ID槽位发送。反馈极性与发送极性分别适用；零力矩是零电流指令，不代表轮子立即停止。 |
| [imcalib/drivers/dji.h](../imcalib/drivers/dji.h) | DJI轮电机反馈与力矩到电流发送。解析连续角和轮速；换算力矩对应raw电流，按反馈ID槽位发送。反馈极性与发送极性分别适用；零力矩是零电流指令，不代表轮子立即停止。 |
| [imcalib/drivers/dm.c](../imcalib/drivers/dm.c) | 达妙髋电机MIT协议与状态/使能管理。解析物理角/速/矩和接收时间，维护在线/故障/使能，量化MIT命令并发送。所有实体极性/零点在驱动边界统一处理；控制层传物理量，不再重复乘符号。 |
| [imcalib/drivers/dm.h](../imcalib/drivers/dm.h) | 达妙髋电机MIT协议与状态/使能管理。解析物理角/速/矩和接收时间，维护在线/故障/使能，量化MIT命令并发送。所有实体极性/零点在驱动边界统一处理；控制层传物理量，不再重复乘符号。 |
| [imcalib/drivers/dr16.c](../imcalib/drivers/dr16.c) | DR16遥控协议校验与在线管理。解析通道/拨杆/鼠标键盘、缓存有效帧并提供协议快照/在线状态。驱动不决定LQR/RL模式，也不直接控制电机；统一指令转换由rc_command处理。 |
| [imcalib/drivers/dr16.h](../imcalib/drivers/dr16.h) | DR16遥控协议校验与在线管理。解析通道/拨杆/鼠标键盘、缓存有效帧并提供协议快照/在线状态。驱动不决定LQR/RL模式，也不直接控制电机；统一指令转换由rc_command处理。 |
| [imcalib/drivers/hi229.c](../imcalib/drivers/hi229.c) | HI229串口姿态协议与CRC校验。解码四元数/欧拉角/角速度/比力，保存协议样本和在线期限。此层保留模块输出，机器取轴/符号和单位换算在task_imu，勿重复处理。 |
| [imcalib/drivers/hi229.h](../imcalib/drivers/hi229.h) | HI229串口姿态协议与CRC校验。解码四元数/欧拉角/角速度/比力，保存协议样本和在线期限。此层保留模块输出，机器取轴/符号和单位换算在task_imu，勿重复处理。 |
| [imcalib/drivers/ws2812.c](../imcalib/drivers/ws2812.c) | SPI编码的WS2812灯效发送。编码GRB为SPI波形并显示彩虹闪烁。SPI忙或预算不足时返回；灯效在后台而非commTask中运行，不承担故障许可决策。 |
| [imcalib/drivers/ws2812.h](../imcalib/drivers/ws2812.h) | SPI编码的WS2812灯效发送。编码GRB为SPI波形并显示彩虹闪烁。SPI忙或预算不足时返回；灯效在后台而非commTask中运行，不承担故障许可决策。 |

### imcalib/common

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/common/arm_cos_f32.c](../imcalib/common/arm_cos_f32.c) | CMSIS单精度余弦查表实现。以相位偏移/插值求cos值，供运动学调用。与arm_sin_table_f32共用查表数据，不另挂整套DSP静态库。 |
| [imcalib/common/arm_sin_f32.c](../imcalib/common/arm_sin_f32.c) | CMSIS单精度正弦查表实现。输出插值sin值，供运动学快速三角计算。移植的DSP源码；保留原许可/插值算法，LEG_TRIG_LIBM可在几何层切换数学实现。 |
| [imcalib/common/arm_sin_table_f32.c](../imcalib/common/arm_sin_table_f32.c) | CMSIS三角函数公共查表数据。提供arm_sin_f32/arm_cos_f32所需浮点常量。只截取项目实际需要的表，避免把整个CMSIS常量表带入固件；不改采样值。 |
| [imcalib/common/kalman.c](../imcalib/common/kalman.c) | 带加速度预测的一维速度卡尔曼。预测速度并执行观测修正，返回估计值。这是保留的通用标量接口；当前执行层接触门控速度融合使用slip.c的两状态实现。 |
| [imcalib/common/kalman.h](../imcalib/common/kalman.h) | 带加速度预测的一维速度卡尔曼。预测速度并执行观测修正，返回估计值。这是保留的通用标量接口；当前执行层接触门控速度融合使用slip.c的两状态实现。 |
| [imcalib/common/math_utils.h](../imcalib/common/math_utils.h) | 公共有限性检查等轻量数学辅助。供RL/控制路径验证NaN/Inf等无效量。这里是无HAL/RTOS依赖的纯辅助接口，不维护策略或设备状态。 |
| [imcalib/common/motor_types.h](../imcalib/common/motor_types.h) | 公共电机反馈类型与电机枚举。为任务快照/算法提供统一数据布局。电机枚举决定数组顺序；物理量已在驱动边界处理标定，算法不能再重复反向。 |
| [imcalib/common/pid.c](../imcalib/common/pid.c) | 位置式/增量式PID、误差历史与前馈。更新P/I/D、误差历史和输出，提供首拍历史预置。位置式D沿用误差逐拍差分，未除dt；改给定会影响D，首拍预置只消除初始化突变。 |
| [imcalib/common/pid.h](../imcalib/common/pid.h) | 位置式/增量式PID、误差历史与前馈。更新P/I/D、误差历史和输出，提供首拍历史预置。位置式D沿用误差逐拍差分，未除dt；改给定会影响D，首拍预置只消除初始化突变。 |
| [imcalib/common/rc_types.h](../imcalib/common/rc_types.h) | DR16原始数据与统一遥控指令类型。声明dr16_t、rc_command_t及拨杆原始取值。原始拨杆编码不等于策略编号；vel/yaw指令单位与len调节输入需要分清。 |
| [imcalib/common/rl_types.h](../imcalib/common/rl_types.h) | RL模型编号、动作大小等公共定义。供观测/推理/力矩模块共享动作维度和模型类型。纯类型头，不包含网络运行库、任务共享状态或策略选择逻辑。 |
| [imcalib/common/simple-function.c](../imcalib/common/simple-function.c) | 一阶低通与原目标轨迹斜坡。返回滤波量或逐拍受速率限制的给定值。Ramp_Target_Update仅第一次从实测起步，之后沿用上拍输出；控制刚度不能用每拍重置起点替代。 |
| [imcalib/common/simple-function.h](../imcalib/common/simple-function.h) | 一阶低通与原目标轨迹斜坡。返回滤波量或逐拍受速率限制的给定值。Ramp_Target_Update仅第一次从实测起步，之后沿用上拍输出；控制刚度不能用每拍重置起点替代。 |

### imcalib/config

| 文件 | 职责与流向 |
| --- | --- |
| [imcalib/config/machine_config.c](../imcalib/config/machine_config.c) | 两机参数、标定与编译选择。向驱动/几何/IMU/RL/LQR提供同一只读machine配置和节拍宏。极性/零点/量程/几何只能按作者实测修改；当前FDCAN时序以Core/Src/fdcan.c实际初始化为准。 |
| [imcalib/config/machine_config.h](../imcalib/config/machine_config.h) | 两机参数、标定与编译选择。向驱动/几何/IMU/RL/LQR提供同一只读machine配置和节拍宏。极性/零点/量程/几何只能按作者实测修改；当前FDCAN时序以Core/Src/fdcan.c实际初始化为准。 |

### Core

| 文件 | 职责与流向 |
| --- | --- |
| [Core/Inc/dma.h](../Core/Inc/dma.h) | DMA时钟与中断初始化。配置全局DMA相关时钟/IRQ，具体通道归各外设MSP初始化。DMA数据缓存一致性另由imcalib/bsp/dma_cache.h处理。 |
| [Core/Inc/fdcan.h](../Core/Inc/fdcan.h) | 三路FDCAN硬件初始化/MSP。设置仲裁/数据时序、滤波资源、引脚与中断。实际CAN时序在本文件；协议/反馈ID分发在can_bus与DM/DJI驱动。 |
| [Core/Inc/FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h) | FreeRTOS内核编译配置。定义调度、时钟、堆、优先级和钩子选项。这是编译配置，不是运行时参数表；用户说明仅写在USER CODE区。 |
| [Core/Inc/gpio.h](../Core/Inc/gpio.h) | 板级GPIO初始状态与模式。初始化片选、控制输出、输入等GPIO硬件。它只设置硬件，不判断电机使能或机器人策略。 |
| [Core/Inc/main.h](../Core/Inc/main.h) | 上电初始化与主入口。初始化通信/单调时钟/控制模块，创建RTOS并启动调度。USB DMA存储和MPU适配在用户区；FDCAN附加时序套用目前未调用，实际参数看fdcan.c。 |
| [Core/Inc/spi.h](../Core/Inc/spi.h) | SPI外设及GPIO/DMA初始化。提供SPI句柄、片选相关引脚/MSP和外设初始化。BMI088访问和WS2812波形发送由各驱动完成，不能把协议解析放在此文件。 |
| [Core/Inc/stm32h7xx_hal_conf.h](../Core/Inc/stm32h7xx_hal_conf.h) | STM32 HAL模块和系统常量配置。决定HAL模块、回调开关、时钟常量及断言配置。宏定义在编译时生效；不在此放机器人控制参数。 |
| [Core/Inc/stm32h7xx_it.h](../Core/Inc/stm32h7xx_it.h) | CPU异常与外设中断入口。转交HAL DMA/FDCAN/UART/USB/TIM处理，UART用户区通知IDLE接收。异常入口和外设中断不是RTOS任务；ISR中不运行完整控制器。 |
| [Core/Inc/tim.h](../Core/Inc/tim.h) | 定时器/PWM及控制节拍硬件配置。创建TIM句柄、时基/PWM和相关中断资源。控制/策略节拍回调在main用户区，HAL系统tick相关配置另看timebase文件。 |
| [Core/Inc/usart.h](../Core/Inc/usart.h) | 串口硬件、DMA与printf重定向。初始化DR16/HI229/VOFA所用串口与DMA/引脚。IDLE新字节获取在uart_idle，协议解析在drivers；这里不重复解析。 |
| [Core/Src/dma.c](../Core/Src/dma.c) | DMA时钟与中断初始化。配置全局DMA相关时钟/IRQ，具体通道归各外设MSP初始化。DMA数据缓存一致性另由imcalib/bsp/dma_cache.h处理。 |
| [Core/Src/fdcan.c](../Core/Src/fdcan.c) | 三路FDCAN硬件初始化/MSP。设置仲裁/数据时序、滤波资源、引脚与中断。实际CAN时序在本文件；协议/反馈ID分发在can_bus与DM/DJI驱动。 |
| [Core/Src/freertos.c](../Core/Src/freertos.c) | RTOS任务创建与循环节拍。创建default/comm/imu/policy/actuation任务并调用各单周期函数。本文件只管任务入口/等待/节拍；不要把协议、控制器或报文逻辑放进循环。 |
| [Core/Src/gpio.c](../Core/Src/gpio.c) | 板级GPIO初始状态与模式。初始化片选、控制输出、输入等GPIO硬件。它只设置硬件，不判断电机使能或机器人策略。 |
| [Core/Src/main.c](../Core/Src/main.c) | 上电初始化与主入口。初始化通信/单调时钟/控制模块，创建RTOS并启动调度。USB DMA存储和MPU适配在用户区；FDCAN附加时序套用目前未调用，实际参数看fdcan.c。 |
| [Core/Src/spi.c](../Core/Src/spi.c) | SPI外设及GPIO/DMA初始化。提供SPI句柄、片选相关引脚/MSP和外设初始化。BMI088访问和WS2812波形发送由各驱动完成，不能把协议解析放在此文件。 |
| [Core/Src/stm32h7xx_hal_msp.c](../Core/Src/stm32h7xx_hal_msp.c) | HAL通用板级MSP初始化。设置系统相关时钟/中断等底层资源。各外设MSP细节分别在fdcan/spi/tim/usart中，本文件不是通信协议层。 |
| [Core/Src/stm32h7xx_hal_timebase_tim.c](../Core/Src/stm32h7xx_hal_timebase_tim.c) | HAL毫秒时基的定时器适配。提供HAL_GetTick所依赖的tick更新与时基启停。HAL毫秒超时与Mono_Ns纳秒时间用途不同，不能互相当作传感器采样时刻。 |
| [Core/Src/stm32h7xx_it.c](../Core/Src/stm32h7xx_it.c) | CPU异常与外设中断入口。转交HAL DMA/FDCAN/UART/USB/TIM处理，UART用户区通知IDLE接收。异常入口和外设中断不是RTOS任务；ISR中不运行完整控制器。 |
| [Core/Src/system_stm32h7xx.c](../Core/Src/system_stm32h7xx.c) | CMSIS系统启动和SystemCoreClock维护。初始化内核基础环境并维护系统时钟查询。厂商启动支持代码，详细用途在文件索引说明，不改生成/厂商实现。 |
| [Core/Src/tim.c](../Core/Src/tim.c) | 定时器/PWM及控制节拍硬件配置。创建TIM句柄、时基/PWM和相关中断资源。控制/策略节拍回调在main用户区，HAL系统tick相关配置另看timebase文件。 |
| [Core/Src/usart.c](../Core/Src/usart.c) | 串口硬件、DMA与printf重定向。初始化DR16/HI229/VOFA所用串口与DMA/引脚。IDLE新字节获取在uart_idle，协议解析在drivers；这里不重复解析。 |
| [Core/Startup/startup_stm32h723xx.s](../Core/Startup/startup_stm32h723xx.s) | STM32H723复位向量与启动汇编。设置启动入口、链接段初始化并进入C运行环境。不同工具链有各自启动文件；不把板上控制逻辑放在向量表。 |

### USB_DEVICE

| 文件 | 职责与流向 |
| --- | --- |
| [USB_DEVICE/App/usb_device.c](../USB_DEVICE/App/usb_device.c) | USB设备栈启动入口。注册描述符/CDC接口并启动USB设备。这里只初始化USB，不发送电机命令；VOFA是否选USB由传输层按失能/空闲许可决定。 |
| [USB_DEVICE/App/usb_device.h](../USB_DEVICE/App/usb_device.h) | USB设备栈启动入口。注册描述符/CDC接口并启动USB设备。这里只初始化USB，不发送电机命令；VOFA是否选USB由传输层按失能/空闲许可决定。 |
| [USB_DEVICE/App/usbd_cdc_if.c](../USB_DEVICE/App/usbd_cdc_if.c) | USB CDC应用接口与发送缓冲状态。管理CDC初始化/接收/发送完成和发送busy/ready检查。DMA缓冲/发送完成决定能否复用；接收字节不作为电机命令解析。 |
| [USB_DEVICE/App/usbd_cdc_if.h](../USB_DEVICE/App/usbd_cdc_if.h) | USB CDC应用接口与发送缓冲状态。管理CDC初始化/接收/发送完成和发送busy/ready检查。DMA缓冲/发送完成决定能否复用；接收字节不作为电机命令解析。 |
| [USB_DEVICE/App/usbd_desc.c](../USB_DEVICE/App/usbd_desc.c) | USB设备与字符串描述符。返回VID/PID、设备/语言/产品/序列号等描述符。只影响枚举身份与描述信息，不参与机器人状态或控制策略。 |
| [USB_DEVICE/App/usbd_desc.h](../USB_DEVICE/App/usbd_desc.h) | USB设备与字符串描述符。返回VID/PID、设备/语言/产品/序列号等描述符。只影响枚举身份与描述信息，不参与机器人状态或控制策略。 |
| [USB_DEVICE/Target/usbd_conf.c](../USB_DEVICE/Target/usbd_conf.c) | USB PCD与设备栈低层适配。连接端点收发、设备地址/复位、内存配置与USB硬件。缓存/内存可见性和DMA位置要与main及CDC缓冲配套；不做VOFA通道打包。 |
| [USB_DEVICE/Target/usbd_conf.h](../USB_DEVICE/Target/usbd_conf.h) | USB PCD与设备栈低层适配。连接端点收发、设备地址/复位、内存配置与USB硬件。缓存/内存可见性和DMA位置要与main及CDC缓冲配套；不做VOFA通道打包。 |

### tools/matlab

| 文件 | 职责与流向 |
| --- | --- |
| [tools/matlab/baseline/lqr_gain_small_legacy.inc](../tools/matlab/baseline/lqr_gain_small_legacy.inc) | 小机器历史LQR表对照。被公共增益入口的小机历史对照分支包含。仅适用于显式legacy对照，不是大机器表；系数和原状态约定保持。 |
| [tools/matlab/build_gain_table.m](../tools/matlab/build_gain_table.m) | 腿长网格LQR设计、拟合、闭环检查与C表导出。扫描离散LQR，拟合K多项式，检查float32/延迟/饱和并输出报告/候选表。生成检查通过只代表对应模型数值检查通过；实机稳定性仍需台架。 |
| [tools/matlab/cache/AB_sjtu5_gen.m](../tools/matlab/cache/AB_sjtu5_gen.m) | 符号推导生成的SJTU5 A/B数值缓存。快速计算连续时间A/B，避免每次重新符号推导。自动生成的长表达式仅在索引解释，不重写模型数学表达式。 |
| [tools/matlab/machine_table.m](../tools/matlab/machine_table.m) | MATLAB机械/惯性参数表。返回动力学、几何、腿数据和采样周期配置。与固件配置需核对几何/周期；来源和待实测项目不等于已上机标定。 |
| [tools/matlab/model_AB.m](../tools/matlab/model_AB.m) | 连续时间动力学A/B统一接口。查/生成符号缓存，再转换到固件髋轴/前摆状态约定。坐标变换属于模型契约，不能据此擅自改固件电机极性、零点或轴向。 |
| [tools/matlab/output/lqr_gain_big.c](../tools/matlab/output/lqr_gain_big.c) | 大机器生成的LQR增益多项式。提供大机器lqr_gain_desc_t及K(lL,lR)求值函数。生成物：调参入口在tools/matlab/run_all.m；注释补充不改变任何系数或机器标识。 |
| [tools/matlab/output/lqr_gain_small.c](../tools/matlab/output/lqr_gain_small.c) | 小机器生成的LQR增益多项式。提供小机器lqr_gain_desc_t及K(lL,lR)求值函数。状态序/输出序以本文件生成头和公共描述结构为准；不能当作大机器表。 |
| [tools/matlab/output/lqr_gain_table.c](../tools/matlab/output/lqr_gain_table.c) | 历史单表生成结果。保留历史候选表供核对，当前双机运行入口见imcalib/Algorithm/lqr_gain_table.c。不要将此旧候选文件误当公共绑定实现或当前部署表。 |
| [tools/matlab/reference/leg3_leg_data.m](../tools/matlab/reference/leg3_leg_data.m) | 参考腿质心与惯性离散数据。给MATLAB机器表提供腿质心/惯性候选数据。参考数据的原点/单位与本机标定需区分，不将其直接写回固件物理量。 |
| [tools/matlab/run_all.m](../tools/matlab/run_all.m) | LQR生成总入口与调参入口。调用build_gain_table，生成机器候选C表与报告。日常Q/R选择在这里；机械参数在machine_table，不能用调增益替代物理标定。 |
| [tools/matlab/verify_models.m](../tools/matlab/verify_models.m) | MATLAB动力学模型一致性核对入口。执行现有模型数值/维度相关检查并报告。用于生成侧模型核对，不下载固件、不驱动机器人。 |

### tests

| 文件 | 职责与流向 |
| --- | --- |
| [tests/dm_timestamp_host_test.c](../tests/dm_timestamp_host_test.c) | 达妙反馈接收时刻的历史C宿主检查。核对解析后时间戳和状态刷新行为。不是固件任务或当前控制入口；Python运行器已删除，保留C源码作参考。 |
| [tests/fixtures/baseline_leg_balance.c](../tests/fixtures/baseline_leg_balance.c) | 历史leg_balance实现对照。保留作历史实现参考，不参与固件运行。Python测试入口已按作者要求删除；此文件不是当前控制算法，不能拿旧参数覆盖生产配置。 |
| [tests/fixtures/baseline_leg_balance.h](../tests/fixtures/baseline_leg_balance.h) | 历史leg_balance实现对照。保留作历史实现参考，不参与固件运行。Python测试入口已按作者要求删除；此文件不是当前控制算法，不能拿旧参数覆盖生产配置。 |
| [tests/fixtures/baseline_lqr_balance.c](../tests/fixtures/baseline_lqr_balance.c) | 历史lqr_balance实现对照。保留作历史实现参考，不参与固件运行。Python测试入口已按作者要求删除；此文件不是当前控制算法，不能拿旧参数覆盖生产配置。 |
| [tests/fixtures/baseline_lqr_balance.h](../tests/fixtures/baseline_lqr_balance.h) | 历史lqr_balance实现对照。保留作历史实现参考，不参与固件运行。Python测试入口已按作者要求删除；此文件不是当前控制算法，不能拿旧参数覆盖生产配置。 |

### X-CUBE-AI

| 文件 | 职责与流向 |
| --- | --- |
| [X-CUBE-AI/App/networkzn1_config.h](../X-CUBE-AI/App/networkzn1_config.h) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1_data_params.c](../X-CUBE-AI/App/networkzn1_data_params.c) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1_data_params.h](../X-CUBE-AI/App/networkzn1_data_params.h) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1_data.c](../X-CUBE-AI/App/networkzn1_data.c) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1_data.h](../X-CUBE-AI/App/networkzn1_data.h) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1.c](../X-CUBE-AI/App/networkzn1.c) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |
| [X-CUBE-AI/App/networkzn1.h](../X-CUBE-AI/App/networkzn1.h) | CubeAI模型生成物。由rl_policy加载，生成内容不在任务层复制。重新生成模型会替换此目录，模型版本/接口需配套。 |

## 生成物、工程配置与第三方目录

| 文件/目录 | 作用及维护入口 |
| --- | --- |
| CtrBoard-H7_ALL.ioc | CubeMX外设/引脚/时钟生成配置；Core与USB用户注释放在USER CODE保护区 |
| .eide/eide.yml | eIDE源码目录、包含路径与工具链配置；重载后核对生效，旧Python同步工具已删除 |
| MDK-ARM/*.uvprojx / *.uvoptx | Keil工程及本地调试选项；源路径/包含目录以工程内容为准 |
| MDK-ARM/*.sct | Keil链接内存布局，不是控制参数或编译源文件 |
| Core/Startup/*.s、MDK-ARM中的启动文件 | 对应工具链的向量表/复位入口，建立C运行环境后进入main |
| Drivers/STM32H7xx_HAL_Driver | ST HAL外设实现，由Core初始化/BSP调用，机器人协议与控制算法不放这里 |
| Drivers/CMSIS | 内核寄存器/启动支持及DSP接口；项目实际采用的少量DSP源码位于common |
| Middlewares/Third_Party/FreeRTOS | RTOS调度器、任务/队列/信号量与工具链端口 |
| Middlewares/ST/STM32_USB_Device_Library | USB设备核心与CDC类，应用层接口在USB_DEVICE |
| Middlewares/ST/AI、X-CUBE-AI | AI运行库及networkzn1图/参数产物；通过rl_policy调用，权重不是手工调参表 |
| tools/matlab/cache/*.mat、AB_sjtu5_gen.m | 模型扫描/符号计算缓存；改变来源参数后由MATLAB流程判断是否更新 |
| tools/matlab/output/*.c | 生成侧候选增益表；与imcalib/Algorithm的部署版本区分 |
| tools/matlab/output/report*.txt | 生成参数、拟合/闭环数值核验报告；不是固件实际运行日志 |
| tools/matlab/run_all.asv | MATLAB编辑器自动备份，当前调参入口是run_all.m |
| tools/matlab/baseline/*.inc、tests/fixtures | 历史对照实现/表；只有公共K入口显式包含的inc才进入固件，其余是参考源码 |
| build/、out/、MDK-ARM/Objects等 | 编译/链接/缓存产物；本轮验证对象输出在工程外临时目录 |
| build/.../compile_commands.json | 编译命令快照，供语言服务和编译核验；源码更新后应由工程工具生成 |
| .vscode、.clangd、.clang-format、*.code-workspace | 编辑器、语言服务和格式化设置；不参与实体控制算法 |
| keilkilll.bat | 本地Keil相关辅助批处理；执行前读取其中命令，不作为固件代码编译 |
| md/ | 模块契约、阅读说明、维护约束和历史变更；历史参数/验证不视为当前实机结果 |
| LICENSE_X-CUBE-AI.txt及第三方许可 | 模型运行库/移植代码许可来源，保留原版权说明 |

## 本轮维护约束

Python测试和工具文件已按作者要求删除，不再生成额外Python测试文件。后续优先使用Keil/ARM GCC编译、现有MATLAB生成侧检查和VOFA台架数据验证；不把编译通过等同于实体站立/落地稳定。文件注释说明职责/单位/状态转移，未修改物理极性、零点、量程、K系数或PID参数。

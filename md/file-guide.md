# 工程代码总览

2026-10-13按当前工作区源码整理。本文说明职责、调用关系和参数入口；数值以源码为准，编译成功不等于实机标定或控制效果已验证。

## 文档与阅读顺序

工程只保留四份Markdown：[AGENTS.md](AGENTS.md)是AI协作规则，[CLAUDE.md](CLAUDE.md)是阅读入口，本文是现行代码总览，[sysid-change-map.md](sysid-change-map.md)保存历史变更及删除前文档快照。历史中的旧参数、旧通道和工具命令不能当作现行操作依据。

先看`main.c → freertos.c → robot_control.c`了解启动，再看`task_comm/task_imu → feedback_snapshot → task_actuation`了解控制主链；处理RL时继续看`task_policy → rl_observation/rl_policy/rl_torque`。

## 目录职责

| 路径 | 职责与主要入口 |
| --- | --- |
| [Core/Src/main.c](../Core/Src/main.c) | 时钟/外设与通信初始化，启动FreeRTOS；用户改动限定在CubeMX保护区 |
| [Core/Src/freertos.c](../Core/Src/freertos.c) | 五个任务的创建、初始化调用、等待与循环节拍 |
| [imcalib/config/machine_config.c](../imcalib/config/machine_config.c) / [头文件](../imcalib/config/machine_config.h) | 两机只读配置、编译选机和节拍宏；驱动/几何/IMU/LQR/RL共用 |
| imcalib/task | 反馈准备、策略仲裁、求解调用、最终输出及后台服务 |
| imcalib/Algorithm | 姿态、几何、LQR/RL、自起、离地、速度融合与补偿；不直接发送控制报文 |
| imcalib/drivers | DR16、HI229、DM/DJI等协议解析、反馈换算、在线判定与报文发送 |
| imcalib/bsp | CAN/UART、DMA缓存、单调时间、VOFA发送及板级适配 |
| imcalib/common | PID、斜坡、低通、数学/DSP及motor/RC/RL纯类型；kalman是保留数学工具 |
| [tools/matlab/run_all.m](../tools/matlab/run_all.m) | LQR模型、Q/R选择及增益表生成入口 |
| X-CUBE-AI/App、Middlewares/ST/AI | networkzn1生成网络/权重和AI运行库；通过rl_policy封装调用 |
| Drivers、Middlewares | HAL/CMSIS、FreeRTOS、USB及厂商运行库，保留其版权和生成边界 |
| USB_DEVICE | USB设备/CDC基础实现，当前不承担VOFA传输或电机控制协议 |

## 启动与任务主链

`main`完成外设初始化后调用`FDCAN_Config`、单调时钟及遥控/IMU/电机初始化，再启动RTOS。`FDCAN_Config`会把机器宏的数据段时序套到FDCAN1/3，当前确实生效；FDCAN2仍取其初始化配置。BMI088源文件保留，主初始化未启用它，运行姿态来源为HI229。

| 任务入口 | 单周期职责 | 节拍来源 |
| --- | --- | --- |
| [comm_task_body](../imcalib/task/task_comm.c) | DM/DJI解析与物理量刷新、两腿解算、遥控解算/发布、故障/跌倒与使能看门狗 | freertos中的osDelay |
| [imu_task_body](../imcalib/task/task_imu.c) | HI229读取/去重、按机器表取轴和符号、姿态/补角、IMU发布 | freertos中的osDelay |
| [ctrl_task_body](../imcalib/task/task_policy.c) | RL观测/历史、预热、CubeAI推理、动作发布 | TIM6按MACHINE_POLICY_DIV释放策略信号量 |
| [output_task_body](../imcalib/task/task_actuation.c) | 快照与许可、速度/离地更新、模式求解、唯一力矩分发 | TIM6基础执行信号量；RL按MACHINE_RL_CTRL_DIV执行分频 |
| [background_task_body](../imcalib/task/background_service.c) | 普通VOFA与灯效；USB初始化仍在默认任务入口 | 默认任务循环及osDelay |

LQR使用`MACHINE_LQR_DT`，RL内环使用`MACHINE_RL_CTRL_DT`，策略使用`MACHINE_POLICY_DT`。公共腿长拨轮累加在通信任务的`Rc_Command_Update`完成，按`MACHINE_TICK_DT`推进。原通用周期兼容别名已移除，不能用一个周期替代各路径。大机RL间隔拍保持原已发输出，退出/故障仍在每个基础执行拍优先检查。

```mermaid
flowchart LR
    Sensors[IMU / CAN / 遥控] --> Prepare[IMU与通信任务]
    Prepare --> Snapshot[公共反馈快照]
    Snapshot --> Policy[RL观测与推理]
    Policy --> Action[动作发布]
    Snapshot --> Act[执行任务与许可]
    Action --> Act
    Act --> Solve[LQR / 自起 / RL / 补偿求解]
    Solve --> Dispatch[唯一力矩分发]
    Dispatch --> Motors[DM / DJI驱动]
```

<a id="shared-state"></a>
## 状态所有权与模块边界

[robot_control.c](../imcalib/task/robot_control.c)定义共享实例、总初始化、动作清零和模型切换；[robot_control.h](../imcalib/task/inc/robot_control.h)声明任务/许可接口，[robot_state.h](../imcalib/task/inc/robot_state.h)声明共享状态。算法不能包含这两个任务头，应接收显式参数或读取只读机器配置。

执行拍类型`control_frame_t`定义在robot_control.h，实例仍由task_actuation.c私有持有；其compensation字段保存本拍统一预计算的模型值。发布元数据集合类型`feedback_metadata_t`定义在feedback_snapshot.h，metadata实例仍由feedback_snapshot.c私有持有。工程自有结构体类型统一放入对应头文件，类型可见不代表实例跨模块共享。

| 状态 | 周期写入方 | 消费方式 |
| --- | --- | --- |
| imu_state | IMU任务 | 经公共快照供策略/执行读取 |
| motor_state、leg_l/r | 通信任务 | 电机反馈及对应双腿解算配套发布 |
| rc_command、robot_state、ctrl_fault | 通信任务；有效落地更新公共len | 执行/策略消费命令和许可，后台读取状态 |
| RL指令、观测/历史、policy、action_state | 策略任务 | 执行任务消费已发布动作；内部量由Watch查看 |
| lqr_state、leg_balance、自起/离地/速度状态、最终力矩及补偿诊断 | 执行任务 | 求解和最终输出；Watch及后台按实际代码读取 |

[feedback_snapshot.c](../imcalib/task/feedback_snapshot.c) / [头文件](../imcalib/task/inc/feedback_snapshot.h)由发布方在短临界区提交，策略和执行每周期各读取一次。成功复制不代表传感器有效，各消费者仍检查在线、几何、历史和许可；同一软件快照不代表物理采样同步。

快照中已有TODO标记的元数据/时间字段仍属作者选择保留的预留链路；当前控制消费者不使用这些字段，不能把它们当作已经接入的样本龄门控。离地已改为固定周期，DM接收时间不再传入离地输入；驱动及快照保留的时间元数据不参与其dt计算。

几何、速度和雅可比只归`leg_solver`；DR16字节解析只归驱动，遥控死区/量程与模式请求解码只归`rc_command`；网络生命周期只归`rl_policy`。实际出力许可、控制求解与力矩下发只归`task_actuation`，通信任务保留电机使能/失能/故障看门狗职责。

## 数据有效性与校验边界

当前调用关系中，控制/观测对象和数组由任务提供固定有效地址；相应固定指针防御和重复扫描已精简。SDK返回的动态缓冲区、可选模型指针、通用接口允许的空指针、几何奇异性、设备/动作失联超时、运行时参数和最终发送许可仍分别按实际语义检查。离地不再另设样本龄、四髋时间偏差或新帧门控。

| 数据 | 当前保证与消费者 |
| --- | --- |
| IMU | Attitude_Update完成四元数归一化、欧拉角换算、三路数值验证，并调用原Attitude_Accel_Vertical生成accel_vertical；竖直结果也通过后才返回成功，由task_imu发布online=1。离地直接读取缓存值，不每执行拍重复投影和校验。pitch_world_valid仍独立表示补角可用；轴向和原计算公式保持 |
| 双腿 | comm在两路源电机在线时解算，否则清输出有效标志；Leg_Solve统一验证腿长/腿角/变化率/虚拟关节量与各雅可比，成功后valid=1。force_valid另外保证力映射行列式有限且满足原奇异性阈值。LQR/自起/离地消费这两个标志，不重复扫描原始反馈 |
| 电机组 | motor.valid由通信更新时一次汇总四髋/双轮online及四髋无DM故障；故障沿用Dm_Has_Fault原判据，与反馈复制共用一次遍历。RL、执行许可和Robot_Fault_Update直接读组标志，不再另遍历DM故障；CAN异常仍单独检查，数据可用不等同于电机已使能或命令已执行 |
| 轮速/遥控 | 轮速由int16_t RPM及本机固定非零齿比换算，LQR/RL的重复轮速有限性扫描已删；RC用整数通道及固定公共量程产生有限命令，LQR目标更新只保留在线门控 |
| RL观测/动作 | 输入反馈由发布端验证；RL_Joint_Map在角色映射后验证新关节数组并返回可用状态，RL_Observation_Build消费source_valid，不再重扫陀螺仪/关节/RC命令。原投影和训练缩放/裁剪表达式保持，生成观测的结果验证后valid=1，历史由有效观测复制。推理信任valid/history_ready；动态网络缓冲区、推理返回值、输出动作仍检查 |
| 补偿 | 执行拍准备阶段使用固定本机模型和同拍几何计算实体弹簧力、抵消量及两种角度引用的重力值；模型新结果在准备端验证，消费端读取valid，保留反馈反解和叠加新结果检查。模型、坐标、符号及原叠加顺序保持 |

LQR_State_Update已删除无计算用途的dt参数，只检查imu.online/左右腿valid后进行状态构建；速度融合、位置积分、PID/斜坡等实际使用周期的函数仍传各自周期。执行入口读取本拍IMU/补角/双腿/电机组可用标志后形成drive，避免仅等待缓存故障汇总。自起角度范围、运行参数、新计算结果、最终出力许可仍保留必要检查。纯计算助手只在调用方确认反馈可用后使用，不能把无效数据强行标为有效。

2026-10-13再次按作者要求检查当前全部自有源码调用：固定对象/数组不重复检查NULL；Slip消费LQR已验证的运动学速度/加速度和固定周期，LQR速度回填消费Slip.valid，不再逐层检查同一速度。斜坡实测量由几何保证、周期由任务保证，目标/速率及新斜坡结果仍检查。自起保留入口反馈标志与阶段/许可，删掉后续重复补角检查；补偿台架信任normal已包含的补角许可。Rc_Axis不再重复夹取DR16_Validate已验证范围内的原始通道，公共命令量程与腿长限幅保持。DM/DJI接收回调信任Can_Bus已经检查的标准数据帧和8字节长度，缓冲地址和上下文由注册表固定提供；协议入口、缓存容量、设备状态、SDK返回及最终发送保护仍保留。

## 拨杆模式与输出许可

唯一解码入口为[Rc_Command_Update](../imcalib/task/rc_command.c)。每次通信更新生成`rc.mode.strategy / rc_enable / engage`，随遥控指令一起发布；执行快照直接读取`control_frame.rc.mode`，总使能检查读取`rc_command.mode`，不再重复解码。模式类型归公共`rc_types.h`，保留原策略枚举值；`robot_state.rc_enable`在发布时同步mode许可，供既有状态/调试读取。故障、跌倒恢复、电机使能和总输出开关仍按实际周期检查，不缓存成模式请求。

| 左拨杆 | 右拨杆 | 请求行为 |
| --- | --- | --- |
| 中 | 上 | 仅弹簧与完整重力台架，两轮为零；模型或许可不足则失能 |
| 中 | 中 | 自起/LQR投入 |
| 中 | 下 | 已选LQR，零力矩 |
| 上 | 中 | RL投入，须本机rl.configured |
| 上 | 上或下 | 已选RL，零力矩 |
| 下或遥控离线 | 任意 | 失能 |

投入请求不等于实际出力，仍需在线、故障、翻倒/恢复许可、电机使能、总输出开关和求解有效性。`output_dispatch`在发送前复查实时故障与总输出开关；无效/失能路径发送零力矩或停止。发送入队、请求力矩和实际受力是不同概念。

## 配置与调参入口

只通过`MACHINE_DEFAULT`编译选机，`machine`指针不可运行时换机。小机RL未配置，不把大机网络参数直接套给小机。

| 配置 | 内容与生效位置 |
| --- | --- |
| machine.rc | 公共前进/偏航量程、腿长初值/速率；Rc_Command_Update统一解算，LQR/RL直接读取，RL保留原偏航方向适配 |
| machine.lqr | trim/位移目标、停车条件、角速度低通、腿长/横滚PID、支撑前馈和运行开关默认值 |
| lqr_debug | LQR运行时工作配置与对照入口；初始化取机器默认值，部分物理符号对照保持原源码，不能擅改 |
| machine.rl | 训练映射/默认角、观测缩放/裁剪、动作缩放、腿PD/轮PID、预热与两条动作看门狗 |
| standup_param / air_param / slip_param | 独立自起、接触估计/空中接入、速度融合模块参数，读取对应源文件顶部 |
| machine.spring / machine.gravity | 弹簧几何模型及完整腿自重模型；大机绑定，小机按NULL旁路 |
| DM/DJI量程、极性、零点、几何、IMU映射 | 物理标定；只能由作者实测决定，遵守AGENTS红线 |

控制周期直接取`MACHINE_LQR_DT / MACHINE_RL_CTRL_DT / MACHINE_POLICY_DT`；仅为配套检查设置的`.lqr.dt`已删除。运行参数或PID在初始化/复位位置加载，不能假定改配置副本会自动同步现有控制器。name仅是调试标签，不参与控制。

`rc_command.len`为米，沿用原公式：拨轮死区/归一化后，`len += wheel × rc.len_rate × dt`，回中保持，夹到原机械范围与K表域交集。通信任务直接传`MACHINE_TICK_DT`，更新后若未投入LQR/RL则把腿长恢复`rc.len_init`；离线由RC恢复初值。LQR离地时拨轮仍更新公共目标，轴向释放继续由执行层处理。有效落地仍设置0.20 m，写公共目标及执行当拍命令；作者已移除通信整段临界区，指令发布本身仍由快照接口保护。无新增累加状态、时间戳或复位接口，模式请求在RC解码，实际出力由执行层检查。

## LQR与腿力控

[lqr_balance.c](../imcalib/Algorithm/lqr_balance.c)构建十维状态/目标，按实测腿长求K并输出双轮及左右虚拟髋摆矩。状态顺序为`[s,ds,phi,dphi,thL,dthL,thR,dthR,pitch,dpitch]`，输出顺序为`[左轮,右轮,左虚拟髋,右虚拟髋]`。

[lqr_gain_table.c](../imcalib/Algorithm/lqr_gain_table.c)由`MACHINE_DEFAULT`编译绑定本机独立的`lqr_gain_small/big.c`，与machine配置选机使用同一宏；运行时不切换大小机K表，也不做机型/周期/几何/状态约定等配套检查。生成表中相关字段只保留来源描述。目标腿长夹到机械范围与K表域的交集；实测超域只夹增益求值位置，不伪造测量，也不证明域外稳定。实时输入/计算结果的有限性检查保留；LQR请求许可直接读取machine.lqr_configured。

大小机固定使用各自当前生成表；小机历史表、对照开关/状态/分支和同系数旧候选文件已删除。腿长变化超过原阈值时重新求K，这是同一张表随实测腿长求值，不是切换表。

[leg_balance.c](../imcalib/Algorithm/leg_balance.c)计算左右长度PID、横滚差动力及支撑前馈，加入LQR摆矩后经公共雅可比映射/补偿/限幅。最终腿长目标、斜坡输出/PID给定、实测腿长是三个量。原Ramp_Target_Update首次从实测播种，之后沿上一给定推进；它限制给定速率，不保证实际运动速率。

通用[pid.c](../imcalib/common/pid.c)位置式D使用每拍误差差分，未除周期；RL腿D则直接乘实际关节速度，两者不能按同一Kd口径解释。PID_History_Set只预置历史，不能每拍调用来抹掉真实阻尼。

## RL观测、推理与执行

`task_policy`按机器映射处理实体关节，再通过`RL_Observation_Roles_To_Training/From_Training`做当前训练角色与实体角色互逆转换；角色交换不等同修改电机极性。

| 模块 | 契约 |
| --- | --- |
| [rl_observation.c](../imcalib/Algorithm/rl_observation.c) | 当前观测25维，5帧历史125维；缩放/默认角/裁剪来自machine.rl，首帧按既有规则填满历史 |
| [rl_policy.c](../imcalib/Algorithm/rl_policy.c) | 仅networkzn1；校验25/125输入、6维动作和3维latent输出，管理网络/工作区与耗时/成功失败计数 |
| [task_policy.c](../imcalib/task/task_policy.c) | 未投入只预览；投入后先零动作预热，随后推理/裁剪/保存训练last_action/角色转换/发布；无效或失败发不可用零动作 |
| [rl_torque.c](../imcalib/Algorithm/rl_torque.c) | 动作缩放成默认角偏移或轮速目标，腿P加实际速度阻尼、左右轮独立PID，经雅可比/弹簧/机器限幅输出 |

腿部p_gains/d_gains只有四维，按实体左大腿/左膝/右大腿/右膝排列；六维动作的0/1/3/4对应腿增益0/1/2/3，两轮继续使用wheel_pid。腿PID的Ki及积分限幅固定为零；腿默认角和观测默认角共用machine.rl.default_pos。

RL高度指令直接取公共`rc.len`，进入网络观测；`input_command.height_cmd`仅保存镜像。LQR左右最终腿长同样取`rc.len`，两套控制器均不再重复积分；LQR的原执行斜坡继续使用`rc.len_rate`。动作停更与持续不可用是两条独立看门狗阈值，均由通信任务处理。

## 自起、离地与姿态补角

[standup.c](../imcalib/Algorithm/standup.c)独立管理起立准备和倒置恢复，按姿态/许可推进阶段，生成目标后运行独立控制器与公共补偿；PREPARE/BALANCE/STOP路由交给执行层，准备过程两轮为零。正常自起和翻身共用独立长度速率standup_param.length_rate，与LQR长度速率分开。

[air_detection.c](../imcalib/Algorithm/air_detection.c)用同侧髋反馈反解、弹簧/自重和IMU估计支撑力；armed是初次支撑确认，valid是本拍估计有效，flight是接触状态。缓存未更新也按固定MACHINE_LQR_DT处理；无效观测不会自动清flight，检测只发布状态。

作者2026-10-13进一步要求取消离地时间戳处理，统一固定dt并允许旧缓存参与。估力差分、两级滤波、初次支撑/离地与左右压缩计时均使用MACHINE_LQR_DT（当前两机1ms），不再传接收时间或按样本去重；首次和估计失败恢复仍先播种差分，首次有效拍也推进计时。joint_ready只汇总原四髋在线/实际使能条件，不包含新帧和时间偏差判断。竖直加速度随新IMU帧生成并随online发布，离地读取imu.accel_vertical。

Air_Detection_Init在Robot_Control_Init中一次校验参数并清历史，失败进入Error_Handler；实时Update不再全量扫描参数，参数数值修改后应重新初始化。IMU时间戳停滞门控仍按作者此前要求删除；本轮继续删除rx_ns/now_ns/sample_ns/valid_ns、sample_timeout/sample_skew及执行快照接收时间中转。输入可用性由原反馈标志和joint_ready保证，新估力结果仍验证；失败清连续计时、保留flight，并立即置诊断fault，恢复成功清fault，不新增失败时间计数器。

固定周期假设每次调用经过一个执行拍；重复缓存也持续满足条件时会完成状态切换。若反馈实际间隔变长，腿投影速度的阶跃差分可能增大；若执行任务漏拍，累计时间不补偿真实调度延迟。全项目审核发现其他控制算法已使用固定MACHINE周期，自起阶段计时已按固定dt累计，无其他接收时间dt链路需要替换。DM/DJI/DR16/HI229在线超时、RL动作停更/持续不可用、CAN重试/死链、SPI发送预算仍使用实际时钟；改成固定调用次数会在任务延迟或阻塞时改变保护时限。推理/任务耗时统计与预留接收元数据不属于控制dt门控。

执行层组合AIR与释放开关。释放期间轴向主动力为零，暂停长度/roll PID和执行斜坡，公共RC拨轮仍按原累加规则更新；跳过弹簧抵消，仍保留原空中摆矩及重力补偿。有效落地事件一次复位长度目标，恢复后从实测重播斜坡；退出、复位或坏反馈不伪造落地。AIR还暂停轮速修正及停车积分；恢复冲击、阈值与实时性仍需台架确认。

[pitch_world.c](../imcalib/task/pitch_world.c)保留原IMU pitch的大小，通过同帧四元数投影重力Z的符号选补角分支，返回环绕后的正常/倒置pitch；不重写公共IMU标定或LQR反馈。当前补角无法单独区分横滚倒置和俯仰倒置，输入/有效标志与物理姿态分类应分清。

## 弹簧与重力补偿

[gas_spring.c](../imcalib/Algorithm/gas_spring.c)按实测腿长求挂点几何传动比，把模型弹簧推力换算为等效伸腿力。Gas_Spring_Prepare一次生成gas_spring_result_t中的force[2]、delta_dm[4]及valid：实体力供离地估计，负等效力按原Leg_Force_Map_Forward映射成输出抵消量。Gas_Spring_Apply只叠加已准备值，不再查模型或映射。

当前模型把弹簧轴向推力作为常量force_n，随腿长变化的是几何传动比；没有另建弹簧行程/压力、摩擦或迟滞模型。命令路径先算Fs，再将F=-Fs、Tp=0转换为前后髋增量，与原控制力矩相加并在调用方限幅。只作用于四髋，不直接修改轮矩。

RL仅叠加弹簧抵消；地面LQR、自起和补偿台架走弹簧加完整重力的组合入口。LQR空中轴向释放时跳过命令侧弹簧抵消，使实体弹簧自然伸腿，重力摆矩仍保留。GAS_SPRING_COMP_ENABLE控制是否生成/叠加抵消量，不关闭离地所需的实体力估计；未绑定spring的小机返回零实体弹簧力并旁路补偿。Gas_Spring_Force是准备端的模型入口，后续消费者直接读缓存force，不再重复调用。

执行任务顺序为Control_Frame_Read → Control_Compensation_Update → Control_State_Update → 路由求解 → 唯一分发。Control_Compensation_Update按模式/许可选择准备项，结果放在control_frame.compensation（leg_compensation_values_t）：spring供实体力和抵消量，gravity供原姿态引用，gravity_world供pitch_world补角引用。未投入时清空值；RL无动作或分频等待拍不计算模型，RL始终不接入重力。恢复路径重建LQR状态仍使用同拍准备值，不重复计算。缓存仅属于当前快照，不新增跨周期命中、序号或时间状态。

模型/检测链：准备Fs与原姿态引用重力摆矩 → Air_Estimate反馈反解 → 读取force[side]和gravity.leg[side].torque计算支持力。控制链：基础力矩 → 读取准备的spring.delta_dm → 读取对应gravity.delta_dm → 原限幅及诊断 → 唯一输出许可检查。LQR/自起/台架通过leg_compensation组合入口直接叠加；RL调用Gas_Spring_Apply，仅叠加弹簧值。

[gravity_comp.c](../imcalib/Algorithm/gravity_comp.c)由腿长插值质心半径/偏角，用世界腿角算完整腿自重摆矩，只补摆矩，不增加轴向重力力。Gravity_Comp_Prepare生成摆矩、四髋增量和状态；两种引用需要同时准备时共用同腿查表值，相同规范化角度直接复用摆矩/映射，不再重复求值。原姿态角与pitch_world引用保持区别，自起原fmod环绕在有效角度域内与准备端规范化逐值核验一致；表外夹边界和诊断保持。旧Gravity_Comp_Apply包装已删除，叠加统一归leg_compensation。

[leg_compensation.c](../imcalib/Algorithm/leg_compensation.c)准备本拍模型值并按路径组合弹簧抵消、重力与最终四髋限幅。台架/自起/LQR使用组合入口，RL保留仅弹簧路径；保持base先加spring再加gravity的浮点顺序，不预先合并增量。诊断spring_dm仍取加弹簧后结果减base，避免改变舍入；失败不发布部分输出，保持最终保护。新增本拍缓存用于减少模型调用，不宣称源码行数或所有模式实测耗时下降；有效模型、轴向和极性只能按作者实测修改。

<a id="communication"></a>
## 通信与输入输出

DR16和HI229分别由驱动从UART IDLE/DMA快照取字节并解析；设备解析在任务内完成，UART中断/BSP只搬运和交付接收批次。[uart_idle.c](../imcalib/bsp/uart_idle.c)提供四参数初始化、ISR及统一带时间快照接口UART_Rx_Take_Timed；不需要时间时传NULL。未调用的普通Take包装、parse回调类型/字段/初始化参数和DR16空回调已清理。DMA接收缓存与发送缓存须保持M7 DCache一致性。

debug_rx按作者要求保留用于后续VOFA接收；当前只是预留实例，尚未接入初始化、中断和消费端。当前速度融合直接使用Slip_Update_Contact传入接触状态，未使用的默认接地Slip_Update包装已删除。低通/斜坡初始化与复位接口保持，气弹簧模型和补偿路径保持。

[rc_command.c](../imcalib/task/rc_command.c)唯一读取DR16通道并做死区/夹取/归一化、拨轮累加和模式请求解码。当前ch1为前进、ch2为偏航、wheel为长度/高度调节，保留原偏航符号；公共vel/yaw/len分别为m/s、rad/s、m。拨杆语义每次更新解码为rc.mode，执行层直接读取并检查实际出力许可。

[dm.c](../imcalib/drivers/dm.c)负责MIT量化、反馈换算、极性/零点、在线/使能看门狗及报文；[dji.c](../imcalib/drivers/dji.c)负责编码器/RPM/电流反馈、减速比、轮矩到raw电流及按反馈ID选择电流槽。所有控制者输出逻辑机体量，物理输出极性在驱动边界处理一次。

<a id="vofa"></a>
## 当前VOFA与调试入口

当前[vofa_debug.c](../imcalib/task/vofa_debug.c)直接填39个float，经[Vofa_send.c](../imcalib/bsp/Vofa_send.c)追加JustFloat帧尾、清理DCache并提交所选UART DMA。发送前先查DMA空闲，忙时跳过；没有页面切换、旧10ms节流或UART/USB运行时切换。USB基础初始化保留，当前VOFA只走串口。

旧VOFA选页参数已从air_param清理；RL执行模块中未使用的虚拟小腿限位宏也已删除。实际观测通道、接触判据与输出限幅继续读取各自现行实现。

| 通道 | 当前源码填充值 |
| --- | --- |
| 0 | IMU、遥控、四髋与双轮在线位 |
| 1 | 电机使能、翻倒、两腿有效及LQR/RL投入状态位 |
| 2 | ctrl_fault |
| 3～6 | 左前/左后/右前/右后髋输入角 |
| 7～9 / 10～12 | 左/右腿长、虚拟腿角、虚拟小腿角 |
| 13～15 / 16～18 | 左/右腿长速度、虚拟腿角速度、虚拟小腿角速度 |
| 19～38 | 当前清零 |

作者持续调整通道，采集前必须再次核对实际dbg赋值，不能沿用历史LQR、离地页或自起页下标。未上普通帧的LQR/RL/自起/离地/PID/补偿内部量使用Watch查看。查输出异常先核对采集/下标、最终门控与总线，再分析算法，不改极性或靠低通/降增益掩盖问题。

<a id="matlab-gain"></a>
## MATLAB增益表与模型

日常机器、模型与Q/R入口为[run_all.m](../tools/matlab/run_all.m)，机械来源为[machine_table.m](../tools/matlab/machine_table.m)，动力学为[model_AB.m](../tools/matlab/model_AB.m)，扫描/设计/拟合/闭环检查/写C由[build_gain_table.m](../tools/matlab/build_gain_table.m)完成。当前sjtu5路径已接入，newton15仍保留未接入错误分支。

运行前读取CFG.machine/model/deploy。开启deploy并通过检查后才覆盖对应机型生成表；不能覆盖公共lqr_gain_table绑定/求值实现，也不能用他机K。output中的候选C和报告是数值生成结果，不是板上运行或实机稳定性证明。当前只有大小机对应的生成表，不保留旧单表对照入口。

<a id="build"></a>
## 构建与验证

Keil工程入口为[CtrBoard-H7_ALL.uvprojx](../MDK-ARM/CtrBoard-H7_ALL.uvprojx)，eIDE为[.eide/eide.yml](../.eide/eide.yml)。GCC/AC5分别使用各自的启动、FreeRTOS端口、硬浮点ABI与AI运行库，不混用厂商库。核验时读取当前compile_commands或工程参数，把对象、依赖、ELF/map输出到工程外临时目录，避免污染增量构建。

变更共享头/配置后按两种MACHINE_DEFAULT核验相应消费者；需验证链接时使用完整来源清单。改动纯文档只检查文件、链接、历史保留与引用；注释/提示文字变更核验相应源文件。记录实际检查结果及告警，不能沿用旧测试计数宣称本轮通过。

工具链路径由本机配置提供，工程不写入个人依赖路径。eIDE旧实例可能回写缓存，目录或包含路径变更后先重载，再查源码/编译清单；找不到GCC/Newlib或.NET时核对安装位置、specs、PATH与DOTNET_ROOT。GCC构建通过不代表原任务栈一定足够，USB初始化后的栈余量与节拍信号量应在台架核验。

Python测试/工具和旧tests/Sysid固件已经移除，不再生成额外工程测试文件。旧测试命令、失败记录、源码指纹流程和USB/Sysid操作只保存在历史账本；当前优先使用实际工程编译、MATLAB已有检查和VOFA/Watch台架数据。

## 生成物与维护边界

Core/USB的CubeMX生成区、HAL/CMSIS/FreeRTOS/USB库、CubeAI网络/权重、生成K表、链接脚本与编译缓存各有自己的维护入口。改Q/R走MATLAB；改模型走对应生成流程；业务算法不复制到任务循环或驱动。构建产物和数值报告不能替代当前源码或板端验证。

后续新增/删除模块更新本文的目录、职责、调用和参数入口；每次历史修改写入sysid-change-map。保持四份项目Markdown，不恢复独立模块文档或计划文件，除非作者另行要求。物理标定及修改授权遵守AGENTS。

# 自写代码阅读与冗余检查（2026-11-02）

> 后续输出模型已按路径改为互斥；下文注释重构验收是此前快照，最新模型接口与验证见文末。

本轮重写注释并检查当前调用关系，不改算法、参数、物理映射或运行接口。此前已做的AIR模型请求整合保留；本轮的“仅注释”验证以开始前工作区为基线，不以HEAD为基线。时序诊断按作者要求保留。下面“无调用”均指当前仓库源代码，不能据此推断外部项目没有消费者。

## 先看数据怎么流动

```text
main.c 上电初始化 → freertos.c 创建五个任务
  通信任务：DM/DJI字节 → 物理电机反馈 → 双腿几何 → 发布反馈
  IMU任务：HI229字节 → 取轴/单位/姿态校验 → 发布IMU
  执行任务：读快照 → 恢复/模型/选路 → 力矩计算 → 最终检查 → 电机发送
  策略任务：读执行任务采样 → 网络观测 → 推理 → 发布原始action
  后台任务：读诊断 → VOFA/Jetson发送 → 灯效
```

TIM6每个基础拍释放执行任务信号量，每MACHINE_POLICY_DIV拍释放策略信号量。TIM23维护HAL毫秒时钟。通信和IMU是处理后osDelay轮询，不能把循环次数当作传感器新帧数。

## 目录与阅读入口

| 目录/文件 | 做什么 | 主要输入 → 输出 |
|---|---|---|
| Core/Src/main.c | USB DMA内存、外设及协议初始化，启动调度；定时器节拍回调 | 机器配置 → 驱动/RTOS运行 |
| Core/Src/freertos.c | 创建后台/通信/IMU/推理/执行任务并等待对应节拍 | 信号量或轮询 → task单拍入口 |
| imcalib/config | 机型、周期、几何、物理映射及控制参数 | machine_table → 各模块只读配置 |
| imcalib/bsp | CAN/UART DMA、缓存维护、单调时钟、VOFA打包 | HAL收发 → 带时间的字节或提交状态 |
| imcalib/drivers | DM/DJI/DR16/HI229协议及设备状态，Jetson镜像和灯效 | 字节 → 设备反馈；物理命令 → 报文 |
| imcalib/common | 电机/遥控/RL/力矩公共类型，PID、斜坡、滤波、数学工具 | 标量/结构 → 数学结果 |
| imcalib/task | 各任务单拍入口、共享对象初始化 | 反馈/命令/动作 → 调度各算法 |
| task/service | 反馈快照、遥控目标转换、VOFA布局 | 多任务共享数据 → 统一输入/诊断 |
| task/control | 状态与模型准备、检测调度、控制计算、唯一最终输出 | control_frame → torque_output |
| Algorithm/estimation | 姿态校验、世界俯仰、腿几何、速度融合 | 测量 → 状态及有效性 |
| Algorithm/detection | 支持力/离地、倒地候选、三态选路 | 状态/反馈力矩 → NORMAL/FALLEN/FLIGHT |
| Algorithm/process | 自起翻身和AIR响应 | 选路/测量 → 恢复目标或控制选择 |
| Algorithm/compensation | 弹簧和自重模型计算及叠加 | 腿长/角度 → 模型缓存 → 补偿后力矩 |
| Algorithm/lqr | 十态平衡、四态腿长横滚、参考约束、增益表 | 状态/目标 → 腿力、摆矩、轮矩 |
| Algorithm/rl | 观测、CubeAI推理封装、原始动作到电机力矩 | 反馈/action → 模型观测/候选力矩 |

各模块inc存放配套类型/接口；config的头文件直接位于config。CMSIS三角函数、BMI088移植驱动和生成增益/模型数据属于移植或生成来源，保留其许可证、数值及原始接口。

## 执行拍最重要的变量

| 变量 | 含义与单位 | 谁写、谁读 |
|---|---|---|
| robot_feedback_t | 最新已发布设备反馈；设备异步，不等于同一采样瞬间 | IMU/通信发布，执行任务复制 |
| control_frame_t | 本拍IMU、腿、电机、遥控及许可和模型缓存 | 执行任务内部逐步补齐 |
| drive_permit | 基础出力条件成立，不代表候选有效或已经发送 | 读帧生成，计算/输出复查 |
| nonfallen_drive_permit | 基础许可且公共FALLEN归属为0，不等同NORMAL状态 | 读帧及选路后刷新，控制路径读取 |
| recovery_permit | LQR已投入且自起功能允许的执行许可 | 读帧生成，自起路径读取 |
| strategy | 本拍策略：禁用/补偿/LQR/RL | 策略准备返回，控制计算分支 |
| entered_fallen | 本拍首次转入公共FALLEN的事件 | 检测返回，控制计算复位动作 |
| enabled（模型准备局部量） | 请求计算哪些模型的位组合，不是电机使能 | Control_Model_Prepare组合 |
| frame.compensation | spring、gravity、gravity_world三份可选缓存 | 模型准备写，AIR及输出计算读 |
| torque_output_t | dm四髋、dji双轮候选力矩N·m及valid | 控制计算写，最终输出检查 |
| dispatch | 本拍是否调用输出层；0不是主动发零 | RL分频/控制计算写，执行入口读 |
| action_state.a | 六路网络原始动作，未缩放为物理目标 | 策略推理发布，RL执行消费 |
| action_state.reset_count | 控制段变更标记，拒绝过期在途推理 | 清动作/阶段切换增加，推理前后比较 |
| rl_control.recovery | RL启动保持/连续倾倒失败及计时 | 执行拍推进，策略复制 |
| robot_state.fallen | 公共CONTROL_FALLEN选路归属 | 公共检测发布，不由RL40度角写入 |
| output_dm/wheel_torque_nm | 最终提交的电机命令N·m | 输出层写，诊断读；不是实测力矩 |
| output_debug_*_sent | HAL发送提交成功标志 | 输出层写，不证明实体执行成功 |

## 模型准备为什么分两步

1. Control_Model_Prepare组合需求：输出补偿开关提出命令需求；AIR在反馈就绪时提出估力需求。
2. Leg_Compensation_Prepare根据位标志计算缓存。spring.force是实体力N，spring.delta_dm是抵消力矩N·m（仅补偿编译开启时计算映射）。
3. AIR读取实体力与常规重力摆矩估算支持力；Leg_Compensation_Apply_Mode按所选路径叠加缓存并限矩。

因此“模型存在”不表示“补偿已接到输出”。gravity和gravity_world是同一腿自重公式的两种姿态引用；同时计算也不代表叠加两遍。轴向力F为N，虚拟腿摆矩Tp及髋/轮力矩为N·m。

## 冗余、无调用与保留项

### 当前项目无调用的接口候选（未删除）

- Algorithm/compensation/gas_spring.c/.h：Gas_Spring_Apply。当前实际叠加走Leg_Compensation_Apply_Mode，旧接口另实现了弹簧叠加/有限性检查。
- Algorithm/lqr/lqr_gain_table.c/.h：LQR_K_WBR。仅包装LQR_Gain_Eval并丢弃其有效性返回；当前控制直接调用后者。生成文件注释中的函数名只是文本，不能算消费者。
- drivers/hi229.c/.h：HI229_Deadline。当前IMU链没有调用该死区工具。
- common/kalman.c/.h：Kalman_Accel_Init/Update。当前速度融合使用estimation/slip.c的两状态实现，无调用这套一维工具。
- common/pid.c/.h：FeedForwardController；common/simple-function.c/.h：LPF_Reset、Ramp_Init。当前代码未调用这些接口；参数初始化与Ramp_Reset/Ramp_Target_Update仍有其他用途，不能整文件删除。
- bsp/can_bus.c/.h：Can_Bus_Rx_Count、Can_Bus_Last_Rx_Id仅为无当前调用的诊断接口；不能由此删除CAN接收统计或恢复机制。

### 实现重复或含义过期

- estimation/leg_solver.c同时保留Leg_Solve_Geometry/Velocity与带_z的实现，Leg_Solve当前仅调用_z版。旧静态实现无调用，属于可清理候选；不能按名称猜两版物理等价。
- RL_RECOVERY_RECOVER保留枚举和输出许可判断，但当前RL_Recovery_Update没有进入它。stable_ms仍有累加和条件读取，但当前200ms HOLD/500ms stable配置下未形成独立恢复阶段。这是行为/命名不一致，不在注释任务中擅改状态机。
- Fallen_Process_Wrap与Angle_Wrap_Pi都做弧度包角，但前者用fmodf、后者逐圈运算；浮点和边界行为需单独核对，不能当成完全等价直接替换。
- Control_Model_Prepare的重复AIR模型请求已在此前授权清理中合并；lqr_running形参随之移除，本轮保留该工作区改动。

### 已核对，不能当无效代码删掉

- CONTROL_TIME_VOFA_ENABLE及control_time_debug：作者要求保留的备用时序诊断；默认关闭且当前无VOFA消费者。
- HAL_FDCAN_RxFifo0Callback/ErrorStatusCallback：由HAL回调进入，项目内无直接调用不代表无效。
- BMI088驱动/板级适配：main当前未启用测量链，但属于保留硬件来源，不在本轮删除。
- RL_Action_Index：当前映射恰为恒等，仍表达训练action与电机索引接口，不能仅因数组为0..5就判定无意义。
- 注释掉的VOFA布局、试验入口及作者备注：人工切换资料，保留。

## 本轮验证方法

对开始前的111个C/H文件逐个保存快照。完成后用识别字符串/字符常量的C词法扫描排除注释与空白，核对所有非注释词元完全相同，并核对main/freertos的USER CODE标记未变。该核对覆盖此前未提交控制改动之外的本轮注释改动，不宣称构建、烧录或台架通过。

验证结果：111个文件非注释词元全部一致；98个C/H文件注释有改动（以最终差异为准）；main/freertos的USER CODE标记保持。CMSIS三角函数、BMI088移植源及大小机生成增益源保留开始前原字节。


### 2026-11-02 输出重力补偿按路径互斥选择

- 按作者要求，正常LQR/FLIGHT输出请求LEG_COMP_GRAVITY，自起FALLEN及补偿测试请求LEG_COMP_GRAVITY_WORLD。不再用自起功能enabled表示正在自起；两种重力标志不同时请求。
- Leg_Compensation_Prepare分支只填同一angle数组，末尾一处Gravity_Comp_Prepare调用，结果统一存gravity；删除输出结构的gravity_world成员并同步自起/测试消费者。双重标志请求不生成有效重力结果。
- 为避免上一拍状态影响输入选择，公共检测在输出补偿准备前执行；同拍自起交接到平衡时再按新状态准备普通模型，因此一次准备只调用一次，交接拍可重新准备。
- AIR仍每拍独立估力，直接用Gas_Spring_Force和Gravity_Comp_Compute求原欧拉俯仰对应的实体力/自重摆矩，不再依赖输出缓存；原支持力投影、差分/滤波、连续计时和状态机不改。未修改物理映射、阈值或控制增益。
- 验证：8个相关源ARM GCC语法检查通过；静态对照确认AIR状态/计时函数以及估力的投影/滤波代码保持。未完整链接、烧录或台架验证。

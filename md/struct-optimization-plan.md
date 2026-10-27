# 文件结构整理：当前实施版本

2026-10-27按作者最终要求执行纯文件迁移和命名同步，替代此前目录讨论；历史恢复算法变更见sysid-change-map，不据旧计划恢复删除的阶段或限幅。

## 当前结构

```text
imcalib/
  Algorithm/
    lqr/           十态、四态、增益数据及leg_balance
    rl/            观测、网络和关节控制
    estimation/    姿态、独立pitch_world、腿几何和速度融合
    detection/     air_detection、fallen_detection、control_condition
    process/       air_process、fallen_process
    compensation/  弹簧、重力、补偿组合
  task/
    根目录         六个任务入口/整机初始化源
    control/       strategy、detection、compute、output
    service/       反馈快照、公共遥控、VOFA
  drivers/         C在本目录，H在inc
  bsp/             C在本目录，H在inc
  common/          公共工具，包含inc/torque_output.h
  config/          machine_config.c/h保持原布局
```

Algorithm及task各源码职责目录均设自身inc；父分类目录不造空inc。robot_control.h和robot_state.h保持task/inc，不重划共享状态所有权。

## 特殊迁移

- control_air_response.c/h整体变为Algorithm/process/air_process.c及inc/air_process.h，入口Air_Process_Update，原函数体、全局依赖、临界区和两个调用位置不变。
- ground_recovery.c/h变为process/fallen_process，模块前缀同步Fallen_Process/fallen_process/FALLEN_PROCESS，参数及动作含义不变。
- pitch_world独立迁入estimation，不合并。control_detection保留task/control，继续调度检测并管理公共超时撤力/等待/重判。

## MATLAB

```text
tools/matlab/
  machine_table.m   共用机器参数
  reference/       共用参考数据
  lqr10/           run_all/build_gain_table/model_AB/verify_models、output、cache
  lqr4/            原四态七个M函数及output
```

保留M函数名及全部调参值，不复制机器表。同步搜索路径、根路径计算和固件导出目的地；验证关闭部署，不重写已有固件模型。

## 验证边界

C/H仍54/53，文件移动后函数体仅允许对应标识符和include路径变化。Keil/eIDE登记各一次；完整GCC大小机与Keil编译、MATLAB前后数据核对按本轮记录执行。没有下载、烧录或实机验收，不自动提交Git。保留已有未提交内容及个人未跟踪资料。

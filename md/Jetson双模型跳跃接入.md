# Jetson双模型跳跃接入

本工程在Jetson常驻WALK 6838与Jump 5200，H7负责反馈、观测、PD和最终出力许可。当前为实机试验版本，完整跳跃效果与训练任务一致性尚未验收。

## 遥控与命令

- 左上右中普通RL，左上右下小陀螺，左上右上跳跃待命，替换原关节辨识入口。
- 待命使用WALK。拨轮腿长命令升至公共上限0.33 m时触发一次Jump，保持上限不会连跳。再次触发前先降离上限，再升到上限。
- 进入待命时已经在上限，需要先降低再升高。
- 普通RL机高命令范围为0.15～0.30 m。Jump独立机高命令1 m，对应网络高度输入5.0。
- 触发使用命令端点。1 m是模型命令，不是实测腾空高度。

## 控制与协议

H7组装当前25维观测和125维历史。新契约3000202，每半帧增加12字节，含u32标志、f32平均支持力N和f32平均腿长m。标志低四位依次为跳跃选择、拨轮到顶、支持力有效、传统AIR腾空。
Jetson兼容旧3000201的普通WALK请求，新H7才发送跳跃字段。动作仍校验会话、代次、序号和时间戳。

Jump前进命令乘3，WALK乘2。Jetson单独维护Jump历史并覆盖高度输入。TAKEOFF、FLIGHT和落地缓冲均运行同一个Jump网络，没有手写蓄力或收腿轨迹叠加。
策略100 Hz，PD 200 Hz。腿目标为名义角加0.5倍action，PD 20/1、限矩40 Nm；轮速目标10倍action、限速150 rad/s，速度P 0.1、限矩3.9 Nm。

支持力低于35 N连续30 ms判离地，高于35 N连续50 ms判落地，再缓冲80 ms回WALK。缓冲失去支撑会重新确认。1.4秒超时或支持力失效时停止回包，由H7原有动作超时保护处理。

## 文件与模型

H7入口为USB_DEVICE/App/jetson_usb.c、imcalib/common/inc/jetson_protocol.h和imcalib/task/service/rc_command.c。
Jetson实现位于RL_Jetson_Nona/rl-ynzz_33/policy6838-stage/deployment/jetson_rl/src/，jump_switch.hpp管理阶段和Jump输入，main.cpp负责调度、回包与日志。
Jump包位于RL_Jetson_Nona/rl-ynzz_33/jump5200/，包含ONNX、目标设备TensorRT引擎、接口、参考向量与SHA256SUMS。迁移设备时重新生成引擎并数值核对。

PT来源为训练仓099cf06的logs/mjlab/fudan_ppo_jump/20261119_preload4200_to5200_4096_01/model_final.pt，SHA256为1f1ae7d029c6f741a84b6b703d2ce496a4787c4d29c0934deeb9582c2aa17f80。

## 部署与录制

远端程序位于/home/imca/rl-ynzz_33/jump-dual-stage/，模型位于/home/imca/rl-ynzz_33/jump5200/。机器人失能并烧录配套H7后执行：

    bash /home/imca/rl-ynzz_33/jump-dual-stage/activate_jump_dual.sh

脚本写入jump-model.conf，重启既有jetson-rl-external-4000.service，两模型加载预热后建立通信。回退时删除该drop-in并重启，同时恢复配套旧H7固件。

日志位于/home/imca/rl-ynzz_33/logs/external/，每5分钟分段。kind15记录实际模型的150维输入、六动作、阶段、请求编号、支持力和腿长。使用scripts/export_jump_log.py导出CSV。
当前脚本不把Jetson生成的Jump输入回写为H7的WALK观测，因此分析Jump时读取kind15实际输入。

## 验证边界

H7 ARMCC构建、Jetson CMake构建、阶段检查及22个伪串口请求通过。256组固定向量中ONNX及TensorRT最大绝对误差约1.81e-5和1.91e-5，判据atol=2e-5、rtol=1e-5。
100组真实输入以训练仓原生网络复算，与记录动作最大差异约5.72e-6。

实机已有跳跃记录，目标跟踪和接触判据尚需验证。训练交付采用助跑后切Jump，零速原地跳跃尚无对应验收证据。专用Jump环境及仿真切模源码尚未归档，1 m命令和当前支持力判据属于部署侧试验设置。

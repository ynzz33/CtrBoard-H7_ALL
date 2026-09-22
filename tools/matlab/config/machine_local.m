function m = machine_local()
%MACHINE_LOCAL 小机器 (local-m2006-j4310) 参数表 — 对应 machine_config.c [MACHINE_ID_LOCAL]
% 机体参数照抄 Leg2 WBR_modeling.mlx 生效组 (板上现表就是这组生成的); 腿几何 / 区间 / 限幅与板上机器表对齐,
% check_machine_config.m 会去读 machine_config.c 逐项比对。
% status 取值: 实测 / Leg2 / 旧脚本 / 待实测 ('待实测' 会打进导出 C 的文件头)

m.name = 'local';
m.c_id = 'MACHINE_ID_LOCAL';

% ---- 机体 ----
m.body.g          = 9.81;
m.body.wheel_r    = 0.04;           % 驱动轮半径 [m]            = machine_config.c .wheel_r
m.body.half_track = 0.3459;         % 两驱动轮中心距 / 2 [m]    (模型 R_l)
m.body.l_c        = 0.0501;         % 机体质心到髋轴距离 [m]
m.body.m_w        = 0.13463;        % 单驱动轮质量 [kg]
m.body.m_l        = 0.949;          % 单腿质量 [kg]
m.body.m_b        = 5.25;           % 机体质量 [kg]
m.body.I_w        = 0.000073294;    % 驱动轮转动惯量 [kg m^2]
m.body.I_b        = 0.028669861;    % 机体俯仰转动惯量 [kg m^2]
m.body.I_z        = 0.084144108;    % 整机偏航转动惯量 [kg m^2]

% ---- 腿几何 (= machine_config.c) ----
m.leg.lu      = 0.13087;            % 上杆 (大腿) [m]           = .leg_lu
m.leg.lg      = 0.15240;            % 下杆 (小腿) [m]           = .leg_lg
m.leg.len_min = 0.13;               % 机器腿长区间 [m]          = .leg_len_min (作者 2026-09-22 改 0.09→0.13)
m.leg.len_max = 0.23;               %                           = .leg_len_max (作者 2026-09-22 改 0.21→0.23)

% ---- 腿质心表: sjtu5 用 [L0 l_wl l_bl I_ll] (Leg2 Leg_data, 11 点) ----
%   L0   腿长 (髋心到轮轴)            l_wl 轮轴到腿质心距离
%   l_bl 髋心到腿质心距离             I_ll 腿绕自身质心转动惯量
%   注意: 该表 l_wl + l_bl ≠ L0, 与 sjtu5 模型自洽但不能直接喂 newton15 (见 LQR_MATLAB_PLAN.md §2.3)
m.leg.data_sjtu5 = [
    0.13, 0.13301, 0.08290, 0.011777385;
    0.14, 0.13348, 0.08814, 0.012884469;
    0.15, 0.13403, 0.09345, 0.014075467;
    0.16, 0.13465, 0.09884, 0.015350323;
    0.17, 0.13534, 0.10427, 0.016709084;
    0.18, 0.13610, 0.10975, 0.018151806;
    0.19, 0.13694, 0.11527, 0.019678555;
    0.20, 0.13784, 0.12083, 0.021289413;
    0.21, 0.13880, 0.12641, 0.022984478;
    0.22, 0.13984, 0.13202, 0.024763878;
    0.23, 0.13889, 0.13465, 0.025659060 ];
% newton15 用 [l lw_y lb_y delta Ileg], 须满足 lw_y + lb_y = l — 待作者定 (计划 §九 #1)
m.leg.data_newton15 = [];
m.leg.row_mode = 'nearest';         % 'nearest' = 与 mlx 一致按最近行取 (阶段 0) / 'interp' = pchip 插值

% ---- 控制约束 ----
m.ctrl.Ts          = 0.001;         % 控制周期 [s]              = CTRL_DT
m.ctrl.T_wheel_max = 1.8;           % 轮力矩限幅 [N m]          = .dji_trq_clamp
m.ctrl.T_hip_max   = 10.0;          % 髋力矩限幅 [N m]          = .dm_trq_clamp
m.ctrl.grid        = 0.13:0.01:0.23;% K 表拟合域 (首尾 = LQR_K_LEN_MIN/MAX)
m.ctrl.delay_steps = 1;             % 真实层仿真: 指令延迟拍数 (阶段 1)
m.ctrl.tau_motor   = 0.003;         % 真实层仿真: 电机一阶滞后 [s] (阶段 1)

% ---- 来源 / 状态 ----
m.status = {
    'body.wheel_r',     'Leg2 (作者 2026-09-20 确认 0.04)';
    'body.half_track',  'Leg2';
    'body.l_c',         'Leg2';
    'body.m_w',         'Leg2';
    'body.m_l',         'Leg2';
    'body.m_b',         'Leg2';
    'body.I_w',         'Leg2';
    'body.I_b',         'Leg2';
    'body.I_z',         'Leg2';
    'leg.lu/lg',        '实测 (machine_config.c)';
    'leg.len_min/max',  '实测 (machine_config.c, 作者自标)';
    'leg.data_sjtu5',   'Leg2';
    'leg.data_newton15','待实测 (阶段 2 前作者定)';
    'ctrl.T_*_max',     'machine_config.c';
    'ctrl.grid',        'Leg2 (与 LQR_K_LEN_MIN/MAX 同)';
    };
end

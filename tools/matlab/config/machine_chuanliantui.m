function m = machine_chuanliantui()
%MACHINE_CHUANLIANTUI 大机器 (chuanliantui) 参数表 — 对应 machine_config.c [MACHINE_ID_CHUANLIANTUI]
% 机体参数来自旧脚本 leg_matlab_script/leg_param.m, 全部未实测; 只作预研, 出表不上板。
% 腿几何 / 区间 / 限幅与板上机器表对齐 (轮径板上是 0.04 占位, 旧脚本 0.06, 这里跟板上以便 check 通过, 标待实测)。

m.name = 'chuanliantui';
m.c_id = 'MACHINE_ID_CHUANLIANTUI';

% ---- 机体 (旧脚本 leg_param.m) ----
m.body.g          = 9.81;                       % 旧脚本用 9.8, 这里统一 9.81
m.body.wheel_r    = 0.04;                       % 板上占位值 (旧脚本 0.06) — 待实测
m.body.half_track = 0.221;
m.body.l_c        = sqrt(0.015^2 + 0.01^2);     % 旧脚本 x_c=-0.015, z_c=0.01 → l_c; phi_c 在 sjtu5 里不体现
m.body.m_w        = 0.3;
m.body.m_l        = 2.0;
m.body.m_b        = 20.0;
m.body.I_w        = 0.008;
m.body.I_b        = 0.5 + 20.0 * 0.015^2;       % 旧脚本 I_b = 0.5 + m_b*x_c^2
m.body.I_z        = 0.7;

% ---- 腿几何 (= machine_config.c) ----
m.leg.lu      = 0.21;
m.leg.lg      = 0.25;
m.leg.len_min = 0.14;
m.leg.len_max = 0.34;

% ---- 腿质心表 ----
% newton15 格式 [l lw_y lb_y delta Ileg] 来自旧脚本 (9 点, lw_y + lb_y = l 逐点成立)
m.leg.data_newton15 = [
    0.11, 0.09, 0.02, -0.066, 0.021;
    0.13, 0.10, 0.03, -0.067, 0.022;
    0.15, 0.11, 0.04, -0.067, 0.023;
    0.18, 0.12, 0.06, -0.066, 0.025;
    0.21, 0.13, 0.08, -0.064, 0.026;
    0.24, 0.15, 0.09, -0.062, 0.029;
    0.27, 0.16, 0.11, -0.059, 0.031;
    0.30, 0.18, 0.12, -0.055, 0.034;
    0.33, 0.20, 0.13, -0.051, 0.036 ];
% sjtu5 格式由上表换算: l_wl = sqrt(lw_y^2 + delta^2), l_bl = sqrt(lb_y^2 + delta^2), I_ll = Ileg
D = m.leg.data_newton15;
m.leg.data_sjtu5 = [D(:, 1), sqrt(D(:, 2).^2 + D(:, 4).^2), sqrt(D(:, 3).^2 + D(:, 4).^2), D(:, 5)];
m.leg.row_mode = 'interp';                      % 表只有 9 点, 网格 1 cm, 只能插值

% ---- 控制约束 ----
m.ctrl.Ts          = 0.001;
m.ctrl.T_wheel_max = 4.8;                       % = .dji_trq_clamp
m.ctrl.T_hip_max   = 20.0;                      % = .dm_trq_clamp
m.ctrl.grid        = 0.14:0.01:0.33;            % 待定: 板上 LQR_K_LEN_MIN/MAX 现为小机器的 0.13/0.23
m.ctrl.delay_steps = 1;
m.ctrl.tau_motor   = 0.003;

% ---- 来源 / 状态 ----
m.status = {
    'body.wheel_r',     '待实测 (板上占位 0.04, 旧脚本 0.06)';
    'body.half_track',  '待实测 (旧脚本 0.221)';
    'body.l_c',         '待实测 (旧脚本)';
    'body.m_w',         '待实测 (旧脚本)';
    'body.m_l',         '待实测 (旧脚本)';
    'body.m_b',         '待实测 (旧脚本 20)';
    'body.I_w',         '待实测 (旧脚本)';
    'body.I_b',         '待实测 (旧脚本)';
    'body.I_z',         '待实测 (旧脚本)';
    'leg.lu/lg',        'machine_config.c';
    'leg.len_min/max',  'machine_config.c';
    'leg.data_newton15','待实测 (旧脚本 9 点)';
    'leg.data_sjtu5',   '由 newton15 表换算';
    'ctrl.T_*_max',     'machine_config.c';
    'ctrl.grid',        '待实测 (区间实测后定)';
    };
end

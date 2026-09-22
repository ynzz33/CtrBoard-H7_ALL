function [Q, R, tag] = lqr_weights(machine_name)
%LQR_WEIGHTS Q/R 权重, 按机器分组, 与机械参数分开维护 (调参只改这里)
%   状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b]
%   输出序 u = [T_wl T_wr T_bl T_br]
%   tag 用于表号与复现判定: 'mlx-2026-07-27' = 板上现表那组, 改了 Q/R 就换 tag
switch lower(machine_name)
    case 'local'
        % Leg2 WBR_modeling.mlx 生效组 = 板上现表 (lqr_gain_table.c, 2026-07-27)
        q   = [16000, 1200, 1000, 870, 2500, 365, 2500, 365, 10500, 2000];
        r   = [5480, 5480, 650, 650];
        tag = 'mlx-2026-07-27';
    case 'chuanliantui'
        % 旧脚本 leg_matlab_script/lqr_numeric.m, 未上过板
        q   = [600, 1000, 5000, 80, 15, 2, 15, 2, 90000, 500];
        r   = [50, 50, 1, 1];
        tag = 'leg_matlab_script';
    otherwise
        error('lqr_weights:unknown', '未知机器 "%s"', machine_name);
end
Q = diag(q);
R = diag(r);
end

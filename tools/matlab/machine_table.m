% 文件职责：MATLAB机械/惯性参数表（machine_table.m）。
% 输入/来源：作者现有大机机械数据。
% 输出/去向：返回动力学、几何、腿数据和采样周期配置。
% 关键边界：与固件配置需核对几何/周期；来源和待实测项目不等于已上机标定。
% 阅读入口：md/file-guide.md；参数当前值以源码为准。

function m = machine_table()
%MACHINE_TABLE 大机参数表 — 与 machine_config.c 固定大机配置对应, 不常改
%   m = machine_table()
% 板上 machine_config.c 改了轮径 / 杆长 / 腿长区间 / 限幅, 这里要手动跟着改 (比对脚本已删)。
% status: 实测 / Leg2 / 旧脚本 / 待实测 ('待实测' 会打进导出 C 的文件头)
m.name = 'big_wheelleg'; m.c_id = 'MACHINE_ID_BIG_WHEELLEG';
m.suffix = 'big';
% Leg3 s03 同机参数，轮径已由作者确认。
m.body.g = 9.81;
m.body.wheel_r = 0.0525;
m.body.half_track = 0.212755;
m.body.l_c = 0.01863;
m.body.m_w = 0.65; m.body.m_l = 1.408; m.body.m_b = 20.534;
m.body.I_w = 0.000785;
m.body.I_b = 0.470224494; m.body.I_z = 0.49037341;
m.leg.lu = 0.21; m.leg.lg = 0.25;
m.leg.len_min = 0.14; m.leg.len_max = 0.34;
addpath(fullfile(fileparts(mfilename('fullpath')), 'reference'));
D = leg3_leg_data();
Icom = D(:,4) - m.body.m_l .* D(:,3).^2;
accepted = Icom > 0;
fprintf('Leg3 腿表: %d/%d 行惯量转换有效; 候选域限于 0.15~0.31 m\n', nnz(accepted), size(D,1));
D = D(accepted,:); Icom = Icom(accepted);
lb = D(:,3) .* cos(D(:,2));
m.leg.data_sjtu5 = [D(:,1), D(:,1)-lb, lb, Icom];
m.leg.data_newton15 = [];
m.leg.row_mode = 'interp';
m.ctrl.Ts = 0.001;
m.ctrl.T_wheel_max = (0.30*20)*(15.5/19.2);
m.ctrl.T_hip_max = 54.0;
m.ctrl.grid = 0.15:0.01:0.31;
% 四状态roll/高度LQR专用；Q/R在独立run_roll_height4入口。
% M按站立负载口径取25.544kg（整机含两轮），区别于十态机体质量20.534；
% I_roll/阻尼仍是待辨识初值，不改十状态参数。
m.roll_height = struct('M',25.544,'half_track',m.body.half_track,'I_roll',0.578, ...
    'damping',8,'stiffness',0,'G_roll',0,'g',m.body.g,'Ts',0.001,'H0',0.27);
m.roll_height.pending = 'I_roll/阻尼待辨识（CKP为估算初值，需侧向冲激拟合）；轴向力按近直立等效支持力处理';
m.status = {
    'body.*', 'Leg3 同机 s03 / 轮径作者确认';
    'leg.data_sjtu5', 'Leg3 质心轴向投影及平行轴换算';
    'leg.data_newton15', '待实测 (后续模型)';
    'ctrl.*', 'machine_config / 待台架';
    'leg.table_tail', '待实测 (0.32~0.34 m 惯量不满足平行轴条件)' };


m.ctrl.state_schema = 'hip';

% 自检 + 待实测清单
g = m.ctrl.grid;  D = m.leg.data_sjtu5;
assert(m.leg.len_min < m.leg.len_max, 'leg.len_min 必须小于 len_max');
assert(all(diff(g) > 0) && all(diff(D(:, 1)) > 0), 'grid / 腿数据第 1 列必须递增');
assert(g(1) >= D(1, 1) - 1e-9 && g(end) <= D(end, 1) + 1e-9, ...
    'grid [%.3f %.3f] 超出腿数据覆盖 [%.3f %.3f]', g(1), g(end), D(1, 1), D(end, 1));
m.pending = {};
for k = 1:size(m.status, 1)
    if contains(m.status{k, 2}, '待实测') && ~strcmp(m.status{k,1}, 'leg.data_newton15')
        m.pending{end + 1} = sprintf('%s(%s)', m.status{k, 1}, m.status{k, 2}); %#ok<AGROW>
    end
end
end

function m = machine_load(name)
%MACHINE_LOAD 按名字加载机器参数表 (与 machine_config.c 两份表对应); 所有脚本只认这个入口
%   m = machine_load()            用 machine_default()
%   m = machine_load('local')
if nargin < 1 || isempty(name), name = machine_default(); end
switch lower(name)
    case 'local',        m = machine_local();
    case 'chuanliantui', m = machine_chuanliantui();
    otherwise
        error('machine_load:unknown', '未知机器 "%s" (可选 local / chuanliantui)', name);
end
m = machine_validate(m);
end

function m = machine_validate(m)
% 最低限度自检: 字段齐、区间合理、腿数据能覆盖网格
need = {'name', 'c_id', 'body', 'leg', 'ctrl', 'status'};
for k = 1:numel(need)
    if ~isfield(m, need{k}), error('machine_load:field', '机器表缺字段 %s', need{k}); end
end
bf = {'g', 'wheel_r', 'half_track', 'l_c', 'm_w', 'm_l', 'm_b', 'I_w', 'I_b', 'I_z'};
for k = 1:numel(bf)
    if ~isfield(m.body, bf{k}), error('machine_load:field', '机器表 body 缺 %s', bf{k}); end
end
if ~(m.leg.len_min < m.leg.len_max), error('machine_load:range', 'leg.len_min 必须小于 len_max'); end
if ~isfield(m.leg, 'row_mode'), m.leg.row_mode = 'nearest'; end
g = m.ctrl.grid;
if isempty(g) || any(diff(g) <= 0), error('machine_load:grid', 'ctrl.grid 必须递增'); end
D = m.leg.data_sjtu5;
if ~isempty(D)
    if any(diff(D(:, 1)) <= 0), error('machine_load:legdata', 'leg.data_sjtu5 第 1 列必须递增'); end
    if g(1) < D(1, 1) - 1e-9 || g(end) > D(end, 1) + 1e-9
        error('machine_load:grid', 'ctrl.grid [%.3f, %.3f] 超出 leg.data_sjtu5 覆盖 [%.3f, %.3f]', g(1), g(end), D(1, 1), D(end, 1));
    end
end
if m.ctrl.Ts <= 0, error('machine_load:Ts', 'ctrl.Ts 必须为正'); end
end

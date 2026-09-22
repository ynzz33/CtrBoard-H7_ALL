function ok = run_all(machine_name, model_name)
%RUN_ALL  LQR 增益表管线唯一入口: 配置 → 模型 → 扫描 → dlqr → 拟合 → 导出 C → 核对
%   run_all                    用 config/machine_default.m 与 config/model_default.m
%   run_all('local', 'sjtu5')  显式指定机器 / 模型
% 产物: output/lqr_gain_table.c  output/report_<表号>.txt  cache/scan_<机器>_<模型>.mat
% 计划与验收见 LQR_MATLAB_PLAN.md

here      = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(here));                % CtrBoard-H7_ALL
out_dir   = fullfile(here, 'output');
cache_dir = fullfile(here, 'cache');
addpath(genpath(here));
if ~exist(out_dir, 'dir'),   mkdir(out_dir);   end
if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end

if nargin < 1 || isempty(machine_name), machine_name = machine_default(); end
if nargin < 2 || isempty(model_name),   model_name   = model_default();   end

m = machine_load(machine_name);
[Q, R, qr_tag] = lqr_weights(m.name);
table_id    = sprintf('%s-%s-%s', m.name, model_name, char(datetime('now', 'Format', 'yyyyMMdd-HHmm')));
report_file = fullfile(out_dir, ['report_' table_id '.txt']);
if exist(report_file, 'file'), delete(report_file); end
diary(report_file);
cleanup = onCleanup(@() diary('off')); %#ok<NASGU>

% 复现模式: 与板上现表同机器 / 同模型 / 同 Q-R / 同取行方式, 此时 vs_ref 与 c_vs_board 是判据而不只是信息
is_repro = strcmp(m.name, 'local') && strcmpi(model_name, 'sjtu5') ...
        && strcmp(qr_tag, 'mlx-2026-07-27') && strcmp(m.leg.row_mode, 'nearest');

fprintf('==== run_all  表号 %s ====\n', table_id);
fprintf('机器 %s (%s)   模型 %s   Q/R 组 %s   复现模式 %d\n', m.name, m.c_id, model_name, qr_tag, is_repro);
fprintf('Q = diag([%s])\nR = diag([%s])\n', num2str(diag(Q)'), num2str(diag(R)'));
fprintf('网格 %.2f:%.2f:%.2f (%d 点)   Ts = %g   腿数据取行 %s\n', ...
    m.ctrl.grid(1), m.ctrl.grid(2) - m.ctrl.grid(1), m.ctrl.grid(end), numel(m.ctrl.grid), m.ctrl.Ts, m.leg.row_mode);
pend = machine_pending(m);
if ~isempty(pend), fprintf('待实测项: %s\n', strjoin(pend, ', ')); end

% 1. 网格扫描: A/B → c2d → dlqr
S = scan_grid(model_name, m, Q, R);

% 2. K 拟合 (poly22)
F = fit_K(S, 2);
fprintf('fit_K: poly%d%d, 拟合残差 max|K_fit-K_dlqr| = %.3g (相对 %.3g)\n', ...
    F.order, F.order, max(F.resid_max(:)), max(F.resid_rel(:)));

% 3. 导出 C + 缓存
c_file = fullfile(out_dir, 'lqr_gain_table.c');
emit_gain_table(F, m, model_name, Q, R, qr_tag, c_file, table_id, repo_root);
save(fullfile(cache_dir, sprintf('scan_%s_%s.mat', m.name, model_name)), 'S', 'F', 'm', 'Q', 'R', 'table_id');

% 4. 核对
fprintf('\n---- check_closed_loop ----\n');   ok_cl  = check_closed_loop(S, F);
fprintf('\n---- check_vs_ref ----\n');        ok_ref = check_vs_ref(F, S, m, model_name, is_repro);
fprintf('\n---- check_machine_config ----\n'); ok_cfg = check_machine_config(m, repo_root);
fprintf('\n---- check_c_compile ----\n');     ok_cc  = check_c_compile(c_file, repo_root);
fprintf('\n---- check_c_vs_board ----\n');    ok_c2c = check_c_vs_board(c_file, ...
    fullfile(repo_root, 'imcalib', 'Algorithm', 'lqr_gain_table.c'), m.ctrl.grid, is_repro);

ok = ok_cl && ok_ref && ok_cfg && ok_cc && ok_c2c;
if ok, verdict = '全部通过'; else, verdict = '有未通过项'; end
fprintf('\n==== 结果: closed_loop %d | vs_ref %d | machine_config %d | c_compile %d | c_vs_board %d  =>  %s ====\n', ...
    ok_cl, ok_ref, ok_cfg, ok_cc, ok_c2c, verdict);
fprintf('产物: %s\n报告: %s\n', c_file, report_file);
end

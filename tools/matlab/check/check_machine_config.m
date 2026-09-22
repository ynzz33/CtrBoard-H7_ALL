function ok = check_machine_config(m, repo_root)
%CHECK_MACHINE_CONFIG 读板上 machine_config.c / .h、robot_control.h、lqr_balance.h, 比对与 MATLAB 表重叠的字段
%   wheel_r, leg_lu, leg_lg, leg_len_min, leg_len_max, dji_trq_clamp, dm_trq_clamp, CTRL_DT;
%   若本机器是板上 MACHINE_DEFAULT, 再比 LQR_K_LEN_MIN/MAX 与 ctrl.grid 首尾
ok = true;
tol = 1e-9;
cfg_c = fullfile(repo_root, 'imcalib', 'user-lib', 'machine_config.c');
cfg_h = fullfile(repo_root, 'imcalib', 'user-lib', 'machine_config.h');
rc_h  = fullfile(repo_root, 'imcalib', 'task', 'inc', 'robot_control.h');
lqr_h = fullfile(repo_root, 'imcalib', 'Algorithm', 'lqr_balance.h');
src = fileread(cfg_c, 'Encoding', 'UTF-8');

key = ['[' m.c_id '] = {'];
s = strfind(src, key);
if isempty(s), error('check_machine_config:block', '%s 里找不到 %s', cfg_c, key); end
rest = src(s(1):end);
e = strfind(rest, sprintf('\n    },'));
if isempty(e), error('check_machine_config:block', '%s 的块没找到结束', key); end
blk = rest(1:e(1));

pairs = {
    'wheel_r',       m.body.wheel_r;
    'leg_lu',        m.leg.lu;
    'leg_lg',        m.leg.lg;
    'leg_len_min',   m.leg.len_min;
    'leg_len_max',   m.leg.len_max;
    'dji_trq_clamp', m.ctrl.T_wheel_max;
    'dm_trq_clamp',  m.ctrl.T_hip_max;
    };
fprintf('%-14s %14s %14s  %s\n', '字段', 'MATLAB', 'machine_config.c', '');
for k = 1:size(pairs, 1)
    tok = regexp(blk, ['\.' pairs{k, 1} '\s*=\s*([-+0-9.eE]+)'], 'tokens', 'once');
    if isempty(tok)
        fprintf('%-14s %14.6g %14s  **未找到**\n', pairs{k, 1}, pairs{k, 2}, '?');  ok = false;  continue;
    end
    v = str2double(tok{1});
    same = abs(v - pairs{k, 2}) <= tol;
    if same, mark = 'ok'; else, mark = '**不一致**'; ok = false; end
    fprintf('%-14s %14.6g %14.6g  %s\n', pairs{k, 1}, pairs{k, 2}, v, mark);
end

% CTRL_DT
tok = regexp(fileread(rc_h), '#define\s+CTRL_DT\s+([0-9.eE+-]+)f?', 'tokens', 'once');
if isempty(tok), fprintf('%-14s **robot_control.h 找不到 CTRL_DT**\n', 'Ts'); ok = false;
else
    v = str2double(tok{1});  same = abs(v - m.ctrl.Ts) <= tol;
    if same, mark = 'ok'; else, mark = '**不一致**'; ok = false; end
    fprintf('%-14s %14.6g %14.6g  %s (CTRL_DT)\n', 'Ts', m.ctrl.Ts, v, mark);
end

% K 表域: 只有当本机器是板上默认机器时才是硬判据
tok = regexp(fileread(cfg_h), '#define\s+MACHINE_DEFAULT\s+(\w+)', 'tokens', 'once');
board_default = '';  if ~isempty(tok), board_default = tok{1}; end
lh = fileread(lqr_h);
kmin = str2double(regexp(lh, '#define\s+LQR_K_LEN_MIN\s+([0-9.]+)', 'tokens', 'once'));
kmax = str2double(regexp(lh, '#define\s+LQR_K_LEN_MAX\s+([0-9.]+)', 'tokens', 'once'));
g0 = m.ctrl.grid(1);  g1 = m.ctrl.grid(end);
same = abs(kmin - g0) <= 1e-6 && abs(kmax - g1) <= 1e-6;
if strcmp(board_default, m.c_id)
    if same, mark = 'ok'; else, mark = '**不一致** (换域要先改 lqr_balance.h, 单独授权)'; ok = false; end
    fprintf('%-14s %7.2f~%-6.2f %7.2f~%-6.2f %s (LQR_K_LEN_MIN/MAX, 本机器 = MACHINE_DEFAULT)\n', 'K 表域', g0, g1, kmin, kmax, mark);
else
    fprintf('%-14s %7.2f~%-6.2f %7.2f~%-6.2f 信息 (板上 MACHINE_DEFAULT = %s, 不是本机器)\n', 'K 表域', g0, g1, kmin, kmax, board_default);
end
if ok, fprintf('check_machine_config: 通过\n'); else, fprintf('check_machine_config: **未通过**\n'); end
end

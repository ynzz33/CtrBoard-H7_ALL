function emit_gain_table(F, m, model, Q, R, qr_tag, out_file, table_id, repo_root)
%EMIT_GAIN_TABLE 拟合系数 → C 文件, 与板上 imcalib/Algorithm/lqr_gain_table.c 同签名同下标
%   void LQR_K_WBR(float lL, float lR, float K_sym[40]);   K_sym[状态*4 + 输出]
%   系数按 single 舍入后以 %.9g 打印 (与 MATLAB Coder 一致, 保证 C 端 float 解析回同一个值)
%   多项式求值写在函数体内, 板上求值器不依赖阶次 (poly22 / poly33 都不用改 lqr_balance.c)
if any(~isfinite(F.C(:)))
    error('emit_gain_table:nan', '拟合系数含 NaN/Inf, 拒绝导出');
end
git_hash = '';
[rc, h] = system(sprintf('git -C "%s" rev-parse --short HEAD', repo_root));
if rc == 0, git_hash = strtrim(h); end
pend = machine_pending(m);
if isempty(pend), pend_str = '(无)'; else, pend_str = strjoin(pend, '; '); end

state_names = {'s', 'ds', 'phi', 'dphi', 'th_ll', 'dth_ll', 'th_lr', 'dth_lr', 'th_b', 'dth_b'};
out_names   = {'T_wl', 'T_wr', 'T_bl', 'T_br'};
switch F.order
    case 2, poly_str = 'poly22: K = p00 + p10*lL + p01*lR + p20*lL^2 + p11*lL*lR + p02*lR^2';
    case 3, poly_str = 'poly33: poly22 + p30*lL^3 + p21*lL^2*lR + p12*lL*lR^2 + p03*lR^3';
end

fid = fopen(out_file, 'w', 'n', 'UTF-8');
assert(fid > 0, '无法写 %s', out_file);
c = onCleanup(@() fclose(fid));

w = @(varargin) fprintf(fid, varargin{:});
w('/*\n');
w(' * File: lqr_gain_table.c\n');
w(' *\n');
w(' * WBR LQR 最优反馈增益 K(lL, lR) —— 由 tools/matlab/run_all.m 生成, 勿手改。\n');
w(' *\n');
w(' * 表号   : %s\n', table_id);
w(' * 生成   : %s   git %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), git_hash);
w(' * 机器   : %s (%s)\n', m.name, m.c_id);
w(' * 模型   : %s\n', model);
w(' * Q/R 组 : %s\n', qr_tag);
w(' * Q      : diag([%s])\n', num2str(diag(Q)'));
w(' * R      : diag([%s])\n', num2str(diag(R)'));
w(' * 网格   : lL, lR = %.2f:%.2f:%.2f (%dx%d), Ts = %g, c2d ZOH + dlqr, 腿数据取行 %s\n', ...
    F.grid(1), F.grid(2) - F.grid(1), F.grid(end), numel(F.grid), numel(F.grid), m.ctrl.Ts, m.leg.row_mode);
w(' * 拟合   : %s\n', poly_str);
w(' * 残差   : max|K_fit - K_dlqr| = %.3g (相对 %.3g)\n', max(F.resid_max(:)), max(F.resid_rel(:)));
w(' * 待实测 : %s\n', pend_str);
w(' *\n');
w(' * 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b]\n');
w(' * 输出序 u = [T_wl T_wr T_bl T_br]\n');
w(' * 用法   : LQR_K_WBR(lL, lR, K_sym) -> K_sym[状态*4 + 输出] (lqr_balance.c 按此取 K[输出][状态])\n');
w(' */\n\n');
w('#include "lqr_gain_table.h"\n\n');
w('void LQR_K_WBR(float lL, float lR, float K_sym[40])\n{\n');
w('  float t2;\n  float t3;\n  float t4;\n');
if F.order == 3
    w('  float t5;\n  float t6;\n  float t7;\n  float t8;\n');
end
w('  t2 = lL * lL;\n  t3 = lR * lR;\n  t4 = lL * lR;\n');
if F.order == 3
    w('  t5 = t2 * lL;\n  t6 = t2 * lR;\n  t7 = lL * t3;\n  t8 = t3 * lR;\n');
end
switch F.order
    case 2, vars = {'', 'lL', 'lR', 't2', 't4', 't3'};
    case 3, vars = {'', 'lL', 'lR', 't2', 't4', 't3', 't5', 't6', 't7', 't8'};
end
for j = 1:10
    w('  /* %s */\n', state_names{j});
    for i = 1:4
        cvec = squeeze(F.C(i, j, :));
        expr = fmt_float(cvec(1));
        for k = 2:numel(cvec)
            if cvec(k) >= 0, sgn = '+'; else, sgn = '-'; end
            expr = sprintf('%s %s %s * %s', expr, sgn, fmt_float(abs(cvec(k))), vars{k});
        end
        w('  K_sym[%2d] = %s;   /* %s */\n', (j - 1) * 4 + (i - 1), expr, out_names{i});
    end
end
w('}\n');
fprintf('emit_gain_table: 已写 %s (表号 %s)\n', out_file, table_id);
end

function s = fmt_float(v)
% 先舍入到 single 再以 9 位有效数字打印: C 端解析回来就是同一个 float
s = sprintf('%.9g', double(single(v)));
if isempty(regexp(s, '[.eE]', 'once')), s = [s '.0']; end
s = [s 'F'];
end

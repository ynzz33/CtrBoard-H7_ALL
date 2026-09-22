function ok = check_vs_ref(F, S, m, model, is_repro)
%CHECK_VS_REF 与 Leg2 原件对照
%   (a) A/B: 本管线 model_AB vs ref/AB_WBR_gen.m (Leg2 PSO v2 的 matlabFunction 缓存)
%   (b) K  : 拟合 K vs ref/LQR_K_WBR.m (板上现表的 MATLAB 版), 网格点 + 非对称/非网格点
%   (c) K  : dlqr 原始 K vs ref (网格点), 看"拟合前"差多少
%   复现模式下 (b) 相对误差 <= tol 才算通过; 非复现模式只打印信息, 恒通过
tol = 1e-6;
here = fileparts(mfilename('fullpath'));
ref_dir = fullfile(fileparts(here), 'ref');
grid = S.grid;  n = numel(grid);

% (a) A/B
if strcmpi(model, 'sjtu5') && exist(fullfile(ref_dir, 'AB_WBR_gen.m'), 'file')
    worst = 0;
    for L1 = grid([1, ceil(n/2), n])
        for L2 = grid([1, ceil(n/2), n])
            [A1, B1] = model_AB(model, m, L1, L2);
            [A2, B2] = AB_WBR_gen(sjtu5_param_vec(m, L1, L2));
            d = max(abs([A1(:) - A2(:); B1(:) - B2(:)])) / max(abs([A1(:); B1(:)]));
            worst = max(worst, d);
        end
    end
    fprintf('A/B 对照 (本管线 vs Leg2 AB_WBR_gen, 9 个腿长组合): 最大相对差 %.2e\n', worst);
end

% (b)(c) K
if exist('LQR_K_WBR', 'file') ~= 2
    fprintf('check_vs_ref: 找不到 ref/LQR_K_WBR.m, 跳过\n');
    ok = true;  return;
end
e_fit_grid = 0;  e_raw_grid = 0;  e_fit_abs = 0;  where = [0 0];
for il = 1:n
    for ir = 1:n
        Kr = LQR_K_WBR(grid(il), grid(ir));
        Kf = eval_K_poly(F, grid(il), grid(ir));
        den = max(abs(Kr), 1e-6);
        e = max(abs(Kf - Kr) ./ den, [], 'all');
        if e > e_fit_grid, e_fit_grid = e; where = [grid(il), grid(ir)]; end
        e_fit_abs = max(e_fit_abs, max(abs(Kf - Kr), [], 'all'));
        if S.ok(il, ir)
            e_raw_grid = max(e_raw_grid, max(abs(S.K(:, :, il, ir) - Kr) ./ den, [], 'all'));
        end
    end
end
off = [0.135 0.135; 0.155 0.205; 0.205 0.155; 0.13 0.23; 0.23 0.13; 0.175 0.185; 0.225 0.145];
e_fit_off = 0;
for k = 1:size(off, 1)
    Kr = LQR_K_WBR(off(k, 1), off(k, 2));
    Kf = eval_K_poly(F, off(k, 1), off(k, 2));
    e_fit_off = max(e_fit_off, max(abs(Kf - Kr) ./ max(abs(Kr), 1e-6), [], 'all'));
end
fprintf('K 对照 (拟合 K vs ref, %d 网格点): 最大相对差 %.2e (绝对 %.2e) @ lL=%.3f lR=%.3f\n', n*n, e_fit_grid, e_fit_abs, where(1), where(2));
fprintf('K 对照 (拟合 K vs ref, %d 非网格/非对称点): 最大相对差 %.2e\n', size(off, 1), e_fit_off);
fprintf('K 对照 (dlqr 原始 K vs ref, 网格点): 最大相对差 %.2e  (= ref 自身的拟合误差量级)\n', e_raw_grid);

% 标称腿长把两边 K 打印出来看一眼
kn = ceil(n / 2);
fprintf('标称 lL=lR=%.2f, 本管线拟合 K (行 T_wl T_wr T_bl T_br, 列 s..dth_b):\n', grid(kn));
disp(eval_K_poly(F, grid(kn), grid(kn)));
fprintf('同点 ref K:\n');
disp(LQR_K_WBR(grid(kn), grid(kn)));

if is_repro
    ok = max(e_fit_grid, e_fit_off) <= tol;
    if ok, fprintf('check_vs_ref: 通过 (复现模式, tol %.0e)\n', tol);
    else,  fprintf('check_vs_ref: **未通过** (复现模式, tol %.0e)\n', tol); end
else
    ok = true;
    fprintf('check_vs_ref: 非复现模式, 只作信息\n');
end
end

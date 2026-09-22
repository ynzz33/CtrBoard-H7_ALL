function S = scan_grid(model, m, Q, R)
%SCAN_GRID 左右腿长二维网格: A/B → c2d(ZOH, Ts) → dlqr → K
%   S.A/S.B   连续 10x10 / 10x4 (:,:,il,ir)
%   S.Ad/S.Bd 离散
%   S.K       4x10 (:,:,il,ir), dlqr 失败处为 NaN, S.ok 标记
% c2d 用与 Leg2 mlx 相同的矩阵形式 c2d(A, B, Ts) (ZOH)
grid = m.ctrl.grid(:)';
n    = numel(grid);
Ts   = m.ctrl.Ts;

S.model = model;  S.machine = m.name;  S.grid = grid;  S.Ts = Ts;  S.Q = Q;  S.R = R;
S.A  = zeros(10, 10, n, n);  S.B  = zeros(10, 4, n, n);
S.Ad = zeros(10, 10, n, n);  S.Bd = zeros(10, 4, n, n);
S.K  = nan(4, 10, n, n);     S.ok = false(n, n);

t0 = tic;
for il = 1:n
    for ir = 1:n
        [A, B] = model_AB(model, m, grid(il), grid(ir));
        [Ad, Bd] = c2d(A, B, Ts);
        S.A(:, :, il, ir)  = A;   S.B(:, :, il, ir)  = B;
        S.Ad(:, :, il, ir) = Ad;  S.Bd(:, :, il, ir) = Bd;
        try
            S.K(:, :, il, ir) = dlqr(Ad, Bd, Q, R);
            S.ok(il, ir) = true;
        catch ME
            warning('scan_grid:dlqr', 'dlqr 失败 lL=%.3f lR=%.3f: %s', grid(il), grid(ir), ME.message);
        end
    end
end

% 可控性只在标称点报一次
kn = ceil(n / 2);
S.ctrb_rank_nom = rank(ctrb(S.A(:, :, kn, kn), S.B(:, :, kn, kn)));
fprintf('scan_grid: %s / %s, %dx%d 点, dlqr 成功 %d/%d, 标称 L=%.2f 可控秩 %d/10, 耗时 %.1f s\n', ...
    m.name, model, n, n, nnz(S.ok), n*n, grid(kn), S.ctrb_rank_nom, toc(t0));
end

function F = fit_K(S, order)
%FIT_K 对 K 的 40 个元素做二元多项式最小二乘拟合 (不依赖 Curve Fitting Toolbox)
%   order = 2 : poly22, K = p00 + p10*lL + p01*lR + p20*lL^2 + p11*lL*lR + p02*lR^2   (板上现表格式)
%   order = 3 : poly33, 再加 p30*lL^3 + p21*lL^2*lR + p12*lL*lR^2 + p03*lR^3
%   F.C(i, j, k): 输出 i (1..4), 状态 j (1..10), 系数 k (顺序 = F.names)
%   与 Leg2 mlx 的 fit(X, y, 'poly22') 同一基函数、同一顺序 (p00 p10 p01 p20 p11 p02), 'Normalize' 关
if nargin < 2, order = 2; end
grid = S.grid;
[LL, LR] = ndgrid(grid, grid);
x = LL(:);  y = LR(:);
switch order
    case 2
        X = [ones(size(x)), x, y, x.^2, x.*y, y.^2];
        F.names = {'p00', 'p10', 'p01', 'p20', 'p11', 'p02'};
    case 3
        X = [ones(size(x)), x, y, x.^2, x.*y, y.^2, x.^3, x.^2.*y, x.*y.^2, y.^3];
        F.names = {'p00', 'p10', 'p01', 'p20', 'p11', 'p02', 'p30', 'p21', 'p12', 'p03'};
    otherwise
        error('fit_K:order', '只支持 order = 2 或 3');
end
F.order     = order;
F.grid      = grid;
F.C         = zeros(4, 10, numel(F.names));
F.resid_max = zeros(4, 10);
F.resid_rel = zeros(4, 10);
for i = 1:4
    for j = 1:10
        z  = reshape(S.K(i, j, :, :), [], 1);        % 线性下标 il 变快, 与 ndgrid LL(:) 一致
        ok = isfinite(z);
        if nnz(ok) < numel(F.names)
            error('fit_K:points', 'K(%d,%d) 有效点 %d 个, 不够拟合', i, j, nnz(ok));
        end
        c = X(ok, :) \ z(ok);
        r = X(ok, :) * c - z(ok);
        F.C(i, j, :)     = c;
        F.resid_max(i, j) = max(abs(r));
        F.resid_rel(i, j) = max(abs(r)) / max(max(abs(z(ok))), 1e-12);
    end
end
end

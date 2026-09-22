function ok = check_closed_loop(S, F)
%CHECK_CLOSED_LOOP 拟合后的 K 在全网格上的离散闭环谱半径 + 拟合残差
%   判据: 全部格点 dlqr 成功, 且 rho(Ad - Bd*K_fit) < 1
n = numel(S.grid);
rho_fit = nan(n);  rho_raw = nan(n);
for il = 1:n
    for ir = 1:n
        Ad = S.Ad(:, :, il, ir);  Bd = S.Bd(:, :, il, ir);
        Kf = eval_K_poly(F, S.grid(il), S.grid(ir));
        rho_fit(il, ir) = max(abs(eig(Ad - Bd * Kf)));
        if S.ok(il, ir)
            rho_raw(il, ir) = max(abs(eig(Ad - Bd * S.K(:, :, il, ir))));
        end
    end
end
[rf, kf] = max(rho_fit(:));  [ilf, irf] = ind2sub([n n], kf);
[rr, kr] = max(rho_raw(:));  [ilr, irr] = ind2sub([n n], kr);
% 最慢极点折算成连续衰减率 (1/s), 越负越快
fprintf('闭环谱半径 (拟合 K): 最大 %.6f @ lL=%.2f lR=%.2f  (折算 %.2f 1/s)\n', rf, S.grid(ilf), S.grid(irf), log(rf) / S.Ts);
fprintf('闭环谱半径 (dlqr K): 最大 %.6f @ lL=%.2f lR=%.2f  (折算 %.2f 1/s)\n', rr, S.grid(ilr), S.grid(irr), log(rr) / S.Ts);
[ra, ka] = max(F.resid_max(:));  [ia, ja] = ind2sub([4 10], ka);
[rl, kl] = max(F.resid_rel(:));  [ir_, jr_] = ind2sub([4 10], kl);
fprintf('拟合残差: 最大绝对 %.3g @ K(%d,%d), 最大相对 %.3g @ K(%d,%d)\n', ra, ia, ja, rl, ir_, jr_);
ok = all(S.ok(:)) && all(rho_fit(:) < 1);
if ok, fprintf('check_closed_loop: 通过\n'); else, fprintf('check_closed_loop: **未通过**\n'); end
end

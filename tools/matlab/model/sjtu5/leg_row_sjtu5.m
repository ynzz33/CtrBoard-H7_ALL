function row = leg_row_sjtu5(m, L)
%LEG_ROW_SJTU5 腿长 L → [l_wl l_bl I_ll] (sjtu5 模型用)
%   m.leg.row_mode = 'nearest' : 取 leg.data_sjtu5 里最近的一行 (与 Leg2 mlx 按下标取一致, 阶段 0)
%                    'interp'  : 对三列做 pchip 插值 (表粗、网格细时用)
D = m.leg.data_sjtu5;
switch m.leg.row_mode
    case 'nearest'
        [~, k] = min(abs(D(:, 1) - L));
        row = D(k, 2:4);
    case 'interp'
        row = interp1(D(:, 1), D(:, 2:4), L, 'pchip');
        if any(~isfinite(row))
            error('leg_row_sjtu5:range', '腿长 %.4f 超出 leg.data_sjtu5 范围 [%.3f, %.3f]', L, D(1, 1), D(end, 1));
        end
    otherwise
        error('leg_row_sjtu5:mode', '未知 row_mode "%s"', m.leg.row_mode);
end
end

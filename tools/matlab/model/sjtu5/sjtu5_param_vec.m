function p = sjtu5_param_vec(m, lL, lR)
%SJTU5_PARAM_VEC 机器表 + 左右腿长 → AB_sjtu5_gen 的 18 维参数向量
%   顺序 (与 build_sjtu5.m / Leg2 AB_WBR_gen.m 一致):
%   [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g]
rowL = leg_row_sjtu5(m, lL);      % [l_wl l_bl I_ll]
rowR = leg_row_sjtu5(m, lR);
b = m.body;
p = [b.wheel_r, b.half_track, lL, lR, rowL(1), rowR(1), rowL(2), rowR(2), b.l_c, ...
     b.m_w, b.m_l, b.m_b, b.I_w, rowL(3), rowR(3), b.I_b, b.I_z, b.g];
end

function build_sjtu5(gen_file)
%BUILD_SJTU5 Leg2 上交 5 方程线性模型 → 符号 A/B → matlabFunction 缓存
%   方程 (19)-(23) 与 Leg2 WBR_modeling.mlx / PSO_LQR_Tuning_v2.m Section 2 逐字相同。
%   生成函数 [A, B] = AB_sjtu5_gen(p), p 的顺序见 sjtu5_param_vec.m:
%   [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g]
%   状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b], 输出序 u = [T_wl T_wr T_bl T_br]
assert(~isempty(which('syms')), 'build_sjtu5 需要 Symbolic Math Toolbox');
fprintf('build_sjtu5: 符号推导并生成 %s ...\n', gen_file);
t0 = tic;

syms R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b I_w I_ll I_lr I_b I_z g
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b
syms T_wl T_wr T_bl T_br
syms theta_ll theta_lr theta_b

eqn1 = (I_w*l_l/R_w + m_w*R_w*l_l + m_l*R_w*l_bl)*ddtheta_wl ...
     + (m_l*l_wl*l_bl - I_ll)*ddtheta_ll ...
     + (m_l*l_wl + m_b*l_l/2)*g*theta_ll ...
     + T_bl - T_wl*(1 + l_l/R_w) == 0;

eqn2 = (I_w*l_r/R_w + m_w*R_w*l_r + m_l*R_w*l_br)*ddtheta_wr ...
     + (m_l*l_wr*l_br - I_lr)*ddtheta_lr ...
     + (m_l*l_wr + m_b*l_r/2)*g*theta_lr ...
     + T_br - T_wr*(1 + l_r/R_w) == 0;

eqn3 = -(m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2)*ddtheta_wl ...
     - (m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2)*ddtheta_wr ...
     - (m_l*R_w*l_wl + m_b*R_w*l_l/2)*ddtheta_ll ...
     - (m_l*R_w*l_wr + m_b*R_w*l_r/2)*ddtheta_lr ...
     + T_wl + T_wr == 0;

eqn4 = (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_wl ...
     + (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_wr ...
     + m_l*l_wl*l_c*ddtheta_ll + m_l*l_wr*l_c*ddtheta_lr ...
     - I_b*ddtheta_b + m_b*g*l_c*theta_b ...
     - (T_wl + T_wr)*l_c/R_w - (T_bl + T_br) == 0;

eqn5 = ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wl ...
     - ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wr ...
     + (I_z*l_l)/(2*R_l)*ddtheta_ll - (I_z*l_r)/(2*R_l)*ddtheta_lr ...
     - T_wl*R_l/R_w + T_wr*R_l/R_w == 0;

sols = solve([eqn1, eqn2, eqn3, eqn4, eqn5], ...
             [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b]);
dd = [sols.ddtheta_wl, sols.ddtheta_wr, sols.ddtheta_ll, sols.ddtheta_lr, sols.ddtheta_b];

J_A = jacobian(dd, [theta_ll, theta_lr, theta_b]);
J_B = jacobian(dd, [T_wl, T_wr, T_bl, T_br]);

A_sym = sym(zeros(10, 10));
B_sym = sym(zeros(10, 4));
for r = 1:2:9
    A_sym(r, r + 1) = 1;
end
col = [5, 7, 9];                                  % theta_ll, theta_lr, theta_b
A_sym(2,  col) = R_w * (J_A(1, :) + J_A(2, :)) / 2;                       % dd s
A_sym(4,  col) = (R_w * (-J_A(1, :) + J_A(2, :))) / (2*R_l) ...           % dd phi
               - (l_l * J_A(3, :)) / (2*R_l) + (l_r * J_A(4, :)) / (2*R_l);
A_sym(6,  col) = J_A(3, :);                                               % dd theta_ll
A_sym(8,  col) = J_A(4, :);                                               % dd theta_lr
A_sym(10, col) = J_A(5, :);                                               % dd theta_b
for h = 1:4
    B_sym(2,  h) = R_w * (J_B(1, h) + J_B(2, h)) / 2;
    B_sym(4,  h) = (R_w * (-J_B(1, h) + J_B(2, h))) / (2*R_l) ...
                 - (l_l * J_B(3, h)) / (2*R_l) + (l_r * J_B(4, h)) / (2*R_l);
    B_sym(6,  h) = J_B(3, h);
    B_sym(8,  h) = J_B(4, h);
    B_sym(10, h) = J_B(5, h);
end

param_sym_vec = [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, ...
                 m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g];

gen_dir = fileparts(gen_file);
if ~exist(gen_dir, 'dir'), mkdir(gen_dir); end
matlabFunction(A_sym, B_sym, 'File', gen_file, 'Vars', {param_sym_vec}, 'Outputs', {'A', 'B'});
addpath(gen_dir);
rehash;
fprintf('build_sjtu5: 完成, 耗时 %.1f s\n', toc(t0));
end

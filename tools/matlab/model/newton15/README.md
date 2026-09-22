# model/newton15 — 阶段 2 接入

自研 15 方程牛顿-欧拉模型（`leg_matlab_script/eq2ss.m` 的推导）。阶段 2 才写：

- `build_newton15.m`：15 方程 → 消元 → 线性化 → `A_orig / B_u / B_x` → `T_acc` → 拼 10 阶 → `matlabFunction` 缓存到 `cache/AB_newton15_gen.m`
- `leg_row_newton15.m`：腿长 → `[lw_y lb_y delta Ileg]`，断言 `lw_y + lb_y = l`

前置：作者定小机器 `leg.data_newton15` 的来源（`LQR_MATLAB_PLAN.md` §九 #1）。

# ref/ — 对照件（只读，不参与生成）

| 文件 | 来源 | 用途 |
|---|---|---|
| `LQR_K_WBR.m` | `Leg2_v1(1)/Leg2_v1/轮腿上交建模MATLAB/LQR_K_WBR.m`（Symbolic Math Toolbox 生成，2026-07-27 15:02:39） | 板上现表 `imcalib/Algorithm/lqr_gain_table.c` 的 MATLAB 原型（现表 = 此文件经 MATLAB Coder 转 C，逐字节与 Leg2 `Code/Matlab/LQR_K_WBR.c` 一致）。`check_vs_ref.m` 用它做双精度对照 |
| `AB_WBR_gen.m` | 同目录 `AB_WBR_gen.m`（PSO_LQR_Tuning_v2.m 首次运行时 `matlabFunction` 生成，2026-07-21） | Leg2 5 方程模型的 A/B 数值函数，参数顺序与本管线 `sjtu5_param_vec.m` 相同。`check_vs_ref.m` 用它核对本管线的符号推导 |

复制日期：2026-09-22。原件改了要同步，否则对照失真。

# LQR 增益表 MATLAB 管线 — 计划（tools/matlab/）

> 状态：**阶段 0 已完成（2026-09-22，变更 85），阶段 1 未开工**。脚本按 §六 的阶段逐步落地，每阶段结束回来更新 §十 的进度表。运行方式见 §十二。
> 作者决策（2026-09-21）：新建一套完整管线，不改 `leg_matlab_script`；先用 Leg2 模型复现现表并调参验证可用，再换成自研 15 方程模型；参数用"机器配置表"，小机器/大机器一处切换，与板上 `machine_config.c` 同思路。
> 文中所有数值都是 2026-09-21 的快照，以对应脚本/代码为准。

---

## 一、目标与边界

**目标**：让"改 Q/R → 出 K 表 → 核对 → 上板"这条路一条命令跑完、可复现、可追溯，并且能在两台机器、两个动力学模型之间切换。

**边界**：
- 产出物只有一个：与现 `imcalib/Algorithm/lqr_gain_table.c` **同签名、同下标**的 C 文件。板上求值器 `LQR_Control_Update()`、拟合域宏 `LQR_K_LEN_MIN/MAX` 在阶段 0~2 一行不动。
- 本管线不碰极性、零点、轴向、量程（AGENTS §0.1）。它输出的是增益，输入的机械参数里"待实测"项只能作者定。
- `leg_matlab_script`（大机器旧脚本）和 `Leg2_v1/轮腿上交建模MATLAB`（上交原件）只作来源与对照，不改、不删。

---

## 二、背景：现表从哪来，两份 MATLAB 差在哪

### 2.1 板上现表的来源

`imcalib/Algorithm/lqr_gain_table.c` 的生成时间 2026-07-27 15:02:48，Leg2 `LQR_K_WBR.m` 15:02:39，`WBR_modeling.mlx` 保存于同日 15:02。三者时间连续，**现表 = `WBR_modeling.mlx` 用其文件内那组 Q/R 生成**：

```
Q = diag([16000 1200 1000 870 2500 365 2500 365 10500 2000])   % s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b
R = diag([5480 5480 650 650])                                   % T_wl T_wr T_bl T_br
```

PSO 脚本里的 `Q0=[6000 2000 1 800 …]` 是另一组基线，没上过板。

### 2.2 两份 MATLAB 的主要区别

| 项 | `leg_matlab_script`（自研，大机器） | Leg2 `轮腿上交建模MATLAB`（小机器，现表来源） |
|---|---|---|
| 物理模型 | 15 条牛顿-欧拉方程消元成 5 条，含机体质心偏置 `l_c/phi_c`、腿质心离轴 `delta`（`eq2ss.m`） | 上交论文 (3.11)~(3.15) 直接写好的 5 条线性方程，腿质心假设在腿轴线上，无 `phi_c` |
| 状态/输入顺序 | `[s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b]`、`[T_wl T_wr T_bl T_br]` | 同 |
| 机器参数 | `R_w 0.06`、`R_l 0.221`、`m_b 20`、`I_b 0.5+m_b*x_c^2` 等；腿数据 9 点 0.11~0.33 m 带 `delta` 列 | `R_w 0.04`、`R_l 0.3459`、`l_c 0.0501`、`m_b 5.25`；腿数据 11 点 0.13~0.23 m 无 `delta` |
| 腿参数进模型 | poly3 拟合后按腿长求值；`lw_y + lb_y = l` 逐点成立，几何自洽 | 直接按网格下标取 `Leg_data(il,:)`；`leg_fit.mlx` 的 poly4 拟合没被主循环使用；表内 `l_wl + l_bl ≠ L0`（0.13 m 时和为 0.216），0.23 行 `l_wl` 倒退 |
| 腿长网格 | 二维 0.11:0.01:0.33，529 点 | 二维 0.13:0.01:0.23，121 点 |
| A/B 数值化 | 每格点 `subs`+`vpa`，很慢 | mlx 也是 `subs`；PSO v2 改为 `matlabFunction` 缓存 `AB_WBR_gen.m`，121 点秒级 |
| 离散化/求解 | `c2d` Ts=0.001 + `dlqr` | 同（PSO 调参用连续 `lqr`） |
| Q/R | `[600 1000 5000 80 15 2 15 2 90000 500]` / `[50 50 1 1]` | 见 §2.1 |
| K 拟合 | `poly33`，10 项/元素 | `poly22`，6 项/元素 |
| 输出 | `wholebody.hpp` 数组 `kMatureLqrPoly_whole[4][10][10]` + `A_L/B_L/U_d` 表，板上不认 | `matlabFunction` → `LQR_K_WBR.m` → Coder → `LQR_K_WBR.c`，接口 `LQR_K_WBR(lL,lR,K_sym[40])` |
| 附加工具 | 无 | PSO v1/v2（v2 带 ZOH+一拍延迟+饱和+电机滞后的"真实层"仿真与极点整形）、VMC 三函数、`check.slx` |
| 已知小毛病 | `lqr_numeric.m` 的 `Ts = meta.step` 取的是腿长步距（未用到但误导）；`g=9.8` | `R_w_ac=0.03` 组是被注释掉的另一台车；`leg_fit.mlx` 内还有第三套 0.06~0.23 腿数据，与 mlx 用的不是一回事 |

**符号约定两边一致**（读式子得出，阶段 2 用数值比对兜底）：`phi = -R_w/(2R_l)(th_wl-th_wr)+…` 左转为正；`S_b` 中 `l*sin(th)` 与 `l_c*sin(th_b+phi_c)` 同号，即"θ 正 = 机体相对轮向前"，对应板上 `x[th_ll] = -解算摆角 + pitch`、俯仰低头为正。

### 2.3 为什么不能直接拿一份改参数

- 拿 Leg2 mlx 改参数：参数、Q/R、推导、导出混在一个 mlx 里，无法分机器、无法复现"某张表对应哪组参数"，且腿数据按下标取、不能换网格。
- 拿自研脚本换参数：模型不同、腿数据定义不同、输出格式板上不认，三处都要动，出问题无法归因。
- **Leg2 的 `Leg_data` 不能喂 15 方程模型**：Leg2 模型里 `l_wl/l_bl` 只以乘积或单独项出现，表内不自洽它自己不报错；15 方程模型会把 `l_wl + l_bl ≠ l` 解释成腿质心偏离轴线几十度，A/B 全错。

---

## 三、总体路线（作者定）

```
阶段 0  搭管线：Leg2 5 方程模型 + 小机器表 + mlx 那组 Q/R  →  复现现表（240 系数逐个对上）
阶段 1  用它调参：whatif + 真实层仿真 → 出表 → 核对 → 上板 → 记账；验证"可用"
阶段 2  接自研 15 方程模型：同参数比 A/B、比 K 趋势，过了才允许上板
阶段 3  文档收口
```

一次只换一个变量：阶段 0/1 不换模型，阶段 2 不换 Q/R。

---

## 四、目录与文件职责

放在本仓库 `CtrBoard-H7_ALL/tools/matlab/`（与 `tools/sysid_export.py` 同级）。放仓库里的理由：生成的 C 直接落到 `imcalib/Algorithm/`，git 把"哪张表 ↔ 哪组 Q/R ↔ 哪版固件"一起记住。

```
tools/matlab/
├── LQR_MATLAB_PLAN.md          ← 本文件
├── run_all.m                   ← 唯一入口：读配置 → 取模型 → 扫描 → dlqr → 拟合 → 导出 → 五项核对 → 报告
├── config/
│   ├── machine_default.m       ← 一行：当前机器 'local' | 'chuanliantui'（对应 machine_config.h 的 MACHINE_DEFAULT）
│   ├── machine_load.m          ← 按 machine_default 返回 struct 并自检；所有脚本只认它
│   ├── machine_pending.m       ← 列出 status 为"待实测"的字段（进导出 C 的文件头）
│   ├── machine_local.m         ← 小机器参数表
│   ├── machine_chuanliantui.m  ← 大机器参数表（多数项 status='待实测'，只作预研）
│   ├── model_default.m         ← 一行：当前模型 'sjtu5' | 'newton15'
│   └── lqr_weights.m           ← Q/R，按机器分组，带 tag（改了 Q/R 就换 tag；tag 决定是否进"复现模式"）
├── model/
│   ├── model_AB.m              ← 统一接口 [A,B] = model_AB(model, m, lL, lR)；无缓存则先 build
│   ├── sjtu5/
│   │   ├── build_sjtu5.m       ← Leg2 5 条方程 → solve → jacobian → matlabFunction → cache/AB_sjtu5_gen.m（首次约 13 s）
│   │   ├── sjtu5_param_vec.m   ← 机器表 + 腿长 → 18 维参数向量（顺序与 Leg2 AB_WBR_gen 相同）
│   │   └── leg_row_sjtu5.m     ← 腿长 → [l_wl l_bl I_ll]（'nearest' 与 mlx 一致 / 'interp' pchip）
│   └── newton15/README.md      ← 阶段 2 才写 build_newton15.m / leg_row_newton15.m
├── design/
│   ├── scan_grid.m             ← 网格 → A/B → c2d(A,B,Ts)（与 mlx 同一调用）→ dlqr → K_grid(4,10,nL,nR)
│   ├── fit_K.m                 ← poly22 / poly33 最小二乘（不依赖 Curve Fitting Toolbox）
│   └── eval_K_poly.m           ← 用拟合系数求 K(lL,lR)
├── emit/
│   └── emit_gain_table.m       ← fprintf 直接写 C：void LQR_K_WBR(float lL, float lR, float K_sym[40])
├── check/
│   ├── check_closed_loop.m     ← 拟合后 K 在全网格算离散闭环谱半径、拟合残差
│   ├── check_vs_ref.m          ← A/B 对照 ref/AB_WBR_gen.m；K 对照 ref/LQR_K_WBR.m（复现模式下 tol 1e-6 为判据）
│   ├── check_machine_config.m  ← 解析 machine_config.c/.h、robot_control.h、lqr_balance.h，比对重叠字段
│   ├── check_c_compile.m       ← 用 compile_commands.json 里的 AC5 命令编生成的 C（-o 到临时目录）
│   ├── check_c_vs_board.m      ← 调下面的 Python：生成 C vs 板上 C 逐点数值比对
│   └── compare_c_tables.py     ← 把两份 C 的函数体当直线代码执行（兼容 Coder 写法与本管线写法）
├── ref/
│   ├── LQR_K_WBR.m             ← Leg2 原表 MATLAB 版（复制，只作对照）
│   ├── AB_WBR_gen.m            ← Leg2 PSO v2 的 A/B 缓存（复制，只作对照）
│   └── README.md               ← 每个 ref 文件的来源路径与日期
├── cache/                      ← AB_sjtu5_gen.m、scan_<机器>_<模型>.mat；已进 .gitignore
└── output/
    ├── lqr_gain_table.c        ← 产物；核对通过后手动拷到 imcalib/Algorithm/
    └── report_<表号>.txt       ← run_all 全程输出（diary），随表号记进 sysid-change-map.md
```

---

## 五、关键设计

### 5.1 机器配置表（与 `machine_config.c` 同思路）

每台机器一个文件返回一个 struct，`machine_default.m` 选哪台。字段分四组，每个数值字段可挂 `status`：`'实测'` / `'Leg2'` / `'旧脚本'` / `'待实测'`。导出 C 的文件头会列出所有 `'待实测'` 项。

| 组 | 字段 | 小机器 `local` 来源 | 大机器 `chuanliantui` 来源 |
|---|---|---|---|
| 机体 | `g, wheel_r, half_track(R_l), l_c, m_w, m_l, m_b, I_w, I_b, I_z` | Leg2 mlx 生效组：0.04 / 0.3459 / 0.0501 / 0.13463 / 0.949 / 5.25 / 7.3294e-5 / 0.028669861 / 0.084144108 | `leg_param.m`：0.06 / 0.221 / sqrt(0.015²+0.01²) / 0.3 / 2.0 / 20 / 0.008 / 0.5+20·0.015² / 0.7；全部 `'旧脚本'` 或 `'待实测'` |
| 腿几何 | `leg.lu, leg.lg, leg.len_min, leg.len_max` | 对齐 `machine_config.c`：0.13087 / 0.15240 / 0.13 / 0.23（作者 2026-09-22 把区间从 0.09~0.21 改成 0.13~0.23） | 0.21 / 0.25 / 0.14 / 0.34 |
| 腿质心表 | `leg.data_sjtu5 = [L0 l_wl l_bl I_ll]`；`leg.data_newton15 = [l lw_y lb_y delta Ileg]` | sjtu5 抄 Leg2 `Leg_data` 11 点；newton15 **待作者定**（推荐按五连杆各杆质量 + 解算几何算，或 CAD 导出） | newton15 抄 `leg_param.m` 9 点；sjtu5 由 newton15 换算（`l_wl = sqrt(lw_y²+delta²)` 等） |
| 控制约束 | `ctrl.Ts, ctrl.T_wheel_max, ctrl.T_hip_max, ctrl.grid` | 0.001 / 1.8（`dji_trq_clamp`）/ 10（`dm_trq_clamp`）/ 0.13:0.01:0.23 | 0.001 / 4.8 / 20 / 待定 |

**与板上重叠、必须一致的字段**：`wheel_r`、`leg.lu`、`leg.lg`、`leg.len_min/max`、`T_wheel_max`、`T_hip_max`、`Ts`（= `CTRL_DT`）。`check_machine_config.m` 读 `machine_config.c` 对应机器那段做比对，不一致直接报错。其余质量、惯量板上没有，只在 MATLAB 表维护。

**K 表域**：`ctrl.grid` 的首尾就是 `LQR_K_LEN_MIN/MAX`。阶段 0~2 固定 0.13~0.23 与板上宏一致；要换域先改宏，属代码改动，单独授权。

### 5.2 模型接口

```matlab
[A, B] = model_AB(model_name, m, lL, lR)   % A 10x10, B 10x4，连续时间
```

- 内部找 `cache/AB_<model>_gen.m`，没有就先跑 `model/<model>/build_<model>.m` 生成（`matlabFunction`，参数向量顺序在 `build_*.m` 内写死并注释）。
- 腿长 → 腿质心参数由各模型自己的 `leg_row_*.m` 负责，外面只给 `lL, lR`。
- 加第三个模型 = 加一个子目录，`model_AB` 不改。

### 5.3 导出格式契约（与板上一致，逐字对齐）

- 签名：`void LQR_K_WBR(float lL, float lR, float K_sym[40])`
- 下标：`K_sym[状态*4 + 输出]`，状态 0..9 按 §2.2 顺序，输出 0..3 = T_wl T_wr T_bl T_br。`lqr_balance.c` 按 `K_sym[j*4+i]` 取成 `K[i][j]`，**不许换序**。
- 多项式：poly22，`K = p00 + p10*lL + p01*lR + p20*lL² + p11*lL*lR + p02*lR²`，用 `float` 字面量：先舍入到 single 再以 `%.9g` 打印加 `F` 后缀（与 Coder 一致，C 端解析回同一个 float）。多项式求值写在生成的 C 函数体内，板上只调 `LQR_K_WBR()`，**换阶次不需要改板上代码**。
- 文件头注释：机器名、模型名、Q/R、网格、Ts、拟合阶次、日期、表号、待实测项清单。
- include 行与现文件一致：`#include "lqr_gain_table.h"`；头文件不重新生成。

### 5.4 核对脚本的最低要求

| 脚本 | 判据 |
|---|---|
| `check_vs_ref` | 阶段 0：与 `ref/LQR_K_WBR.m` 在 lL=lR=0.13/0.15/0.18/0.21/0.23 及若干非对称点，240 系数最大相对误差 < 1e-5（只剩打印精度） |
| `check_closed_loop` | 拟合后 K 在全 121 格点：离散闭环谱半径 < 1；拟合残差 max\|K_fit − K_dlqr\| 打印并与 K 量级比 |
| `check_machine_config` | §5.1 重叠字段全部一致 |
| `check_c_compile` | 生成的 C 用 AC5 编过，0 err 0 warn，`.o` 在临时目录 |

### 5.5 调参流程（阶段 1 起固定）

```
1. tune/whatif.m 改 Q/R → 看标称腿长 K、极点、真实层响应
2. run_all.m 出 output/lqr_gain_table.c + report
3. 四个 check 全过
4. 只拷一个文件到 imcalib/Algorithm/lqr_gain_table.c，编译、上板
5. md/sysid-change-map.md 记一条：表号 / Q/R / 现象
```

---

## 六、分阶段任务与验收

### 阶段 0 · 搭管线并复现现表

固定：模型 `sjtu5`，机器 `local`，Q/R = §2.1 那组，网格 0.13:0.01:0.23，腿数据按最近行取（与 mlx 完全一致）。

任务：
1. `config/`：四个机器/模型文件 + `lqr_weights.m` + `machine_load.m`。
2. `model/sjtu5/`：搬 Leg2 5 条方程，`matlabFunction` 缓存。
3. `design/`：`scan_grid.m`、`fit_K.m`。
4. `emit/emit_gain_table.m`。
5. `check/` 四个脚本；`ref/` 放 Leg2 `LQR_K_WBR.m` 副本。
6. `run_all.m` 串起来，一条命令跑完。
7. `.gitignore` 加 `tools/matlab/cache/`。

验收：
- `check_vs_ref` 240 系数逐个对上。
- 其余三个 check 通过。
- 板上代码零改动。

**结果（2026-09-22，表号 local-sjtu5-20260922-0945，变更 85）**：A/B 与 Leg2 `AB_WBR_gen` 相对差 0；拟合 K 与 `ref/LQR_K_WBR.m` 在 121 个网格点最大相对差 3.5e-11、7 个非网格/非对称点 1.3e-11；生成的 C 与板上 C 在 21×21 点 × 40 元素上最大相对差 1.6e-8（float 舍入）；拟合 K 闭环谱半径最大 0.998929（折算 −1.07 1/s）；`check_machine_config`、`check_c_compile`（AC5 0 err 0 warn）通过。板上代码零改动。顺带看到：现表 poly22 对 dlqr 真值的拟合残差最大 0.026（相对 1.5%，位移→右髋一项），是现表自带的误差。

### 阶段 1 · 用它调参，验证可用

任务：
1. `tune/sim_real.m`（搬 PSO v2 `sim_wbr_real`，参数取机器表 `ctrl.*`，延迟拍数默认 1，电机滞后默认 0.003 s 可关）。
2. `tune/whatif.m`：输入 Q/R 缩放或直接给 Q/R，输出标称腿长 K、连续/离散极点表、四个初值场景的响应图（俯仰 5°、0.1 m/s+3°、双腿 2°、8°+0.3 m/s）。
3. 表号机制：`output/report_*.txt` + 文件头表号。

验收：
- 至少一轮"改 Q/R → 上板 → 现象变化能归因"，记进 `sysid-change-map.md`。
- 绕圈问题的处理顺序：先用 VOFA ch3−ch4（偏航角误差是否收敛）、ch27（扶住不动偏航角速度是否为零）排除"环没起作用"和"零偏"，再动偏航角/角速度两列的 Q。

### 阶段 2 · 接自研 15 方程模型

前置：作者定小机器 newton15 格式腿质心表的来源（§5.1）。

任务：
1. `model/newton15/build_newton15.m`：搬 `eq2ss.m` 全部推导（15 方程 → 消元 → 线性化 → `A_orig/B_u/B_x` → `T_acc` → 拼 10 阶），`matlabFunction` 缓存。
2. `leg_row_newton15.m`：断言 `lw_y + lb_y == l`（容差 1e-6）。
3. `check/check_model_pair.m`：同机器、`phi_c=0`、`delta=0`，比 sjtu5 与 newton15 的 A/B：符号全同，量级差异逐元素归到上交模型的简化项，写进 report。
4. 同 Q/R 出两张 K 表，比符号趋势与量级。

验收：
- A/B 比对符号全同；K 趋势一致。
- 两条都过，`model_default` 才允许切到 `newton15` 出表上板；上板仍只换一个文件。

### 阶段 3 · 文档收口

- 新建 `md/LQR_MATLAB.md`：目录、机器表字段、模型接口、导出契约、调参流程、"为什么"。
- `md/LQR_PLAN.md` §七替换成指向它；§1.1 "小机器重算增益必须使用 mlx" 改为指向本管线。
- `md/AGENTS.md` 文件树加 `tools/matlab/`。
- 本文件 §十 进度表补齐。

---

## 七、明确不做的

- 不改 `lqr_balance.c`、`LQR_K_LEN_MIN/MAX`；换域是单独一次授权。升 poly33 只需 `fit_K(S, 3)`，板上无需改（多项式在生成的 C 里，2026-09-22 纠正原计划的说法）。
- 不把 Leg2 `Leg_data` 直接喂 15 方程模型。
- 阶段 0/1 不换模型，阶段 2 不换 Q/R。
- 不删、不改 `leg_matlab_script` 与 Leg2 原件。
- 不用 MATLAB Coder；不生成 `A_L/B_L/U_d` 表（板上无人用）。
- 不在本管线里做 PSO 自动整定（PSO v2 的仿真器可以搬，搜索本身等手动调参跑通后再议）。

---

## 八、已知风险与取舍

| 项 | 风险 | 处理 |
|---|---|---|
| Leg2 `Leg_data` 几何不自洽 | sjtu5 模型下不报错，但它描述的"腿"和实物有出入 | 阶段 0/1 照用以复现现表；阶段 2 用自洽表，比对时把差异归因 |
| 腿数据按最近行取 vs 插值 | mlx 是按下标取；换插值 K 会微变 | 阶段 0 用最近行保证复现；之后可切插值，作为独立变量记录 |
| poly22 残差 | 现表本身对 dlqr 真值的拟合残差就有 max 0.026（相对 1.5%）；新 Q/R 下可能更大 | `check_closed_loop` 打印；不够就 `fit_K(S, 3)` 升 poly33，板上无需改 |
| 大机器参数 | 多数是旧脚本值、未实测 | 表内 `status='待实测'`，导出文件头列出；大机器出表只作预研 |
| `check_machine_config` 解析 C 源 | 正则解析可能被格式变化打断 | 只解析 `.wheel_r/.leg_lu/.leg_lg/.leg_len_min/.leg_len_max/.dji_trq_clamp/.dm_trq_clamp` 七个字段，失败即报错不猜 |
| MATLAB 版本 | 本机 R2024b，Control / Symbolic / Curve Fitting 三个工具箱都有 | `fit_K` 用最小二乘反斜杠，不依赖 Curve Fitting；`build_sjtu5` 首次需 Symbolic（约 13 s，之后走缓存）；`dlqr/c2d` 需 Control |

---

## 九、待作者决定

| # | 事项 | 推荐 | 何时需要 |
|---|---|---|---|
| 1 | 小机器 newton15 格式腿质心表来源 | 按五连杆各杆质量 + 解算几何算（自洽、可复现）；有 CAD 就用 CAD | 阶段 2 前 |
| 2 | 腿数据按最近行取还是插值 | 阶段 0 最近行，阶段 1 起可切插值 | 阶段 1 |
| 3 | 大机器 K 表域 `ctrl.grid` | 待 `leg_len_min/max` 实测后定 | 大机器出表前 |

---

## 十、进度

| 阶段 | 状态 | 日期 | 备注 |
|---|---|---|---|
| 计划 | ✅ 本文件 | 2026-09-21 | 作者确认路线：新建目录、先 Leg2 模型、再 15 方程、机器表可切换 |
| 0 搭管线复现现表 | ✅ 完成 | 2026-09-22 | 五项核对全过，数值见 §六 阶段 0 结果；变更 85 |
| 1 调参验证可用 | ⚪ 未开工 | | |
| 2 接 15 方程模型 | ⚪ 未开工 | | 前置：§九 #1 |
| 3 文档收口 | ⚪ 未开工 | | |

---

## 十一、来源文件索引

| 用途 | 路径 |
|---|---|
| 现表（板上） | `CtrBoard-H7_ALL/imcalib/Algorithm/lqr_gain_table.c/.h` |
| 板上求值与状态定义 | `CtrBoard-H7_ALL/imcalib/Algorithm/lqr_balance.c/.h` |
| 板上机器表 | `CtrBoard-H7_ALL/imcalib/user-lib/machine_config.c/.h` |
| Leg2 建模主脚本 | `Leg2_v1(1)/Leg2_v1/轮腿上交建模MATLAB/WBR_modeling.mlx`（zip 内 `matlab/document.xml` 的 CDATA） |
| Leg2 A/B 缓存写法 | 同目录 `AB_WBR_gen.m`、`PSO_LQR_Tuning_v2.m` Section 2 |
| Leg2 真实层仿真 | `PSO_LQR_Tuning_v2.m` 的 `sim_wbr_real` |
| Leg2 原表 MATLAB 版 | 同目录 `LQR_K_WBR.m` |
| 自研 15 方程推导 | `leg_matlab_script/eq2ss.m` |
| 自研大机器参数 | `leg_matlab_script/leg_param.m` |
| 自研扫描/拟合/导出 | `leg_matlab_script/leg_scan.m`、`lqr_numeric.m` |

---

## 十二、运行方式

命令行（不开 MATLAB 桌面；控制台里的中文会因编码显示成乱码，看 `-logfile` 或 `output/report_<表号>.txt`）：

```
"D:\matlab\bin\matlab.exe" -wait -batch "cd('<仓库>/tools/matlab'); ok = run_all(); exit(double(~ok))" -logfile "<仓库>/tools/matlab/run_all.log"
```

- 退出码 0 = 五项核对全过。桌面里直接 `cd tools/matlab` 后敲 `run_all` 也行，`run_all('local','sjtu5')` 可显式指定机器 / 模型。
- 首次运行 `build_sjtu5` 做符号推导约 13 s；之后 `cache/AB_sjtu5_gen.m` 存在，全程约 5 s。
- 改 Q/R：只改 `config/lqr_weights.m`，**同时换 tag**（tag 还是 `mlx-2026-07-27` 会被当成复现模式，`check_vs_ref` 必然报未通过）。
- 换机器：改 `config/machine_default.m` 一行；`check_machine_config` 会去读 `machine_config.c` 逐项比对，不一致直接报错。
- 上板：核对全过后把 `output/lqr_gain_table.c` 拷到 `imcalib/Algorithm/` 覆盖，`sysid-change-map.md` 记"表号 + Q/R + 现象"。

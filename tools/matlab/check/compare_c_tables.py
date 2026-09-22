"""compare_c_tables.py  new.c  board.c  g0 step g1 [tol]

把两份 lqr_gain_table.c 的 LQR_K_WBR 函数体当直线代码用 Python 执行 (float 字面量按 double 算),
在网格点 + 网格中点上逐元素比对 K_sym[40]。退出码 0 = 最大相对差 <= tol。
兼容 MATLAB Coder 生成的写法 (带 K_sym_tmp 临时量) 与 emit_gain_table.m 的写法。
"""
import re
import sys


def load(path):
    src = open(path, encoding="utf-8", errors="replace").read()
    i = src.index("void LQR_K_WBR")
    i = src.index("{", i)
    j = src.rindex("}")
    body = src[i + 1:j]
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)
    body = re.sub(r"\bfloat\b[^;]*;", "", body)          # 去掉声明
    stmts = [" ".join(s.split()) for s in body.split(";") if s.strip()]
    code = "\n".join(s.replace("F", "") for s in stmts)   # 去掉 float 后缀 (标识符里没有大写 F)

    def f(lL, lR):
        ns = {"lL": float(lL), "lR": float(lR), "K_sym": [0.0] * 40}
        exec(code, ns)
        return ns["K_sym"]

    return f


def main():
    if len(sys.argv) < 6:
        print(__doc__)
        return 2
    new_c, board_c = sys.argv[1], sys.argv[2]
    g0, step, g1 = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
    tol = float(sys.argv[6]) if len(sys.argv) > 6 else 1e-6
    fn, fb = load(new_c), load(board_c)
    n = int(round((g1 - g0) / step)) + 1
    pts = [g0 + k * step for k in range(n)]
    pts += [g0 + (k + 0.5) * step for k in range(n - 1)]
    worst_rel, worst_abs, where = 0.0, 0.0, None
    per_idx = [0.0] * 40
    for lL in pts:
        for lR in pts:
            kn, kb = fn(lL, lR), fb(lL, lR)
            for q in range(40):
                d = abs(kn[q] - kb[q])
                rel = d / max(abs(kb[q]), 1e-6)
                per_idx[q] = max(per_idx[q], rel)
                worst_abs = max(worst_abs, d)
                if rel > worst_rel:
                    worst_rel, where = rel, (lL, lR, q)
    print(f"C vs C: {len(pts)}x{len(pts)} 点 x 40 元素, 最大相对差 {worst_rel:.3e}, 最大绝对差 {worst_abs:.3e}, "
          f"最差 @ lL={where[0]:.3f} lR={where[1]:.3f} K_sym[{where[2]}]")
    bad = [q for q in range(40) if per_idx[q] > tol]
    if bad:
        print("超 tol 的 K_sym 下标:", bad)
    return 0 if worst_rel <= tol else 1


if __name__ == "__main__":
    sys.exit(main())

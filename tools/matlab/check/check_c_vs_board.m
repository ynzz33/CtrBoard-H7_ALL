function ok = check_c_vs_board(new_c, board_c, grid, is_repro)
%CHECK_C_VS_BOARD 生成的 C 与板上现 C 逐点数值比对 (Python 直接执行两个 C 函数体, float 字面量按 double 算)
%   复现模式下最大相对差 <= 1e-6 才通过; 非复现模式只作信息
here = fileparts(mfilename('fullpath'));
py = fullfile(here, 'compare_c_tables.py');
if ~exist(board_c, 'file')
    fprintf('check_c_vs_board: 板上文件不存在 %s, 跳过\n', board_c);  ok = true;  return;
end
cmd = sprintf('py -3 "%s" "%s" "%s" %g %g %g 1e-6', py, new_c, board_c, grid(1), grid(2) - grid(1), grid(end));
[rc, out] = system(cmd);
fprintf('%s', out);
if is_repro
    ok = (rc == 0);
    if ok, fprintf('check_c_vs_board: 通过 (复现模式)\n'); else, fprintf('check_c_vs_board: **未通过** (复现模式)\n'); end
else
    ok = true;
    fprintf('check_c_vs_board: 非复现模式, 只作信息\n');
end
end

function ok = check_c_compile(c_file, repo_root)
%CHECK_C_COMPILE 用工程自己的 AC5 命令编一遍生成的 C (命令取 build/.../compile_commands.json 里 lqr_gain_table.c 那条)
%   -o 改到临时目录, --depend 去掉, 不污染 eIDE 的增量目录 (AGENTS §6)
ok = false;
db = fullfile(repo_root, 'build', 'CtrBoard-H7_ALL', 'compile_commands.json');
if ~exist(db, 'file')
    fprintf('check_c_compile: 找不到 %s, 跳过 (视为未通过)\n', db);  return;
end
cc = jsondecode(fileread(db));
idx = 0;
for k = 1:numel(cc)
    f = strrep(cc(k).file, '\', '/');
    if endsWith(f, '/lqr_gain_table.c'), idx = k; break; end
end
if idx == 0
    fprintf('check_c_compile: compile_commands.json 里没有 lqr_gain_table.c, 跳过 (视为未通过)\n');  return;
end
cmd = cc(idx).command;
tmp = tempname;  mkdir(tmp);
obj = strrep(fullfile(tmp, 'lqr_gain_table.o'), '\', '/');   % 正斜杠: regexprep 替换串里反斜杠是转义
cmd = regexprep(cmd, '(^|\s)-o\s+\S+', sprintf(' -o "%s"', obj));
cmd = regexprep(cmd, '\s--depend\s+\S+', '');
cmd = regexprep(cmd, '\s\S*lqr_gain_table\.c\s*$', sprintf(' "%s"', strrep(c_file, '\', '/')));
old = cd(cc(idx).directory);
restore = onCleanup(@() cd(old));
[rc, out] = system(cmd);
out = strtrim(out);
has_warn = contains(lower(out), 'warning') && ~contains(out, '0 warnings');
ok = (rc == 0) && ~has_warn;
if isempty(out), out = '(无输出)'; end
fprintf('armcc rc=%d  输出: %s\n', rc, out);
if ok, fprintf('check_c_compile: 通过 (.o 在 %s)\n', tmp); else, fprintf('check_c_compile: **未通过**\n'); end
end

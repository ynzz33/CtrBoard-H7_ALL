function result = run_balance14()
%RUN_BALANCE14 使用现役LQR表头Q/R验证融合约化模型，不调参、不覆盖任何固件表。
% MATLAB运行本文件即可；报告仅写入本目录output，原路径退出时恢复。
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(fileparts(here)));
old_path = path;
cleanup = onCleanup(@() path(old_path));
addpath(here,fileparts(here),fullfile(fileparts(here),'lqr10'),fullfile(fileparts(here),'lqr4'));
machine = machine_table();

% 权重读取现役生成C表的来源头，不调用deploy=true的生成入口，也不另存第二份调参值。
gain_file = fullfile(root,'imcalib','Algorithm','lqr','lqr_gain_big.c');
rh_file = fullfile(root,'imcalib','Algorithm','lqr','roll_height_model.c');
t10 = fileread(gain_file); t4 = fileread(rh_file);
q10 = header_numbers(t10,'\*\s*Q\s*:\s*diag\(\[([^\]]+)\]\)',10);
r10 = header_numbers(t10,'\*\s*R\s*:\s*diag\(\[([^\]]+)\]\)',4);
q4 = header_numbers(t4,'Q=\[([^\]]+)\]',4);
r4 = header_numbers(t4,'R=\[([^\]]+)\]',2);
Q = diag([q10;q4]); R = diag([r10;r4]);
cfg = struct('lengths',[0.23;0.27],'Ts',machine.ctrl.Ts, ...
    'Q',Q,'R',R,'q_source',{{gain_file,rh_file}}, ...
    'state_schema','hip-front-v2 + roll/droll/H/dH', ...
    'input_schema','Twl/Twr/TBl/TBr/FL/FR; Tp=-TB', ...
    'deployment',false,'controller','LQR only; current generated-header weights');
fprintf('balance14: current Q=%s\n',mat2str(diag(Q)'));
fprintf('balance14: current R=%s\n',mat2str(diag(R)'));
result = verify_balance14(machine,cfg);
result.config = cfg;
[status,sha] = system(sprintf('git -C "%s" rev-parse HEAD',root));
if status==0, result.source_sha = strtrim(sha); else, result.source_sha = 'unavailable'; end
[dirty_status,dirty_text] = system(sprintf('git -C "%s" status --porcelain',root));
result.source_dirty = dirty_status~=0 || ~isempty(strtrim(dirty_text));
out = fullfile(here,'output');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'result_latest.mat'),'result');
fid = fopen(fullfile(out,'report_latest.txt'),'w','n','UTF-8');
assert(fid>=0,'balance14:report');
closer = onCleanup(@() fclose(fid));
fprintf(fid,'balance14 reduced-model software checks: %s\n',result.status);
fprintf(fid,'Source HEAD: %s (working tree implementation; not a firmware build)\n',result.source_sha);
fprintf(fid,'Q: %s\nR: %s\n',mat2str(diag(Q)'),mat2str(diag(R)'));
fprintf(fid,'Original fixed-model LQR vs merged nominal K max error: %.9g\n',result.nominal_gain_error);
fprintf(fid,'Jacobian max error: %.9g\nAffine anchor max error: %.9g\n',result.jacobian_error,result.affine_error);
fprintf(fid,'Nonzero-angle cross-gain norm: %.9g\n',result.cross_gain_norm);
fprintf(fid,'Tested local lengths/angles: %d\n',numel(result.local_poles));
fprintf(fid,'Nominal lengths [m]: %s; geometric H [m]: %.9g\n',mat2str(cfg.lengths'),mean(cfg.lengths));
fprintf(fid,'Local closed-loop maximum pole magnitude: %.9g\n',max(result.local_poles));
fprintf(fid,'5 s nonlinear closed-loop final/initial state norms: %s\n',mat2str([result.closed_loops.terminal_relative],6));
fprintf(fid,'Mass sum %.9g kg; inherited height equivalent mass %.9g kg; not reconciled by calibration\n',result.mass_sum,result.height_mass);
fprintf(fid,'Scope: frozen-length reduced candidate, LQR Q/R unchanged, no C export or MPC.\n');
fprintf(fid,'No full multibody/variable-length inertia, wheel vertical coupling, compensation-error identification or hardware proof.\n');
fprintf(fid,'Closed-loop tests: ideal compensation, unsaturated nonlinear projection plant; no claim about actuator feasibility.\n');
fprintf('balance14: %s; report %s\n',result.status,fullfile(out,'report_latest.txt'));
end

function numbers = header_numbers(text,pattern,count)
tokens = regexp(text,pattern,'tokens','once');
assert(~isempty(tokens),'balance14:weights','当前生成表缺少Q/R来源头，停止而不猜测权重');
numbers = sscanf(strrep(tokens{1},',',' '),'%f');
assert(numel(numbers)==count && all(isfinite(numbers)) && all(numbers>0),'balance14:weights');
end

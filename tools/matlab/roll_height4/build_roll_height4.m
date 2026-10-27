function [ok,result] = build_roll_height4(CFG,q,r,observer_tau)
%BUILD_ROLL_HEIGHT4 同一机器/运行配置设计LQR与慢变力观测器，检查通过后覆盖。
here=fileparts(mfilename('fullpath'));parent=fileparts(here);repo=fileparts(fileparts(parent));
addpath(parent);
evalc('machine=machine_table(CFG.machine);');
if ~isfield(machine,'roll_height'),error('roll_height4:machine','该机器尚无匹配四状态参数');end
p=machine.roll_height;p.machine=machine.name;p.machine_id=machine.c_id;
config=fullfile(repo,'imcalib','config','machine_config.c');entry=fullfile(here,'run_roll_height4.m');
source=fileread(config);entry_source=fileread(entry);
token=regexp(source,'\.lpf_alpha\s*=\s*\{\s*([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*\}','tokens','once');
if isempty(token),error('roll_height4:filter','找不到四项lpf_alpha（pitch/yaw/roll角速度/高度速度）');end
p.roll_rate_lp=str2double(token{3});p.hdot_lp=str2double(token{4});
for name={'roll_height_bias_common_max','roll_height_bias_diff_max','len_rate'}
 token=regexp(source,['\.' name{1} '\s*=\s*([\d.eE+-]+)f?\s*,'],'tokens','once');
 if isempty(token),error('roll_height4:runtime','找不到machine_config.c的%s',name{1});end
 p.(name{1})=str2double(token{1});
end
p.test_height=[machine.ctrl.grid(1) machine.ctrl.grid(end)];
p.observer_tau=observer_tau;
if numel(q)~=4 || numel(r)~=2 || any(q<0) || any(r<=0),error('roll_height4:weights','Q必须非负、R必须为正，未覆盖固件');end
if numel(observer_tau)~=2 || any(~isfinite(observer_tau)) || any(observer_tau<=0),error('roll_height4:observer','观测时间常数必须为两个正值，未覆盖');end
m=model_roll_height4(p);m.Q=diag(q);m.R=diag(r);
[m.K,m.P]=dlqr(m.Ad,m.Bd,m.Q,m.R);
% 共模/差模只是原模型的坐标分解，使L不引入额外roll/高度串扰。
C=[eye(2) zeros(2,1)];poles=exp(-p.Ts./[observer_tau(1) 1.2*observer_tau(1) observer_tau(2)]);
Ar=[m.Ad(1:2,1:2),m.Bd(1:2,:)*[1;-1];0 0 1];
Ah=[m.Ad(3:4,3:4),m.Bd(3:4,:)*[1;1];0 0 1];
% C端先用上一拍确认输入预测，再用当拍原始状态校正。
Lr=place(Ar',(C*Ar)',poles)';Lh=place(Ah',(C*Ah)',poles)';
m.L=zeros(6,4);m.L(1:2,1:2)=Lr(1:2,:);m.L(3:4,3:4)=Lh(1:2,:);
m.L(5,:)=[Lr(3,:) Lh(3,:)];m.L(6,:)=[-Lr(3,:) Lh(3,:)];
m.Ae=[m.Ad m.Bd;zeros(2,4) eye(2)];m.Ce=[eye(4) zeros(4,2)];
validation=verify_roll_height4(m);
if ~validation.passed,error('roll_height4:validation','四状态/观测器检查失败，未覆盖固件');end
% 作者可能在生成时继续调参；输入变动时保留上次固件表，避免注释/K跨版本。
if ~strcmp(source,fileread(config)) || ~strcmp(entry_source,fileread(entry))
 error('roll_height4:changed','生成期间参数文件发生变化，未覆盖；保存后重新运行');
end
out=fullfile(here,'output');if ~exist(out,'dir'),mkdir(out);end
candidate=fullfile(out,'roll_height_model.c');report=fullfile(out,'report_latest.txt');
export_roll_height4(m,candidate);
fid=fopen(report,'w','n','UTF-8');if fid<0,error('roll_height4:report','无法写报告，未覆盖固件');end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'四状态LQR+慢变力偏置观测\n机器: %s\n模型: %s\n时间: %s\n',CFG.machine,CFG.model,char(datetime('now')));
fprintf(fid,'Q=%s\nR=%s\nM=%g half_track=%g I_roll=%g damping=%g Ts=%g\n',mat2str(q),mat2str(r),p.M,p.half_track,p.I_roll,p.damping,p.Ts);
fprintf(fid,'roll_rate_lp=%g hdot_lp=%g observer_tau=%s\n',p.roll_rate_lp,p.hdot_lp,mat2str(observer_tau));
fprintf(fid,'bias_common_max=%g bias_diff_max=%g\n',p.roll_height_bias_common_max,p.roll_height_bias_diff_max);
fprintf(fid,'K=\n');fprintf(fid,'%.12g %.12g %.12g %.12g\n',m.K');
fprintf(fid,'L=\n');fprintf(fid,'%.12g %.12g %.12g %.12g\n',m.L');
fprintf(fid,'validation=%s\npending=%s\n',jsonencode(validation),p.pending);
clear cleanup;
board=fullfile(repo,'imcalib','Algorithm','roll_height_model.c');
if CFG.deploy
 if ~strcmp(source,fileread(config)) || ~strcmp(entry_source,fileread(entry)),error('roll_height4:changed','写出期间参数变化，未覆盖');end
 [copied,message]=copyfile(candidate,board,'f');
 if ~copied,error('roll_height4:deploy','覆盖失败：%s',message);end
end
ok=true;result=struct('model',m,'validation',validation,'candidate',candidate,'report',report,'board',board,'deployed',logical(CFG.deploy));
fprintf('生成成功\n');
if CFG.deploy,fprintf('已覆盖文件：%s\n',board);else,fprintf('生成文件：%s\n',candidate);end
end

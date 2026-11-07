function report=verify_mpc10(host_compiler)
%VERIFY_MPC10 真实C插值/有限因子与独立凝聚QP对拍；生产数据须先显式部署候选。
if nargin<1||isempty(host_compiler),host_compiler=getenv('MPC_HOST_CC');end
if isempty(host_compiler),host_compiler='zig';end
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(fileparts(here)));
saved=path;cleanup=onCleanup(@()path(saved));addpath(fullfile(fileparts(here),'mpc_common'));
g=mpc10_production_spec(root);N=g.N;Q=g.Q;Su=diag(g.input_scale);
out=fullfile(here,'output');inc=project_includes(root);exe=fullfile(out,'model_test.exe');
sources={fullfile(here,'host_model_test.c'),fullfile(root,'imcalib/Algorithm/mpc/balance_mpc.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/balance_mpc_data.c'),fullfile(root,'imcalib/Algorithm/mpc/mpc_factor.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/mpc_solver.c')};args='';for j=1:numel(sources),args=[args ' "' sources{j} '"'];end
[code,msg]=system(sprintf('"%s" cc -O2 -UNDEBUG%s%s -o "%s"',host_compiler,inc,args,exe));assert(code==0,'%s',msg);
rng(23);errors=[];factor_errors=[];failures=0;iterations=[];shift_errors=[];
for sample=1:112
    if sample<=4,lengths=[.15 .15;.31 .31;.15 .31;.31 .15];lengths=lengths(sample,:)';
    else,lengths=.15+.16*rand(2,1);end
    state=[.01;.1;0;.05;.015;.04;-.015;-.04;.02;.03].*randn(10,1);
    ref=zeros(10,1);slope=[.15;.05].*randn(2,1);ref(2)=slope(1);ref(4)=slope(2);
    bounds=[4.7;4.7;10;10];if mod(sample,3)==0,bounds=[.08;.08;.8;.8];end
    eq=zeros(4,1);applied=.05*randn(4,1);
    input_file=fullfile(out,'input.bin');output_file=fullfile(out,'output.bin');
    f=fopen(input_file,'w');fwrite(f,[lengths;state;ref;slope;-bounds;bounds;eq;applied],'single');fclose(f);
    [code,msg]=system(sprintf('"%s" "%s" "%s"',exe,input_file,output_file));assert(code==0,'%s',msg);
    f=fopen(output_file,'r');v=fread(f,Inf,'single');fclose(f);status=v(1);iterations(end+1)=v(2);
    u=reshape(v(3:2+4*N),4,N);offset=2+4*N;
    A=reshape(v(offset+(1:100)),10,10)';B=reshape(v(offset+100+(1:40)),4,10)';P=reshape(v(offset+140+(1:100)),10,10)';
    raw=v(offset+240+(1:N*40));k_c=permute(reshape(raw,10,4,N),[2 1 3]);
    reference_factors=mpc_build(A,B,Q,eye(4),P,N,v(end));
    factor_errors(end+1)=max(abs(k_c-reference_factors.K),[],'all');
    assert(factor_errors(end)<.02,'mpc10 factorMismatch');
    if status~=0,assert(status==1,'mpc10 numericFailure');failures=failures+1;fprintf('case %d ITER_LIMIT len=%s residual=%g/%g\n',sample,mat2str(lengths'),v(end-2),v(end-1));continue;end
    % 相同C单精度模型上的独立QP，避免把插值误差误判为求解误差。
    initial=state;for delay=1:g.delay_steps,initial=A*initial+B*((applied-eq)./g.input_scale);end
    S=zeros(10*(N+1),4*N);T=zeros(10*(N+1),10);T(1:10,:)=eye(10);
    for k=1:N,rows=k*10+(1:10);prev=(k-1)*10+(1:10);S(rows,:)=A*S(prev,:);S(rows,(k-1)*4+(1:4))=B;T(rows,:)=A*T(prev,:);end
    xr=repmat(ref,1,N+1);xr(1,:)=xr(1,:)+slope(1)*((0:N)+g.delay_steps)*g.Ts;xr(3,:)=xr(3,:)+slope(2)*((0:N)+g.delay_steps)*g.Ts;
    W=blkdiag(kron(eye(N),Q),P);H=S'*W*S+eye(4*N);H=(H+H')/2;
    grad=S'*W*(T*initial-xr(:));lo=repmat(-bounds./g.input_scale,N,1);hi=-lo;
    expected=quadprog(H,grad,[],[],[],[],lo,hi,[],optimoptions('quadprog','Display','off'));
    error=max(abs(u-Su*reshape(expected,4,N)),[],'all');errors(end+1)=error;
    % 相同问题仅改变静态工作点：目标、状态一同平移，物理输入最优序列不变。
    xeq=zeros(10,1);xeq(1)=.7;xeq(5)=.500*lengths(1)-.1045;xeq(7)=.627*lengths(2)-.1208;xeq(9)=.01;
    f=fopen(input_file,'w');fwrite(f,[lengths;state+xeq;ref+xeq;slope;-bounds;bounds;eq;applied;xeq],'single');fclose(f);
    [code,msg]=system(sprintf('"%s" "%s" "%s"',exe,input_file,output_file));assert(code==0,'%s',msg);
    f=fopen(output_file,'r');shifted=fread(f,Inf,'single');fclose(f);
    assert(shifted(1)==0,'mpc10 shifted problem convergence');
    shift_errors(end+1)=max(abs(shifted(3:2+4*N)-u(:)));
    assert(shift_errors(end)<.001,'mpc10 operating point translation mismatch');

    assert(error<.005,'mpc10 QPMismatch');assert(all(abs(u)<=bounds+.0001,'all'),'mpc10 inputBounds');
end
report=struct('QP_cases',numel(errors),'fallback_cases',failures,'max_sequence_error',max(errors), ...
    'max_factor_error',max(factor_errors),'max_iterations',max(iterations),'operating_point_cases',numel(shift_errors),'max_translation_error',max(shift_errors));
disp(report);assert(failures==0,'mpc10 unexpectedFallback');
% 零偏差静态工作点、非零u_eq以及0.7m位置目标都必须保持。
for L=[.16 .22 .31]
    xeq=zeros(10,1);xeq(5)=.500*L-.1045;xeq(7)=.627*L-.1208;
    state=xeq;state(1)=.7;ref=state;eq=[.2;-.1;.3;-.2];bounds=[4;4;20;20];
    f=fopen(input_file,'w');fwrite(f,[[L;L];state;ref;[0;0];-bounds;bounds;eq;eq;xeq],'single');fclose(f);
    [code,msg]=system(sprintf('"%s" "%s" "%s"',exe,input_file,output_file));assert(code==0,'%s',msg);
    f=fopen(output_file,'r');sv=fread(f,Inf,'single');fclose(f);assert(sv(1)==0,'mpc10 equilibrium convergence');
    assert(max(abs(reshape(sv(3:2+4*N),4,N)-eq),[],'all')<.0001,'mpc10 equilibrium input mismatch');
end
report.equilibrium_cases=3;

% 编译实际消息桥；仅RTOS临界区和时钟/机器输入为夹具，不替代算法源。
host=fullfile(out,'host');if ~isfolder(host),mkdir(host);end
f=fopen(fullfile(host,'FreeRTOS.h'),'w');fprintf(f,'#ifndef HOST_RTOS_H\n#define HOST_RTOS_H\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n#endif\n');fclose(f);
f=fopen(fullfile(host,'task.h'),'w');fprintf(f,'#include "FreeRTOS.h"\n');fclose(f);
exe=fullfile(out,'async_test.exe');args='';
files={fullfile(here,'host_async_test.c'),fullfile(root,'imcalib/task/task_balance_mpc.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/balance_mpc.c'),fullfile(root,'imcalib/Algorithm/mpc/balance_mpc_data.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/mpc_solver.c'),fullfile(root,'imcalib/Algorithm/mpc/mpc_factor.c')};
for j=1:numel(files),args=[args ' "' files{j} '"'];end
[code,msg]=system(sprintf('"%s" cc -O2 -UNDEBUG -I"%s"%s -DMACHINE_BALANCE_USE_MPC=1%s -o "%s"',host_compiler,host,inc,args,exe));assert(code==0,'%s',msg);
[code,msg]=system(['"' exe '"']);fprintf('%s',msg);assert(code==0,'mpc10 async mailbox test');
report.async_mailbox=true;
save(fullfile(out,'verification_latest.mat'),'report');disp(report);
end
function includes=project_includes(root)
text=fileread(fullfile(root,'MDK-ARM','CtrBoard-H7_ALL.uvprojx'));tokens=regexp(text,'<IncludePath>([^<]*Algorithm/lqr/inc[^<]*)</IncludePath>','tokens','once');
dirs=strsplit(tokens{1},';');includes='';for j=1:numel(dirs),includes=[includes ' -I"' fullfile(root,'MDK-ARM',dirs{j}) '"'];end
end

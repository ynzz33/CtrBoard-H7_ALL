function reports=tune_mpc10(host_compiler,rhos,alphas,samples)
%VERIFY_MPC10 真实C插值/有限因子与独立凝聚QP对拍；生产数据须先显式部署候选。
if nargin<1||isempty(host_compiler),host_compiler=getenv('MPC_HOST_CC');end
if isempty(host_compiler),host_compiler='zig';end
if nargin<2,rhos=[.1 .3 .6 1 2 3 6];end
if nargin<3,alphas=[1 1.4 1.7 1.9];end
if nargin<4,samples=1:28;end
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(fileparts(here)));
saved=path;cleanup=onCleanup(@()path(saved));addpath(fullfile(fileparts(here),'mpc_common'));
g=mpc10_production_spec(root);N=g.N;Q=g.Q;Su=diag(g.input_scale);
out=fullfile(here,'output');inc=project_includes(root);exe=fullfile(out,'model_test.exe');
sources={fullfile(here,'host_model_test.c'),fullfile(root,'imcalib/Algorithm/mpc/balance_mpc.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/balance_mpc_data.c'),fullfile(root,'imcalib/Algorithm/mpc/mpc_factor.c'), ...
    fullfile(root,'imcalib/Algorithm/mpc/mpc_solver.c')};args='';for j=1:numel(sources),args=[args ' "' sources{j} '"'];end
[code,msg]=system(sprintf('"%s" cc -O2 -UNDEBUG%s%s -o "%s"',host_compiler,inc,args,exe));assert(code==0,'%s',msg);
reports=[];
for rho=rhos
for alpha=alphas
rng(23);errors=[];factor_errors=[];failures=0;iterations=[];
for sample=1:max(samples)
    if sample<=4,lengths=[.15 .15;.31 .31;.15 .31;.31 .15];lengths=lengths(sample,:)';
    else,lengths=.15+.16*rand(2,1);end
    state=[.01;.1;0;.05;.015;.04;-.015;-.04;.02;.03].*randn(10,1);
    ref=zeros(10,1);slope=[.15;.05].*randn(2,1);ref(2)=slope(1);ref(4)=slope(2);
    bounds=[4.7;4.7;10;10];if mod(sample,3)==0,bounds=[.08;.08;.8;.8];end
    eq=zeros(4,1);applied=.05*randn(4,1);
    if ~ismember(sample,samples),continue;end
    input_file=fullfile(out,'input.bin');output_file=fullfile(out,'output.bin');
    f=fopen(input_file,'w');fwrite(f,[lengths;state;ref;slope;-bounds;bounds;eq;applied],'single');fclose(f);
    [code,msg]=system(sprintf('"%s" "%s" "%s" %.9g %.9g',exe,input_file,output_file,rho,alpha));assert(code==0,'%s',msg);
    f=fopen(output_file,'r');v=fread(f,Inf,'single');fclose(f);status=v(1);iterations(end+1)=v(2);
    u=reshape(v(3:2+4*N),4,N);offset=2+4*N;
    A=reshape(v(offset+(1:100)),10,10)';B=reshape(v(offset+100+(1:40)),4,10)';P=reshape(v(offset+140+(1:100)),10,10)';
    raw=v(offset+240+(1:N*40));k_c=permute(reshape(raw,10,4,N),[2 1 3]);
    reference_factors=mpc_build(A,B,Q,eye(4),P,N,v(end));
    factor_errors(end+1)=max(abs(k_c-reference_factors.K),[],'all');
    assert(factor_errors(end)<.02,'mpc10 factorMismatch');
    if status~=0,assert(status==1,'mpc10 numericFailure');failures=failures+1;continue;end
    % 相同C单精度模型上的独立QP，避免把插值误差误判为求解误差。
    initial=state;for delay=1:g.delay_steps,initial=A*initial+B*((applied-eq)./g.input_scale);end
    S=zeros(10*(N+1),4*N);T=zeros(10*(N+1),10);T(1:10,:)=eye(10);
    for k=1:N,rows=k*10+(1:10);prev=(k-1)*10+(1:10);S(rows,:)=A*S(prev,:);S(rows,(k-1)*4+(1:4))=B;T(rows,:)=A*T(prev,:);end
    xr=repmat(ref,1,N+1);xr(1,:)=xr(1,:)+slope(1)*((0:N)+g.delay_steps)*g.Ts;xr(3,:)=xr(3,:)+slope(2)*((0:N)+g.delay_steps)*g.Ts;
    W=blkdiag(kron(eye(N),Q),P);H=S'*W*S+eye(4*N);H=(H+H')/2;
    grad=S'*W*(T*initial-xr(:));lo=repmat(-bounds./g.input_scale,N,1);hi=-lo;
    expected=quadprog(H,grad,[],[],[],[],lo,hi,[],optimoptions('quadprog','Display','off'));
    error=max(abs(u-Su*reshape(expected,4,N)),[],'all');errors(end+1)=error;
    assert(error<.005,'mpc10 QPMismatch');assert(all(abs(u)<=bounds+.0001,'all'),'mpc10 inputBounds');
end
entry=[rho alpha failures max(iterations) median(iterations) max([0 errors])];reports(end+1,:)=entry;
fprintf('rho=%g alpha=%g failures=%d max=%d median=%.1f err=%g\n',entry);
end
end
horizon=g.N;save(fullfile(out,'tuning_latest.mat'),'reports','horizon','samples');
end
function includes=project_includes(root)
text=fileread(fullfile(root,'MDK-ARM','CtrBoard-H7_ALL.uvprojx'));tokens=regexp(text,'<IncludePath>([^<]*Algorithm/lqr/inc[^<]*)</IncludePath>','tokens','once');
dirs=strsplit(tokens{1},';');includes='';for j=1:numel(dirs),includes=[includes ' -I"' fullfile(root,'MDK-ARM',dirs{j}) '"'];end
end

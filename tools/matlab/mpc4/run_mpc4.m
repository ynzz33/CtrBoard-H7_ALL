function result=run_mpc4(host_compiler)
%RUN_MPC4 原四态模型/Q/R，独立凝聚QP对照真实C求解器；不覆盖原LQR表。
here=fileparts(mfilename('fullpath')); parent=fileparts(here);
saved=path; cleanup=onCleanup(@()path(saved));
if nargin<1 || isempty(host_compiler),host_compiler=getenv('MPC_HOST_CC');end
if isempty(host_compiler),host_compiler='zig';end
addpath(parent,fullfile(parent,'lqr4'),fullfile(parent,'mpc_common'));
solve_div=2; N=10; rho=.0001;
m=machine_table('big_wheelleg'); p=m.roll_height; p.Ts=m.ctrl.Ts*solve_div; md=model_roll_height4(p);
root=fileparts(fileparts(parent));
source=fileread(fullfile(root,'imcalib','Algorithm','lqr','roll_height_model.c'));
qt=regexp(source,'Q=\[([^\]]+)\]','tokens','once');rt=regexp(source,'R=\[([^\]]+)\]','tokens','once');
assert(~isempty(qt) && ~isempty(rt),'mpc4:missingCurrentWeights');
qv=sscanf(qt{1},'%f');rv=sscanf(rt{1},'%f');assert(numel(qv)==4 && numel(rv)==2);
Q=diag(qv);R=diag(rv);
[lqr_gain,P]=dlqr(md.Ad,md.Bd,Q,R);
c=mpc_build(md.Ad,md.Bd,Q,R,P,N,rho);
c.lqr_gain=lqr_gain;
out=fullfile(here,'output'); if ~isfolder(out),mkdir(out);end
solver=fullfile(root,'imcalib','Algorithm','mpc','mpc_solver.c');
inc=fullfile(fileparts(solver),'inc');
wrapper=fullfile(out,'mpc_host.c');
fid=fopen(wrapper,'w'); assert(fid>=0);
fprintf(fid,['#include "mpc_solver.h"\n#include <stdio.h>\n#include <stdlib.h>\n' ...
'int main(int argc,char **argv){FILE *in=fopen(argv[1],"rb");if(!in)return 2;unsigned short dims[3];if(fread(dims,2,3,in)!=3)return 3;int n=dims[0],a=dims[1],N=dims[2];\n' ...
'int sizes[13]={n*n,n*a,n*n,(N+1)*n*n,N*a*n,N*a*a,1,n,(N+1)*n,N*a,N*n,N*a,N*a};float *v[13];for(int j=0;j<13;j++){v[j]=calloc(sizes[j],4);if(fread(v[j],4,sizes[j],in)!=sizes[j])return 4;}unsigned char constrained;if(fread(&constrained,1,1,in)!=1)return 5;fclose(in);\n' ...
'mpc_model_t m={dims[0],dims[1],dims[2],v[0],v[1],v[2],v[3],v[4],v[5],v[6][0]};mpc_problem_t p={&m,v[7],v[8],v[9],v[10],v[11],v[12],30,0.002f,0.0000002f,constrained};mpc_workspace_t w;w.capacity=MPC_Workspace_Size(&m);w.storage=calloc(w.capacity,4);mpc_result_t r;MPC_Solve(&p,&w,&r);\n' ...
'FILE *o=fopen(argv[2],"wb");float status=(float)r.status,it=(float)r.iterations;fwrite(&status,4,1,o);fwrite(&it,4,1,o);if(r.input)fwrite(r.input,4,N*a,o);fclose(o);return 0;}\n']);
fclose(fid);
zig=host_compiler;
exe=fullfile(out,'mpc_host.exe');
cmd=sprintf('"%s" cc -O2 -UNDEBUG -I"%s" "%s" "%s" -o "%s"',zig,inc,wrapper,solver,exe);
[status,msg]=system(cmd);assert(status==0,'%s',msg);
% 凝聚QP独立构建预测矩阵，不复用Riccati求解公式。
nx=4;nu=2; S=zeros((N+1)*nx,N*nu); T=zeros((N+1)*nx,nx);T(1:nx,:)=eye(nx);
for k=1:N
    rows=k*nx+(1:nx);prev=(k-1)*nx+(1:nx);
    T(rows,:)=md.Ad*T(prev,:);S(rows,:)=md.Ad*S(prev,:);S(rows,(k-1)*nu+(1:nu))=md.Bd;
end
W=blkdiag(kron(eye(N),Q),P); H=S'*W*S+kron(eye(N),R); H=(H+H')/2;
initial=[.02;-.05;-.015;.03]; q=zeros(nx,N+1); r=zeros(nu,N);f=zeros(nx,N);
gradient=S'*W*T*initial;
err=zeros(1,2);iters=zeros(1,2);
for constrained=0:1
    lo=-10*ones(nu,N);hi=10*ones(nu,N);
    if constrained,expected=quadprog(H,gradient,[],[],[],[],lo(:),hi(:),[],optimoptions('quadprog','Display','off'));
    else,expected=-H\gradient;end
    [got,status,iters(constrained+1)]=host_solve(exe,out,nx,nu,N,flat(c.A),flat(c.B),flat(c.Q),flat(c.P),flat(c.K),flat(c.Hinv),rho,single(initial),single(q(:)),single(r(:)),single(f(:)),single(lo(:)),single(hi(:)),constrained);
    assert(status==0,'mpc4:C did not converge');
    err(constrained+1)=max(abs(got-expected));
    assert(err(constrained+1)<.05,'mpc4:C/QP mismatch');
end
result=struct('max_sequence_error',err,'iterations',iters,'model',c,'Q',Q,'R',R, ...
    'node_period',md.p.Ts,'input_intervals',N,'solve_div',solve_div);
% 非零线性代价/仿射和逐节点边界，覆盖参考变化而非仅零参考调节。
rng(11); extra_error=zeros(1,16); extra_iterations=zeros(1,16);
for sample=1:16
    initial=[.02;.08;.015;.03].*randn(4,1);
    ref=zeros(4,N+1);ref(3,:)=linspace(0,.01,N+1);ref(4,:)=.01;
    q=-Q*ref;q(:,end)=-P*ref(:,end);
    f=repmat([0;0;.00001;0],1,N);r=.00005*randn(2,N);
    cf=zeros((N+1)*nx,1);
    for k=1:N,cf(k*nx+(1:nx))=md.Ad*cf((k-1)*nx+(1:nx))+f(:,k);end
    gradient=S'*(W*(T*initial+cf)+q(:))+r(:);
    lo=-20*ones(2,N);hi=15*ones(2,N);
    expected=quadprog(H,gradient,[],[],[],[],lo(:),hi(:),[],optimoptions('quadprog','Display','off'));
    [got,status,extra_iterations(sample)]=host_solve(exe,out,nx,nu,N,flat(c.A),flat(c.B),flat(c.Q),flat(c.P),flat(c.K),flat(c.Hinv),rho,single(initial),single(q(:)),single(r(:)),single(f(:)),single(lo(:)),single(hi(:)),1);
    assert(status==0,'mpc4:dynamicProblemDidNotConverge');
    extra_error(sample)=max(abs(got-expected));assert(extra_error(sample)<.05,'mpc4:dynamicProblemMismatch');
end
result.dynamic_error=extra_error;result.dynamic_iterations=extra_iterations;
fprintf('dynamic cases: %d, max error %.6g N, max iterations %d\n',numel(extra_error),max(extra_error),max(extra_iterations));
% 同一真实C核心处理10态4输入；仅复用验证，不接入机器人十态后端。
addpath(fullfile(parent,'lqr10'));
[A10,B10]=model_AB('sjtu5',m,.23,.27);[A10,B10]=c2d(A10,B10,m.ctrl.Ts);
Q10=diag([2000 700 200 540 90000 4000 90000 4000 200000 1000]);R10=diag([500 500 50 50]);
[K10,P10]=dlqr(A10,B10,Q10,R10);c10=mpc_build(A10,B10,Q10,R10,P10,12,1);
x10=[.01;.05;zeros(6,1);.01;0];zq=zeros(10,13);zr=zeros(4,12);zf=zeros(10,12);
[u10,status,n10]=host_solve(exe,out,10,4,12,flat(c10.A),flat(c10.B),flat(c10.Q),flat(c10.P),flat(c10.K),flat(c10.Hinv),c10.rho,single(x10),single(zq(:)),single(zr(:)),single(zf(:)),single(-ones(48,1)),single(ones(48,1)),0);
assert(status==0 && max(abs(u10(1:4)+K10*x10))<.005,'mpc4:10stateReuse');
result.ten_state_first_error=max(abs(u10(1:4)+K10*x10));result.ten_state_iterations=n10;
fprintf('10/4 reuse: first input error %.6g Nm, iterations %d\n',result.ten_state_first_error,n10);
save(fullfile(out,'result_latest.mat'),'result');disp(result.max_sequence_error);disp(result.iterations);
mpc_export(c,fullfile(out,'roll_height_mpc_data.c'));
fid=fopen(fullfile(out,'roll_height_mpc_data.h'),'w');assert(fid>=0);
fprintf(fid,['/* Generated by run_mpc4; deploy with the matching C data. */\n' ...
    '#ifndef ROLL_HEIGHT_MPC_DATA_H\n#define ROLL_HEIGHT_MPC_DATA_H\n#include "mpc_solver.h"\n' ...
    '#define RH_MPC_N %du\n#define RH_MPC_TS %.9ef\n' ...
    '#define RH_MPC_WORK_SIZE MPC_WORKSPACE_FLOATS(4u,2u,RH_MPC_N)\n' ...
    'extern const mpc_model_t roll_height_mpc_model;\n' ...
    'extern const float roll_height_mpc_model_lqr_gain[8];\n' ...
    'extern const float roll_height_mpc_model_input_weight[4];\n#endif\n'],N,md.p.Ts);
fclose(fid);
end
function v=flat(M)
v=single(reshape(permute(M,[2 1 3]),[],1));
end

function [got,status,iters]=host_solve(exe,out,nx,nu,N,varargin)
infile=fullfile(out,'input.bin'); outfile=fullfile(out,'output.bin');
f=fopen(infile,'w');fwrite(f,[nx nu N],'uint16');
for k=1:13,fwrite(f,varargin{k},'single');end
fwrite(f,varargin{14},'uint8');fclose(f);
[code,msg]=system(sprintf('"%s" "%s" "%s"',exe,infile,outfile));assert(code==0,'%s',msg);
f=fopen(outfile,'r');status=fread(f,1,'single');iters=fread(f,1,'single');got=fread(f,Inf,'single');fclose(f);
end

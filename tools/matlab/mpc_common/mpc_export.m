function mpc_export(c,filename,symbol,header)
%MPC_EXPORT 只导出新增MPC数据；原LQR表不覆盖。
if nargin<3,symbol='roll_height_mpc_model';end
if nargin<4,header='roll_height_mpc_data.h';end
[nx,nu]=size(c.B);
f=fopen(filename,'w');assert(f>=0);close=onCleanup(@()fclose(f));
fprintf(f,'/* Generated finite-horizon input-only ADMM factors; original physical terminal P. */\n#include "%s"\n',header);
c.Ct=zeros(nx,nx,c.N);for j=1:c.N,c.Ct(:,:,j)=(c.A-c.B*c.K(:,:,j))';end
names={'A','B','Q','P','K','Hinv','Ct'};
for k=1:numel(names)
    name=names{k};v=reshape(permute(c.(name),[2 1 3]),[],1);
    fprintf(f,'static const float data_%s[%d] = {\n',name,numel(v));
    for j=1:numel(v),fprintf(f,'%.9ef,',single(v(j)));if mod(j,8)==0,fprintf(f,'\n');end;end
    fprintf(f,'\n};\n');
end
fprintf(f,'const mpc_model_t %s = {%d,%d,%d,data_A,data_B,data_Q,data_P,data_K,data_Hinv,%.9ef,data_Ct};\n',symbol,nx,nu,c.N,single(c.rho));
if isfield(c,'R')
    v=reshape(c.R',[],1);fprintf(f,'const float %s_input_weight[%d] = {',symbol,numel(v));
    fprintf(f,'%.9ef,',single(v));fprintf(f,'};\n');
end
if isfield(c,'lqr_gain')
    v=reshape(c.lqr_gain',[],1);fprintf(f,'const float %s_lqr_gain[%d] = {',symbol,numel(v));
    fprintf(f,'%.9ef,',single(v));fprintf(f,'};\n');
end
end

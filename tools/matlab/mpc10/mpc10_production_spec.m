function g=mpc10_production_spec(root)
% 读取正在编译的生产头/常量，避免候选MAT的N与手工部署版本不一致。
h=fileread(fullfile(root,'imcalib/Algorithm/mpc/inc/balance_mpc_data.h'));
s=fileread(fullfile(root,'imcalib/Algorithm/mpc/inc/balance_mpc.h'));
g.N=str2double(regexp(h,'#define BALANCE_MPC_N\s+(\d+)u','tokens','once'));
g.Ts=str2double(regexp(h,'#define BALANCE_MPC_TS\s+([0-9.eE+-]+)f','tokens','once'));
g.delay_steps=str2double(regexp(s,'#define BALANCE_MPC_SOLVE_DIV\s+(\d+)u','tokens','once'));
c=fileread(fullfile(root,'imcalib/Algorithm/mpc/balance_mpc_data.c'));
g.Q=reshape(array(c,'balance_mpc_Q'),10,10)';g.input_scale=array(c,'balance_mpc_input_scale');
end
function v=array(text,name)
a=regexp(text,[name '\[\d+\]\s*=\s*\{([^}]+)\}'],'tokens','once');assert(~isempty(a),'mpc10 production array missing');
v=sscanf(regexprep(a{1},'[f,]',' '),'%f');
end

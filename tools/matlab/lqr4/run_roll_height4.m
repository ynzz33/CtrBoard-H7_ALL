function varargout = run_roll_height4(machine_name)
%RUN_ROLL_HEIGHT4 四状态LQR唯一调参/出表入口，结构对应lqr10/run_all。
% 直接运行：run_roll_height4；可选[ok,result]=run_roll_height4。
% 检查通过后生成候选并覆盖固件C数据；Q/R和观测时间常数只改这里。
CFG.machine='big_wheelleg';
CFG.model='roll_height4_lqr';
CFG.deploy=true;
if nargin>=1 && ~isempty(machine_name),CFG.machine=machine_name;end
% 状态顺序：roll、roll角速度、H、Hdot；Q/R以当前调参值为准，名义检查通过不等于实机稳定。
q=[17 0.01 500 0.01]; % 仅增强roll回正，高度及观测时间常数保持
r=[1e-4 1e-4];
observer_tau=[0.02 0.5]; % s：物理状态估计/慢变力偏置；按当前1kHz重新设计L
addpath(fileparts(mfilename('fullpath')));
[ok,result]=build_roll_height4(CFG,q,r,observer_tau);
if nargout>=1,varargout{1}=ok;end
if nargout>=2,varargout{2}=result;end
end

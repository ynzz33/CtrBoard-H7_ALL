function [total,info]=solve_roll_height4(m,x,target,support,turn_ff,bias)
%SOLVE_ROLL_HEIGHT4 与C一致：原支撑前馈+显式向心项-bias-K*(x-target)。
info.base_force=support(:)+[-turn_ff;turn_ff]-bias(:);
info.feedback=-m.K.*repmat((x-target)',2,1);
total=info.base_force+sum(info.feedback,2);
info.valid=all(isfinite(total));
if ~info.valid,total=zeros(2,1);end
end

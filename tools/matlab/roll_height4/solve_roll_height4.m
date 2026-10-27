function [total,info]=solve_roll_height4(m,x,reference,support,turn_ff,bias)
%SOLVE_ROLL_HEIGHT4 与C一致：原支撑前馈+显式向心项-bias-K*(x-ref)。
info.equilibrium=support(:)+[-turn_ff;turn_ff]-bias(:);
info.feedback=-m.K.*repmat((x-reference)',2,1);
total=info.equilibrium+sum(info.feedback,2);
info.valid=all(isfinite(total));info.result=double(~info.valid);
if ~info.valid,total=zeros(2,1);end
end

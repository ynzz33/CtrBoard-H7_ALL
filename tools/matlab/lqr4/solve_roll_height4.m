function [total,info]=solve_roll_height4(m,x,target,support,turn_ff,bias)
%SOLVE_ROLL_HEIGHT4 与C一致：原请求与下限协调；未触限不改原值。
info.base_force=support(:)+[-turn_ff;turn_ff]-bias(:);
info.feedback=-m.K.*repmat((x-target)',2,1);
raw=info.base_force+sum(info.feedback,2);
info.raw_force=raw;info.common_shift=0;info.difference_request=.5*raw(1)-.5*raw(2);
info.valid=all(isfinite(raw));
if ~info.valid,total=zeros(2,1);return;end
total=raw;
if any(raw<m.p.force_min)
 common=.5*raw(1)+.5*raw(2);
 bounded=max(common,m.p.force_min+abs(info.difference_request));
 info.common_shift=bounded-common;
 total=[bounded+info.difference_request;bounded-info.difference_request];
end
info.valid=all(isfinite(total)) && isfinite(info.common_shift);
if ~info.valid,total=zeros(2,1);end
end

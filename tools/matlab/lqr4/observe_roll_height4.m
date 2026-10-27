function o=observe_roll_height4(m,o,measurement,previous_input,permit)
%OBSERVE_ROLL_HEIGHT4 与C同序：上一拍已确认F预测，当拍原始状态校正。
o.updated=false;o.limited=false;
if ~permit || ~o.seeded
 o.state=measurement;o.seeded=logical(permit);return;
end
predicted=m.Ad*o.state+m.Bd*(previous_input+o.bias)+m.fd;
next=[predicted;o.bias]+m.L*(measurement-predicted);
assert(all(isfinite(next)),'观测器非有限结果');
common=mean(next(5:6));difference=next(5)-next(6);
cm=min(max(common,-m.p.roll_height_bias_common_max),m.p.roll_height_bias_common_max);
df=min(max(difference,-m.p.roll_height_bias_diff_max),m.p.roll_height_bias_diff_max);
o.state=next(1:4);o.bias=cm+[df;-df]/2;o.updated=true;o.limited=cm~=common || df~=difference;
end

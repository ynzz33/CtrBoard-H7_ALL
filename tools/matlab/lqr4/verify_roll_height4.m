function report=verify_roll_height4(m)
%VERIFY_ROLL_HEIGHT4 物理模型、滤波、观测器及假设延迟的数值检查；不代表实机。
p=m.p;x0=[0;0;p.H0;0];support=p.M*p.g/2*ones(2,1);
assert(norm(m.Ad*x0+m.Bd*support+m.fd-x0,inf)<1e-10,'静态平衡错误');
assert(rank(ctrb(m.Ad,m.Bd))==4,'模型不可控');
report.pole=max(abs(eig(m.Ad-m.Bd*m.K)));
A=double(single(m.Ad));B=double(single(m.Bd));K=double(single(m.K));
report.float_pole=max(abs(eig(A-B*K)));
report.delay_ms=[0 1 3 5 10];report.delay_poles=zeros(1,5);report.observer_delay_poles=zeros(1,5);
C=diag([1 p.roll_rate_lp 1 p.hdot_lp]);D=zeros(4,2);
D(2,1)=1-p.roll_rate_lp;D(4,2)=1-p.hdot_lp;
F=[0 p.roll_rate_lp 0 0;0 0 0 p.hdot_lp];G=diag([1-p.roll_rate_lp 1-p.hdot_lp]);
for i=1:numel(report.delay_ms)
 d=round(report.delay_ms(i)*.001/p.Ts);
 if d==0
  closed=[A-B*K*C -B*K*D;F G];
 else
  closed=zeros(6+2*d);closed(1:4,1:4)=A;closed(1:4,5:6)=B;
  if d>1,closed(5:2+2*d,7:4+2*d)=eye(2*(d-1));end
  closed(3+2*d:4+2*d,1:4)=-K*C;closed(3+2*d:4+2*d,5+2*d:6+2*d)=-K*D;
  closed(5+2*d:6+2*d,1:4)=F;closed(5+2*d:6+2*d,5+2*d:6+2*d)=G;
 end
 report.delay_poles(i)=max(abs(eig(closed)));
 % 物理x、控制低通2项、后验估计4项、偏置2项、上一拍确认输入2项、延迟队列。
 n=14+2*d;joint=zeros(n);mf=m;mf.Ad=A;mf.Bd=B;mf.K=K;mf.L=double(single(m.L));
 for j=1:n
  basis=zeros(n,1);basis(j)=1;joint(:,j)=joint_step(basis,mf,d);
 end
 report.observer_delay_poles(i)=max(abs(eig(joint)));
end
report.observer_pole=max(abs(eig((eye(6)-m.L*m.Ce)*m.Ae)));
Aef=double(single(m.Ae));Lf=double(single(m.L));
report.observer_float_pole=max(abs(eig((eye(6)-Lf*m.Ce)*Aef)));
assert(report.pole<1 && report.float_pole<1 && all(report.delay_poles<1),'含滤波/延迟的闭环不稳定，未覆盖');
assert(report.observer_pole<1 && report.observer_float_pole<1 && all(report.observer_delay_poles<1),'含观测器/滤波/延迟的闭环不稳定，未覆盖');
E=expm([m.Ac m.Bc m.fc;zeros(3,7)]*p.Ts);
assert(norm(E(1:4,1:4)-m.Ad,inf)<1e-12 && norm(E(1:4,5:6)-m.Bd,inf)<1e-12,'ZOH不一致');
geom=zeros(1,17);
for i=1:17
 t=(i-1)*.13;dt=1e-6;
 [L,Ld,a,ad,r,rd]=trajectory(t);[~,v]=m.height(L,Ld,a,ad,r,rd);
 [L1,Ld1,a1,ad1,r1,rd1]=trajectory(t+dt);[L2,Ld2,a2,ad2,r2,rd2]=trajectory(t-dt);
 geom(i)=abs(v-(m.height(L1,Ld1,a1,ad1,r1,rd1)-m.height(L2,Ld2,a2,ad2,r2,rd2))/(2*dt));
end
report.geometry_error=max(geom);assert(report.geometry_error<1e-8,'几何速度错误');
[eq,info]=solve_roll_height4(m,x0,x0,support,0,zeros(2,1));
assert(info.valid && norm(eq-support,inf)<1e-9,'支撑前馈错误');
[turn,~]=solve_roll_height4(m,x0,x0,support,10,zeros(2,1));
assert(norm(turn-eq-[-10;10],inf)<1e-9,'显式向心项被抵消');
[comp,~]=solve_roll_height4(m,x0,x0,support,0,[44;40]);
assert(norm(comp-eq+[44;40],inf)<1e-9,'偏置补偿方向错误');
[low,~]=solve_roll_height4(m,x0-[0;0;.001;0],x0,support,0,zeros(2,1));
[high,~]=solve_roll_height4(m,x0+[0;0;.001;0],x0,support,0,zeros(2,1));
assert(all(low>eq) && all(high<eq),'高度反馈方向错误');
[roll,~]=solve_roll_height4(m,x0+[.01;0;0;0],x0,support,0,zeros(2,1));
assert(roll(1)<eq(1) && roll(2)>eq(2),'roll反馈方向错误');
[~,bad]=solve_roll_height4(m,[NaN;0;p.H0;0],x0,support,0,zeros(2,1));assert(~bad.valid,'坏结果未拒绝');
% 下限非线性另测；上面的特征值仅描述未触限平衡点附近。
requests=[50 60;-30 -20;20 -30;-10 -10;-300 200;200 -300]';
report.lower_bound_max_difference_error=0;
for i=1:size(requests,2)
 raw=requests(:,i);[bounded,allocation]=solve_roll_height4(m,x0,x0,raw,0,zeros(2,1));
 assert(allocation.valid && all(bounded>=p.force_min-1e-9),'轴向力下限未满足');
 err=abs((bounded(1)-bounded(2))-(raw(1)-raw(2)));
 report.lower_bound_max_difference_error=max(report.lower_bound_max_difference_error,err);
 assert(err<1e-9,'下限协调损失差模');
 if all(raw>=p.force_min),assert(isequal(bounded,raw) && allocation.common_shift==0,'未触限改变原请求');end
end
report.nominal=simulate(m,'nominal');report.bias=simulate(m,'bias');
report.mismatch=simulate(m,'mismatch');report.noise=simulate(m,'noise');report.gated=simulate(m,'gated');
report.uncompensated_bias_error_prediction=44.3/mean(m.K(:,3));
report.bias_within_10mm=report.bias.final_error<.01;
assert(report.nominal.final_error<.001 && report.nominal.final_roll<.001,'名义响应未回稳');
assert(report.bias.final_error<.001 && report.bias.final_roll<.001 && report.bias.bias_error<1,'恒力偏置未补偿');
assert(report.mismatch.final_error<.005 && report.mismatch.final_roll<.005,'模型偏差响应未回稳');
assert(report.noise.bias_noise_std<2 && report.noise.final_error<.002,'高频测量噪声导致偏置漂移');
assert(report.gated.frozen_bias_change==0 && report.gated.final_error<.001,'冻结期间学习偏置或恢复后未回稳');
o=initial_observer(x0);o.bias=[10;6];
for k=1:10
 o=observe_roll_height4(m,o,x0+[.1;1;.1;1],support,false);
 assert(~o.updated && ~o.seeded && all(o.bias==[10;6]),'无有效输入时不应学习');
end
o=observe_roll_height4(m,o,x0,support,true);assert(o.seeded && ~o.updated,'恢复首拍必须重对齐');
o.bias=1.2*p.roll_height_bias_common_max*[1;1];o=observe_roll_height4(m,o,x0,support,true);
assert(o.limited && abs(mean(o.bias))<=p.roll_height_bias_common_max+1e-9,'共模偏置界限错误');
o.bias=[60;-60];o=observe_roll_height4(m,o,x0,support,true);
assert(o.limited && abs(o.bias(1)-o.bias(2))<=p.roll_height_bias_diff_max+1e-9,'差模偏置界限错误');
report.passed=true;
end
function next=joint_step(s,m,d)
x=s(1:4);rate=s(5:6);hat=s(7:10);bias=s(11:12);previous=s(13:14);
rate=rate+[m.p.roll_rate_lp;m.p.hdot_lp].*([x(2);x(4)]-rate);
pred=m.Ad*hat+m.Bd*(previous+bias);corrected=[pred;bias]+m.L*(x-pred);
u=-corrected(5:6)-m.K*[x(1);rate(1);x(3);rate(2)];
if d==0,force=u;queue=[];else,force=s(15:16);queue=[s(17:end);u];end
next=[m.Ad*x+m.Bd*force;rate;corrected;u;queue];
end
function metrics=simulate(m,scenario)
p=m.p;plant=m;
if strcmp(scenario,'mismatch')
 pp=p;pp.M=1.05*p.M;pp.I_roll=.8*p.I_roll;pp.damping=1.5*p.damping;plant=model_roll_height4(pp);
end
h0=p.test_height(1);h1=p.test_height(2);x=[.03;0;h0;0];rate=zeros(2,1);height_ramp_target=h0;height_rate_target=0;
n=round(8/p.Ts);forces=zeros(2,n);bias_history=zeros(2,n);o=initial_observer(x);
previous=p.M*p.g/2*ones(2,1);frozen_change=0;last=o.bias;disturbance=zeros(2,1);
for k=1:n
 t=(k-1)*p.Ts;height_target=h0+(h1-h0)*double(t>=.5&&t<2);
 [height_ramp_target,height_rate_target]=reference_step(height_ramp_target,height_rate_target,height_target,p.len_rate,p.length_ref_accel_max,p.Ts);
 measured=x;
 if strcmp(scenario,'noise'),measured=measured+[1e-4*sin(2*pi*50*t);.15*sin(2*pi*40*t);.0002*cos(2*pi*45*t);.03*cos(2*pi*35*t)];end
 rate=rate+[p.roll_rate_lp;p.hdot_lp].*([measured(2);measured(4)]-rate);
 permit=~(strcmp(scenario,'gated')&&t>=2.2&&t<2.5);
 o=observe_roll_height4(m,o,measured,previous,permit);
 if ~permit,frozen_change=max(frozen_change,max(abs(o.bias-last)));end
 u=solve_roll_height4(m,[measured(1);rate(1);measured(3);rate(2)],[0;0;height_ramp_target;height_rate_target],p.M*p.g/2*ones(2,1),0,o.bias);
 if t>=1.5 && (strcmp(scenario,'bias')||strcmp(scenario,'gated')),disturbance=[47.3;41.3];end
 forces(:,k)=u;bias_history(:,k)=o.bias;x=plant.Ad*x+plant.Bd*(u+disturbance)+plant.fd;
 previous=u;last=o.bias;
end
assert(all(isfinite(x))&&all(isfinite(forces),'all'),'响应非有限');
metrics=struct('final_error',abs(x(3)-height_ramp_target),'final_roll',abs(x(1)), ...
 'force_min',min(forces(:)),'force_max',max(forces(:)),'bias_final',o.bias', ...
 'bias_error',norm(o.bias-disturbance,inf),'bias_noise_std',max(std(bias_history(:,end-round(1/p.Ts):end),0,2)), ...
 'frozen_bias_change',frozen_change);
end
function o=initial_observer(x)
o=struct('state',x,'bias',zeros(2,1),'seeded',false,'updated',false,'limited',false);
end
function [L,Ld,a,ad,r,rd]=trajectory(t)
L=[.27+.02*sin(t);.28+.01*cos(2*t)];Ld=[.02*cos(t);-.02*sin(2*t)];
a=[.2*sin(3*t);-.15*cos(t)];ad=[.6*cos(3*t);.15*sin(t)];r=.1*sin(2*t);rd=.2*cos(2*t);
end

function [position,velocity]=reference_step(position,velocity,target,speed_max,accel_max,dt)
% 与NORMAL C参考生成器同序；固定零roll/腿角参考时高度等于对称腿长。
error=target-position;dv=accel_max*dt;
brake_speed=sqrt(2*accel_max*abs(error)+.25*dv*dv)-.5*dv;
wanted=sign(error)*min(speed_max,brake_speed);
next=min(max(wanted,velocity-dv),velocity+dv);
if abs(error)<=dv*dt && abs(error/dt-velocity)<=dv
 next=error/dt;position=target;
else
 position=position+next*dt;
end
velocity=next;
end

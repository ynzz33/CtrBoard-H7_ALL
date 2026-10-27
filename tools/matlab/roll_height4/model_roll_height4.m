function m = model_roll_height4(p)
%MODEL_ROLL_HEIGHT4 连续/离散四状态动力学；参数来自machine_table.roll_height。
% x=[roll roll_rate H Hdot]'，u=[F_left F_right]'；只建模，不设计控制器/导出。
if p.stiffness~=0,error('roll_height4:spring','非零刚度需要弹簧预载与参考位置定义');end
m.p=p;
m.Ac=[0 1 0 0;p.G_roll 0 0 0;0 0 0 1;0 0 0 -p.damping/p.M];
m.Bc=[0 0;p.half_track/p.I_roll -p.half_track/p.I_roll;0 0;1/p.M 1/p.M];
m.fc=[0;0;0;-p.g];
sd=c2d(ss(m.Ac,[m.Bc m.fc],eye(4),zeros(4,3)),p.Ts,'zoh');
m.Ad=sd.A;m.Bd=sd.B(:,1:2);m.fd=sd.B(:,3);
m.height=@height_kinematics;
end
function [H,Hdot]=height_kinematics(L,Ldot,theta,theta_dot,roll,roll_dot)
L=L(:);Ldot=Ldot(:);theta=theta(:);theta_dot=theta_dot(:);
projection=sum(L.*cos(theta));
H=.5*cos(roll)*projection;
Hdot=.5*cos(roll)*sum(Ldot.*cos(theta)-L.*sin(theta).*theta_dot)-.5*sin(roll)*roll_dot*projection;
end
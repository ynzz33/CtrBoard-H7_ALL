function mdl = model_balance14(machine, lengths, x0, u0, project_force)
%MODEL_BALANCE14 本机14态/6输入约化模型；只供MATLAB验证，不导出固件。
% x=[s ds yaw dyaw thetaL dthetaL thetaR dthetaR pitch dpitch roll droll H dH]。
% u=[Twl Twr TBl TBr FL FR]，前四项沿用model_AB契约，Tp=-TB。
% F为理想弹簧/腿自重抵消后的等效轴向力N，不是单髋电机力矩。
% 保留原10态冻结腿长方程；加入F*cos(theta)到高度/roll的耦合。
% 不声称完整多体模型：无ldot/lddot惯性、轮矩竖直投影或轴向力反向驱动10态。
% 横向仍沿用原近直立等效模型，不把三维大横滚投影作为已验证项。
if nargin < 3 || isempty(x0)
    x0 = zeros(14,1);
    x0(13) = mean(lengths); % 零姿态实际几何高度，不复用固定四态模型的H0。
end
if nargin < 4 || isempty(u0)
    u0 = [zeros(4,1); repmat(machine.roll_height.M*machine.roll_height.g/2,2,1)];
end
if nargin < 5, project_force = true; end
lengths = lengths(:);
x0 = x0(:); u0 = u0(:);
assert(numel(lengths)==2 && numel(x0)==14 && numel(u0)==6, 'balance14:shape');
assert(all(isfinite([lengths;x0;u0])), 'balance14:finite');
assert(all(lengths>=machine.ctrl.grid(1) & lengths<=machine.ctrl.grid(end)), ...
    'balance14:modelDomain', '腿长超出当前有效惯量域，禁止外推');
assert(machine.roll_height.stiffness==0, 'balance14:stiffness', '非零刚度需重审平衡与预载');
assert(strcmp(machine.ctrl.state_schema,'hip'), 'balance14:coordinate');
[A10,B10] = model_AB('sjtu5',machine,lengths(1),lengths(2));
rh = model_roll_height4(machine.roll_height);
p = machine.roll_height;
dyn = @(x,u) dynamics14(A10,B10,p,logical(project_force),x,u);

% 同一个动力学入口的解析雅可比；独立有限差分在verify中核对。
A = blkdiag(A10,rh.Ac);
B = blkdiag(B10,rh.Bc);
if project_force
    th = x0([5 7]);
    F = u0(5:6);
    A(14,[5 7]) = (-F.*sin(th)/p.M)';
    A(12,[5 7]) = (p.half_track/p.I_roll*[-F(1)*sin(th(1)); F(2)*sin(th(2))])';
    B(14,5:6) = (cos(th)/p.M)';
    B(12,5:6) = p.half_track/p.I_roll*[cos(th(1)) -cos(th(2))];
end
c = dyn(x0,u0)-A*x0-B*u0;
Ts = machine.ctrl.Ts;
sd = c2d(ss(A,[B c],eye(14),zeros(14,7)),Ts,'zoh');
mdl = struct('Ac',A,'Bc',B,'fc',c,'Ad',sd.A,'Bd',sd.B(:,1:6), ...
    'fd',sd.B(:,7),'Ts',Ts,'lengths',lengths,'x0',x0,'u0',u0, ...
    'project_force',logical(project_force),'dynamics',dyn,'A10',A10,'B10',B10);
mdl.operating_residual = norm(dyn(x0,u0),Inf);
mdl.geometric_height_at_operating_point = .5*cos(x0(11))*sum(lengths.*cos(x0([5 7])));
mdl.physical_mass_sum = machine.body.m_b+2*machine.body.m_l+2*machine.body.m_w;
mdl.height_equivalent_mass = p.M;
mdl.scope = 'reduced frozen-length LPV candidate; no firmware deployment';
end

function dx = dynamics14(A10,B10,p,project_force,x,u)
% 重力只在物理高度行出现一次；不再额外叠加support_force或height_trim。
dx = zeros(14,1);
dx(1:10) = A10*x(1:10)+B10*u(1:4);
force_z = u(5:6);
if project_force, force_z = force_z.*cos(x([5 7])); end
dx(11) = x(12);
dx(12) = p.G_roll*x(11)+p.half_track/p.I_roll*(force_z(1)-force_z(2));
dx(13) = x(14);
dx(14) = (force_z(1)+force_z(2))/p.M-p.g-p.damping/p.M*x(14);
end

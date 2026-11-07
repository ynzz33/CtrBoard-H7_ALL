function result = verify_balance14(machine,cfg)
%VERIFY_BALANCE14 模型退化、解析/差分、ZOH、LQR和局部非线性闭环检查。
% 软件自洽不等于本机物理验证；模型近似和未纳入补偿明确写入报告。
Q = cfg.Q; R = cfg.R;
base = model_balance14(machine,cfg.lengths);
rh = model_roll_height4(machine.roll_height);
[a10,b10] = c2d(base.A10,base.B10,cfg.Ts);
expected_A = blkdiag(a10,rh.Ad);
expected_B = blkdiag(b10,rh.Bd);
expected_f = [zeros(10,1);rh.fd];
assert(norm(base.Ad-expected_A,Inf)<1e-10 && norm(base.Bd-expected_B,Inf)<1e-10, ...
    'balance14:zeroAngle','零腿角未退回现有两个模型');
assert(norm(base.fd-expected_f,Inf)<1e-10,'balance14:affineGravity');
assert(base.operating_residual<1e-12,'balance14:equilibrium');
assert(abs(base.x0(13)-mean(cfg.lengths))<1e-12,'balance14:heightOrigin');
[K,P] = dlqr(base.Ad,base.Bd,Q,R);
K10 = dlqr(a10,b10,Q(1:10,1:10),R(1:4,1:4));
K4 = dlqr(rh.Ad,rh.Bd,Q(11:14,11:14),R(5:6,5:6));
gain_error = max(abs(K-blkdiag(K10,K4)),[],'all');
assert(gain_error<1e-6,'balance14:nominalLqr');
assert(max(abs(eig(base.Ad-base.Bd*K)))<1,'balance14:nominalPoles');
assert(min(eig((P+P')/2))>0,'balance14:terminalCost');

lengths = [0.15 0.15;0.23 0.27;0.31 0.31;0.15 0.31;0.31 0.15];
angles = [0 0;0.10 0.10;0.10 -0.07;-0.10 0.08];
errors = []; anchor = []; local_poles = []; cross_gain = 0;
for il=1:size(lengths,1)
    for it=1:size(angles,1)
        x = base.x0; x([5 7]) = angles(it,:)';
        x(13) = .5*sum(lengths(il,:)'.*cos(x([5 7])));
        u = base.u0;
        m = model_balance14(machine,lengths(il,:),x,u);
        [An,Bn] = difference_jacobian(m.dynamics,x,u);
        errors(end+1) = max([abs(An-m.Ac),abs(Bn-m.Bc)],[],'all'); %#ok<AGROW>
        anchor(end+1) = norm(m.Ac*x+m.Bc*u+m.fc-m.dynamics(x,u),Inf); %#ok<AGROW>
        % 增广矩阵指数独立核对ZOH及仿射槽，捕获矩阵拼接/索引错误。
        E = expm([m.Ac m.Bc m.fc;zeros(7,21)]*cfg.Ts);
        assert(norm([m.Ad m.Bd m.fd]-E(1:14,:),Inf)<1e-10,'balance14:zoh');
        [Kl,Pl] = dlqr(m.Ad,m.Bd,Q,R);
        local_poles(end+1) = max(abs(eig(m.Ad-m.Bd*Kl))); %#ok<AGROW>
        assert(local_poles(end)<1 && min(eig((Pl+Pl')/2))>0,'balance14:localLqr');
        if any(angles(it,:)~=0)
            cross_gain = max(cross_gain,norm(Kl(1:4,11:14),'fro')+norm(Kl(5:6,1:10),'fro'));
            assert(norm(m.Ac(11:14,1:10),'fro')>0,'balance14:missingCoupling');
        end
        % 明确不向已消去内部力的十态方程补加轴向力。
        assert(norm(m.Bc(1:10,5:6),'fro')==0,'balance14:doubleCountForce');
    end
end
assert(max(errors)<1e-7 && max(anchor)<1e-11,'balance14:linearization');
assert(cross_gain>1e-6,'balance14:onlyStacked');

% 关闭投影只供回归；不是新固件模式或物理功能开关。
x = base.x0; x([5 7]) = [.12;-.08];
off = model_balance14(machine,cfg.lengths,x,base.u0,false);
assert(norm(off.Ac-blkdiag(base.A10,rh.Ac),Inf)==0,'balance14:offRegression');
assert(norm(off.Bc-blkdiag(base.B10,rh.Bc),Inf)==0,'balance14:offRegression');

% 左右对称性在等腿长处核对；yaw/roll镜像取反，前后姿态保持。
sym = model_balance14(machine,[.23;.23]);
xs = sym.x0; xs([2 4 5 6 7 8 9 10 11 12 14]) = [.2;.1;.06;-.1;-.04;.05;.02;-.03;.02;-.04;.01];
us = sym.u0+[.1;-.05;.2;-.15;4;-3];
sx = eye(14); sx([5 6 7 8],:) = sx([7 8 5 6],:); sx([3 4 11 12],:) = -sx([3 4 11 12],:);
su = eye(6); su = su([2 1 4 3 6 5],:);
assert(norm(sym.dynamics(sx*xs,su*us)-sx*sym.dynamics(xs,us),Inf)<1e-8,'balance14:symmetry');

% 用同一个投影非线性对象检查原权重LQR；不加额外调参或裁剪掩盖不稳定。
tests = zeros(14,5);
tests(9,1) = .03; tests(11,2) = .03; tests(13,3) = .02;
tests([5 7],4) = [.08;-.06]; tests([2 9 11 13],5) = [.15;.02;-.02;.01];
closures = repmat(struct('terminal_relative',0,'peak_input',zeros(6,1),'peak_state',zeros(14,1)),1,size(tests,2));
for n=1:size(tests,2)
    closures(n) = close_loop(base,K,tests(:,n),cfg.Ts);
    assert(closures(n).terminal_relative<.1,'balance14:closedLoop','原权重约化对象未收敛');
end

result = struct('status','PASS (reduced-model software checks only)', ...
    'nominal_gain_error',gain_error,'jacobian_error',max(errors), ...
    'affine_error',max(anchor),'local_poles',local_poles,'cross_gain_norm',cross_gain, ...
    'mass_sum',base.physical_mass_sum,'height_mass',base.height_equivalent_mass, ...
    'closed_loops',closures,'K_nominal',K,'P_nominal',P,'Ad_nominal',base.Ad, ...
    'Bd_nominal',base.Bd,'fd_nominal',base.fd);
fprintf('balance14: %d local LQR points; Jacobian %.3g; nominal K error %.3g; cross gain %.3g\n', ...
    numel(local_poles),result.jacobian_error,gain_error,cross_gain);
end

function [A,B] = difference_jacobian(fun,x,u)
% 此处不复用解析投影导数，避免同一公式错两处仍共同通过。
A = zeros(14); B = zeros(14,6);
for j=1:14
    h = 1e-6*max(1,abs(x(j))); xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
    A(:,j) = (fun(xp,u)-fun(xm,u))/(2*h);
end
for j=1:6
    h = 1e-6*max(1,abs(u(j))); up=u; um=u; up(j)=up(j)+h; um(j)=um(j)-h;
    B(:,j) = (fun(x,up)-fun(x,um))/(2*h);
end
end

function stats = close_loop(mdl,K,offset,dt)
xref = mdl.x0; uref = mdl.u0; x = xref+offset;
initial = norm(offset); peak_input = zeros(6,1); peak_state = abs(offset);
for k=1:round(5/dt)
    % 同拍输入零阶保持，RK4只推进同一非线性对象，不在子步重复更新控制。
    u = uref-K*(x-xref);
    a = mdl.dynamics(x,u); b = mdl.dynamics(x+dt*a/2,u);
    c = mdl.dynamics(x+dt*b/2,u); d = mdl.dynamics(x+dt*c,u);
    x = x+dt*(a+2*b+2*c+d)/6;
    assert(all(isfinite(x)),'balance14:nonfinite');
    peak_input = max(peak_input,abs(u)); peak_state=max(peak_state,abs(x-xref));
end
stats = struct('terminal_relative',norm(x-xref)/initial,'peak_input',peak_input,'peak_state',peak_state);
end

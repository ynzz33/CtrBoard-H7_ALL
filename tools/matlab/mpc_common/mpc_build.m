function c = mpc_build(A,B,Q,R,P,N,rho)
%MPC_BUILD 输入分裂有限时域因子，物理终端P不加rho；不写入固件。
[nx,nu]=size(B); c=struct('A',A,'B',B,'Q',Q,'R',R,'P',zeros(nx,nx,N+1), ...
    'K',zeros(nu,nx,N),'Hinv',zeros(nu,nu,N),'rho',rho,'N',N);
c.P(:,:,end)=P;
for k=N:-1:1
    next=c.P(:,:,k+1); H=R+rho*eye(nu)+B'*next*B;
    c.Hinv(:,:,k)=H\eye(nu); c.K(:,:,k)=H\(B'*next*A);
    c.P(:,:,k)=Q+A'*next*A-A'*next*B*c.K(:,:,k);
    c.P(:,:,k)=(c.P(:,:,k)+c.P(:,:,k)')/2;
end
end

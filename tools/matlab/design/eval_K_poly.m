function K = eval_K_poly(F, lL, lR)
%EVAL_K_POLY 用拟合系数求 K(lL, lR), 4x10 (与板上 LQR_K_WBR 同一多项式)
switch F.order
    case 2
        b = [1; lL; lR; lL^2; lL*lR; lR^2];
    case 3
        b = [1; lL; lR; lL^2; lL*lR; lR^2; lL^3; lL^2*lR; lL*lR^2; lR^3];
    otherwise
        error('eval_K_poly:order', '未知 order %d', F.order);
end
K = reshape(reshape(F.C, 40, []) * b, 4, 10);
end

% 文件职责：MATLAB动力学模型一致性核对入口（verify_models.m）。
% 输入/来源：machine_table和model_AB提供的模型。
% 输出/去向：执行现有模型数值/维度相关检查并报告。
% 关键边界：用于生成侧模型核对，不下载固件、不驱动机器人。
% 阅读入口：md/file-guide.md；参数当前值以源码为准。

function verify_models()
% 验证公共髋轴状态与原上交轮轴状态的映射。
here = fileparts(mfilename('fullpath'));
addpath(here);
names = {'small_wheelleg','big_wheelleg'};
points = [0.15,0.21;0.19,0.27];
for k = 1:2
    m = machine_table(names{k});
    lL = points(k,1); lR = points(k,2);
    [Ah,Bh] = model_AB('sjtu5',m,lL,lR);
    m.ctrl.state_schema = 'wheel';
    [Aw,Bw] = model_AB('sjtu5',m,lL,lR);
    xw = [0.02;0.3;0.01;-0.2;0.03;0.15;-0.02;-0.1;0.04;0.2];
    u = [0.1;-0.2;0.3;-0.1];
    xh = xw;
    xh(1) = xw(1)+(lL*xw(5)+lR*xw(7))/2;
    xh(2) = xw(2)+(lL*xw(6)+lR*xw(8))/2;
    xh(5:8) = -xh(5:8);
    dxw = Aw*xw+Bw*u;
    expected = dxw;
    expected(1) = dxw(1)+(lL*dxw(5)+lR*dxw(7))/2;
    expected(2) = dxw(2)+(lL*dxw(6)+lR*dxw(8))/2;
    expected(5:8) = -expected(5:8);
    error = max(abs(Ah*xh+Bh*u-expected));
    assert(error<1e-10,'状态坐标不一致');
    assert(all(isfinite(Ah(:))) && all(isfinite(Bh(:))));
    fprintf('%s asymmetric state mapping error %.3g\n',names{k},error);
end
end

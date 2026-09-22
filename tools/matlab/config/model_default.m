function name = model_default()
%MODEL_DEFAULT 当前动力学模型
%   'sjtu5'     Leg2 上交 5 方程线性模型 (板上现表来源; 阶段 0/1)
%   'newton15'  自研 15 方程牛顿-欧拉模型 (阶段 2 接入)
name = 'sjtu5';
end

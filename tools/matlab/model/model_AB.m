function [A, B] = model_AB(model, m, lL, lR)
%MODEL_AB 统一模型接口: 连续时间 A (10x10), B (10x4)
%   [A, B] = model_AB('sjtu5', m, lL, lR)
% 内部找 cache/AB_<model>_gen.m (matlabFunction 生成物), 没有就先跑 model/<model>/build_<model>.m 生成一次。
% 腿长 → 腿质心参数由各模型自己的 leg_row_*.m 负责。加新模型 = 加子目录 + 这里加一个 case。
here      = fileparts(mfilename('fullpath'));          % tools/matlab/model
cache_dir = fullfile(fileparts(here), 'cache');
if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end

switch lower(model)
    case 'sjtu5'
        gen = fullfile(cache_dir, 'AB_sjtu5_gen.m');
        if ~exist(gen, 'file')
            build_sjtu5(gen);
        end
        if exist('AB_sjtu5_gen', 'file') ~= 2
            addpath(cache_dir);
            rehash;
        end
        [A, B] = AB_sjtu5_gen(sjtu5_param_vec(m, lL, lR));
    case 'newton15'
        error('model_AB:notyet', 'newton15 在阶段 2 接入 (见 LQR_MATLAB_PLAN.md §六)');
    otherwise
        error('model_AB:unknown', '未知模型 "%s"', model);
end
end

function result=verify_mpc4(host_compiler)
%VERIFY_MPC4 数学/C对拍及真实四态适配层主机回归；不运行HAL或声称板端计时。
if nargin<1 || isempty(host_compiler),host_compiler=getenv('MPC_HOST_CC');end
if isempty(host_compiler),host_compiler='zig';end
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(fileparts(here)));
result=run_mpc4(host_compiler);
out=fullfile(here,'output');
project=fileread(fullfile(root,'MDK-ARM','CtrBoard-H7_ALL.uvprojx'));
token=regexp(project,'<IncludePath>([^<]*Algorithm/lqr/inc[^<]*)</IncludePath>','tokens','once');
assert(~isempty(token),'mpc4 includePaths');dirs=strsplit(token{1},';');includes='';
for k=1:numel(dirs),includes=[includes ' -I"' fullfile(root,'MDK-ARM',dirs{k}) '"'];end
sources={fullfile(here,'host_adapter_test.c'),fullfile(root,'imcalib','Algorithm','mpc','roll_height_mpc.c'), ...
    fullfile(root,'imcalib','Algorithm','mpc','mpc_solver.c'),fullfile(root,'imcalib','Algorithm','mpc','roll_height_mpc_data.c'), ...
    fullfile(root,'imcalib','Algorithm','lqr','roll_height_model.c')};
extra={'imcalib/Algorithm/lqr/leg_balance.c','imcalib/Algorithm/lqr/roll_height_balance.c', ...
    'imcalib/common/pid.c','imcalib/common/simple-function.c','imcalib/Algorithm/estimation/leg_solver.c', ...
    'imcalib/Algorithm/compensation/leg_compensation.c','imcalib/Algorithm/compensation/gas_spring.c', ...
    'imcalib/Algorithm/compensation/gravity_comp.c','imcalib/common/arm_sin_f32.c','imcalib/common/arm_cos_f32.c', ...
    'Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c'};
for k=1:numel(extra),sources{end+1}=fullfile(root,extra{k});end
includes=[includes ' -I"' fullfile(root,'Drivers','CMSIS','DSP','Include') '"'];
args='';for k=1:numel(sources),args=[args ' "' sources{k} '"'];end
for backend=0:1
for predict=0:1
    for constraints=0:1
        exe=fullfile(out,sprintf('adapter_b%d_p%d_c%d.exe',backend,predict,constraints));
        % 使用当前配置的临时副本覆盖试验开关，不改作者当前1/0/1等生产值。
        config=fileread(fullfile(root,'imcalib','config','machine_config.h'));
        keys={'MACHINE_ROLL_HEIGHT_USE_MPC','MACHINE_MPC_REFERENCE_PREDICT','MACHINE_MPC_INPUT_CONSTRAINT'};
        values=[backend predict constraints];
        for k=1:3,config=regexprep(config,['(?m)^#define\s+' keys{k} '\s+[^\r\n]+'],sprintf('#define %s %du',keys{k},values(k)));end
        config_file=fullfile(out,sprintf('config_b%d_p%d_c%d.h',backend,predict,constraints));
        f=fopen(config_file,'w');assert(f>=0);fprintf(f,'%s',config);fclose(f);
        cmd=sprintf('"%s" cc -O2 -UNDEBUG%s -include "%s"%s -o "%s"', ...
            host_compiler,includes,config_file,args,exe);
        [status,msg]=system(cmd);
        if backend==1 && constraints==0
            assert(status~=0 && contains(msg,'MPC requires input constraints'),'mpc4 constraint configuration guard');
            fprintf('MPC enabled with constraints disabled: compile rejection PASS\n');
            continue;
        end
        assert(status==0,'%s',msg);
        [status,msg]=system(['"' exe '"']);fprintf('%s',msg);assert(status==0,'mpc4 adapterRegression');
    end
end
end
% Actual production adapter/reference generator on its nominal plant; isolate trim from this model test.
preview_metrics=zeros(3,8,2);
for predict=0:1
    config=fileread(fullfile(out,sprintf('config_b1_p%d_c1.h',predict)));
    config=regexprep(config,'(?m)^#define\s+MACHINE_HEIGHT_TRIM_ENABLE\s+[^\r\n]+','#define MACHINE_HEIGHT_TRIM_ENABLE 0u');
    config_file=fullfile(out,sprintf('preview_config_p%d.h',predict));f=fopen(config_file,'w');assert(f>=0);fprintf(f,'%s',config);fclose(f);
    exe=fullfile(out,sprintf('preview_p%d.exe',predict));
    preview_args=strrep(args,fullfile(here,'host_adapter_test.c'),fullfile(here,'host_preview_test.c'));
    [status,msg]=system(sprintf('"%s" cc -O2 -UNDEBUG%s -include "%s"%s -o "%s"',host_compiler,includes,config_file,preview_args,exe));assert(status==0,'%s',msg);
    [status,msg]=system(['"' exe '"']);fprintf('Preview %d nominal closed loop:\n%s',predict,msg);assert(status==0,'mpc4 preview closed loop');
    preview_metrics(:,:,predict+1)=reshape(sscanf(msg,'%f'),8,3)';
end
assert(all(preview_metrics(:,2,2)<preview_metrics(:,2,1)),'mpc4 preview must improve nominal height RMS');
assert(all(preview_metrics(:,4,2)<=preview_metrics(:,4,1)+1e-5),'mpc4 preview roll RMS regression');
result.preview_metrics=preview_metrics;
% Existing observer: prove known trim is not learned twice, using a nominal residual-load plant.
for observe=0:1
    config=fileread(fullfile(out,'config_b1_p1_c1.h'));
    config=regexprep(config,'(?m)^#define\s+MACHINE_ROLL_HEIGHT_OBSERVER_ENABLE\s+[^\r\n]+',sprintf('#define MACHINE_ROLL_HEIGHT_OBSERVER_ENABLE %du',observe));
    config=regexprep(config,'(?m)^#define\s+MACHINE_HEIGHT_TRIM_ENABLE\s+[^\r\n]+','#define MACHINE_HEIGHT_TRIM_ENABLE 1u');
    config_file=fullfile(out,sprintf('bias_config_o%d.h',observe));f=fopen(config_file,'w');fprintf(f,'%s',config);fclose(f);
    exe=fullfile(out,sprintf('bias_o%d.exe',observe));bias_args=strrep(args,fullfile(here,'host_adapter_test.c'),fullfile(here,'host_bias_test.c'));
    [status,msg]=system(sprintf('"%s" cc -O2 -UNDEBUG%s -include "%s"%s -o "%s"',host_compiler,includes,config_file,bias_args,exe));assert(status==0,'%s',msg);
    [status,msg]=system(['"' exe '"']);fprintf('Observer %d nominal residual-load test:\n%s',observe,msg);assert(status==0,'mpc4 bias regression');
end
result.bias_regressions=4;
result.adapter_regressions=8;
save(fullfile(out,'verification_latest.mat'),'result');
f=fopen(fullfile(out,'verification_latest.txt'),'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));
fprintf(f,'Software verification PASS; not hardware validation\n');
fprintf(f,'MPC: %d input intervals, node %.9g s, solve_div %d; single-solve deadline disabled\n',result.input_intervals,result.node_period,result.solve_div);
fprintf(f,'4/2 core QP cases: 18; max full-sequence error %.9g N; max iterations %d\n', ...
    max([result.max_sequence_error result.dynamic_error]),max([result.iterations result.dynamic_iterations]));
fprintf(f,'10/4 reuse first-input error %.9g Nm\n',result.ten_state_first_error);
fprintf(f,'Adapter combinations: 6 runtime PASS + 2 forbidden unconstrained MPC compile-rejection PASS; 500Hz/ZOH/changed-bound rejection, real preview/NORMAL path, compensation and invalid cases\n');
fprintf(f,'Nominal preview off/on experiments: 3 rise/fall/roll-disturbance cases, all improve height RMS; trim disabled in this model-only test.\n');
for k=1:3,fprintf(f,'case %d height RMS %.9g -> %.9g m; peak %.9g -> %.9g m; force peak %.9g -> %.9g N; delta-force RMS %.9g -> %.9g N\n',k,preview_metrics(k,2,1),preview_metrics(k,2,2),preview_metrics(k,3,1),preview_metrics(k,3,2),preview_metrics(k,5,1),preview_metrics(k,5,2),preview_metrics(k,6,1),preview_metrics(k,6,2));end
fprintf(f,'No board execution, timing, sensor or physical output validation.\n');
end

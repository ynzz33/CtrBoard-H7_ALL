function output=export_roll_height4(m,output)
%EXPORT_ROLL_HEIGHT4 同一设计写出K与观测器实际使用的A/B/f/L。
p=m.p;
assert(abs(p.Ts-.001)<1e-12,'C执行拍当前1kHz，Ts必须同步为0.001');
folder=fileparts(output);if ~exist(folder,'dir'),mkdir(folder);end
fid=fopen(output,'w','n','UTF-8');if fid<0,error('roll_height4:export','无法写入输出目录');end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'/* MATLAB生成四状态LQR+力偏置观测：%s\n',p.machine);
fprintf(fid,' * Q=[%g %g %g %g] R=[%g %g] Ts=%g\n',diag(m.Q)',diag(m.R)',p.Ts);
fprintf(fid,' * M=%g half_track=%g I_roll=%g damping=%g observer_tau=%s；实机效果待验证。\n */\n',p.M,p.half_track,p.I_roll,p.damping,mat2str(p.observer_tau));
fprintf(fid,'#include "roll_height_model.h"\n#include "machine_config.h"\n#if RH_MODEL_IFACE != 5u\n#error "Regenerate roll_height_model"\n#endif\n\nconst roll_height_model_t roll_height_model = {\n');
fprintf(fid,'    .iface = RH_MODEL_IFACE,\n    .machine_id = %s,\n    .ts = %.9ef,\n',p.machine_id,p.Ts);
write_matrix(fid,'K',m.K);write_matrix(fid,'A',m.Ad);write_matrix(fid,'B',m.Bd);
fprintf(fid,'    .f = {%.9ef, %.9ef, %.9ef, %.9ef},\n',m.fd);
write_matrix(fid,'L',m.L);fprintf(fid,'};\n');
end
function write_matrix(fid,name,value)
fprintf(fid,'    .%s = {\n',name);
for i=1:size(value,1)
 fprintf(fid,'        {%.9ef',value(i,1));fprintf(fid,', %.9ef',value(i,2:end));fprintf(fid,'},\n');
end
fprintf(fid,'    },\n');
end

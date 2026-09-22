function list = machine_pending(m)
%MACHINE_PENDING 列出机器表中 status 为 '待实测' 的字段 (导出 C 文件头会打印)
list = {};
if ~isfield(m, 'status') || isempty(m.status), return; end
for k = 1:size(m.status, 1)
    if contains(m.status{k, 2}, '待实测')
        list{end + 1} = sprintf('%s(%s)', m.status{k, 1}, m.status{k, 2}); %#ok<AGROW>
    end
end
end

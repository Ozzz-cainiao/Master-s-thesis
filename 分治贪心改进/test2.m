C = {["t4", "t2", "theta2"]; ["theta1", "t2", "theta2"]}; % 定义 cell 数组

results = []; % 初始化一个空数组用来存储结果

for i = 1:size(C, 1)
    row = C{i, 1}; % 获取当前行的数据
    if any(endsWith(row, '1')) % 检查当前行是否有元素以 "1" 结尾
        results = [results, i]; % 如果有，则将行索引添加到结果中
    end
end

disp(results); % 显示结果

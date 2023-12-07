%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\NNM.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-05
% 描述: 最近邻匹配算法，用于将来自不同传感器的数据关联到同一个目标上
% 输入:  
% 输出:  
%**************************************************************************


%% 下方是chatGPT的模板
% 假设 sensor1_data 和 sensor2_data 是两个传感器的数据矩阵
% 假设第一列是目标类型，第二列是线谱频率

% 设置距离阈值
distance_threshold = 0.1; % 根据具体情况设置阈值

% 初始化一个空矩阵来存储关联的数据
associated_data = [];

% 遍历 sensor1 的每一行数据
for i = 1:size(sensor1_data, 1)
    type1 = sensor1_data(i, 1);
    freq1 = sensor1_data(i, 2);
    
    % 在 sensor2 中查找距离最近的数据
    min_distance = inf;
    nearest_data = [];
    
    for j = 1:size(sensor2_data, 1)
        type2 = sensor2_data(j, 1);
        freq2 = sensor2_data(j, 2);
        
        % 计算距离（这里使用欧氏距离）
        distance = sqrt((type1 - type2)^2 + (freq1 - freq2)^2);
        
        % 如果距离小于阈值且更近，则更新关联数据
        if distance < distance_threshold && distance < min_distance
            min_distance = distance;
            nearest_data = sensor2_data(j, :);
        end
    end
    
    % 如果找到了关联数据，存储关联结果
    if ~isempty(nearest_data)
        associated_data = [associated_data; sensor1_data(i, :), nearest_data];
    end
end

% 输出关联结果
disp(associated_data);

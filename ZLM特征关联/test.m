clc
clear
close all

center = 463;  % 均值
x = 9;       % 方差

% 生成 x 轴上的值
x_values = linspace(center - 3*sqrt(x), center + 3*sqrt(x), 1000);

% 计算正态分布概率密度函数
y_values = normpdf(x_values, center, sqrt(x));

% 绘制正态分布曲线
plot(x_values, y_values);
title('正态分布曲线');
xlabel('x轴');
ylabel('概率密度');

% 假设你的4x1 cell矩阵为C
C = {
    {'A1', 'B1'}, 
    {'A2', 'B2'}, 
    {'A3', 'B3', 'C3'}, 
    {'A4', 'B4', 'C4'}
};

% 初始化标记矩阵
% 创建一个相同大小的空cell矩阵，并将每个元素置为0
match_labels = cellfun(@(x) cell(size(x)), C, 'UniformOutput', false);
% 初始化一个标记号
label = 1;

% 逐个遍历每个元素
for i = 1:size(C, 1)  % 遍历每一行
    for j = 1:size(C{i}, 2)  % 遍历当前行的每个元素
        if isempty(match_labels{i}{j}) % 如果当前元素还没有被标记
            % 计算当前元素与其他行的匹配度
            base_feature = C{i}{j};
            for k = i+1:size(C, 1)  % 从下一行开始遍历
                for l = 1:size(C{k}, 2)  % 遍历下一行的每个元素
                    % 计算匹配度，这里假设你有一个函数match_degree计算匹配度
                    degree = match_degree(base_feature, C{k}{l});
                    if degree > 0.5
                        if isempty(match_labels{k}{l}) % 如果下一行当前元素还没有被标记
                            match_labels{k}{l} = label;  % 标记
                        end
                    end
                end
            end
            if isempty(match_labels{i}{j})  % 如果当前元素计算完毕仍然为空，标记为0
                match_labels{i}{j} = 0;
            end
            label = label + 1;  % 更新标记号
        end
    end
end

% 输出标记结果
disp(match_labels);
function match_degree()


end

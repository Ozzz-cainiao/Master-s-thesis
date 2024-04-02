

clc
clear
close all


% 示例数据点之间的相似度矩阵
similarity_matrix = rand(10, 10); % 这里假设相似度矩阵已经计算好

% 使用单链接（single-linkage）进行层次聚类
linkage_matrix = linkage(squareform(similarity_matrix), 'single');

% 计算聚类树的保持相似性系数
c = cophenet(linkage_matrix, similarity_matrix);

% 绘制聚类树的树状图
dendrogram(linkage_matrix);

% 输出保持相似性系数
disp(['Cophenetic correlation coefficient: ', num2str(c)]);











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
%%
% 生成一些示例数据
rng(1); % 设置随机数种子以便结果可复现
X = rand(20, 2); % 20个样本，每个样本有2个特征

% 计算数据点之间的距离（可以使用不同的距离度量方法）
distances = pdist(X);

% 使用linkage函数进行层次聚类（使用平均链接方法）
Z = linkage(distances, 'average');

% 绘制谱系树（聚类树）
dendrogram(Z);

% 根据谱系树划分聚类（可以选择划分的阈值）
T = cluster(Z, 'cutoff', 0.5, 'criterion', 'distance');

% 显示聚类结果
disp(T);



%%
clc
close all
clear
% 原始数据
data = {
    [1,1;2,1;3,2];
    [1,1;2,1;4,1];
    [1,2;2,2;3,2;4,2];
    [1,2;3,2];
    [1,3;3,3];
    [1,3;4,3];
    [2,1;4,1];
    [2,2;3,2;4,2];
    [3,1]
};

% 检查每个元素是否为其他元素的子集
to_remove = false(size(data, 1), 1);
for i = 1:size(data, 1)
    for j = 1:size(data, 1)
        if i ~= j && isSubset(data{i}, data{j})
            to_remove(i) = true;
            break;
        end
    end
end

% 删除子集
result = data(~to_remove);

% 显示结果
disp('删除子集后的结果：');
disp(result);

% 判断一个 cell 是否是另一个 cell 的子集
function is_sub = isSubset(cell1, cell2)
    is_sub = all(ismember(cell1, cell2, 'rows'), 'all');
end


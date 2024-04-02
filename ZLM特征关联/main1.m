%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\main.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-12
% 描述: 新特征关联主程序，在这个程序内部实现模糊数学关联,
%      使用蒙特卡罗统计
% 输入:
% 输出:
%**************************************************************************

clc;
clear;
close all;
numOfM = 1; % 控制蒙特卡洛的次数
%% 观测数据
T = 0.1; %观测周期
T_all = 0.1; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.0^2; % 角度制  角度误差
pd = 0.9; % 检测概率
% 虚警期望  % 不知道虚警是怎么设置的

%% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];

%% 运动模型
% 运动方程 x = x_last + v * t + 0.5 * a * t^2;
% 匀加速运动矩阵
F1 = [1, 0, T, 0; ...
    0, 1, 0, T; ...
    0, 0, 1, 0; ...
    0, 0, 0, 1];
%加速度矩阵
F2 = [0.5 * T^2, 0; ...
    0, 0.5 * T^2; ...
    T, 0; ...
    0, T]; %

%% 布放目标
initial_position1 = [4e3, 7e3]; % 初始位置目标1
velocity1 = [0, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
feature1 = {5, {460, 580, 650, 790, 880}, 3.9, 6, 6}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source1 = SoundSource('CW', feature1, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [0, 0]; % 运动速度
acc2 = 0; % 加速度
feature2 = {8, {380, 420, 460, 550, 620, 710, 790, 880}, 4.7, 3, 5}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source2 = SoundSource('CW', feature2, 100, initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [2e3, 5e3]; % 初始位置目标2
velocity3 = [0, 0]; % 运动速度
acc3 = 0; % 加速度
feature3 = {4, {320, 455, 560, 730}, 6.5, 7, 8}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source3 = SoundSource('CW', feature3, 100, initial_position3, velocity3, F1, F2, acc3);
% sourceAll = [source1];
sourceAll = [source1, source2, source3];

%% 观测
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
timeR = cell(1, numOfPlatForm); % 存放时延
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
realwuT = cell(1, numOfPlatForm); % 存放无时延的真实的角度
% feature_matrix = cell(1, T_num);
feature_matrix = cell(numOfPlatForm, numOfSource);

%% 获取每个平台的每个目标信息
% 确保来自同一个平台的不会关联上
% 做一个标记，看是哪个目标
count = 0;
for i = 1:numOfM
    tag = cell(numOfPlatForm, numOfSource);
    P_Featu = cell(numOfPlatForm, 1);
    for j = 1:numOfPlatForm % 遍历平台
        % 平台观测到的特征向量
        for k = 1:numOfSource % 遍历声源
            [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
            % 虚警
            if randi([1, 20]) == 1
                % 创造出一个不存在的目标特征向量
                P_Featu{j}{end + 1} = create_new_feature_vector();
                P_Featu{j}{end + 1} = generate_feature_vector(fre);
                tag{j, k} = ['y', int2str(k)];
            % 漏报
            elseif randi([1, 20]) == 2
                continue;
            % 正常
            else
                P_Featu{j}{end + 1} = generate_feature_vector(fre);
                tag{j, k}{end + 1} = int2str(k);
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm
    % 本次观测内
    count = count + specific(P_Featu, tag);
end
fprintf("正确率为%d", count);
% 对特征向量进行区分的函数
% input: 行：平台数，列 特征向量个数
% 1. 获取线谱特征， 将线谱特征根据相对多普勒频移进行扩展，获取线谱特征集合
% 2. 根据线谱特征集合，构建模糊隶属度函数（梯形分布偏大型或中间型）
% 3. 根据模糊隶属度函数构建线谱模糊关系矩阵元素，利用元素构建模糊矩阵
% function [result] = specific(input, tag)
% 输入：各平台测量的特征，与设置的目标标记
% 输出：设置目标个数与真实目标个数是否一致
function [result] = specific(input, tag)
membership_value1 = demon_match_degree(input);
membership_value2 = lineFre_match_degree(input);

% 加权两次隶属度矩阵
all_membership = membership_value1 + membership_value2;
all_membership(all_membership < 0.1) = 0;
book = [];
for i = 1:size(input, 1)
    for j = 1:size(input{i}, 2)
        book(end+1, :) = [i, j];
    end
end
%% 贪心算法
% 1. 将平台1的所有测量放到base_feature
base_len = size(input{1}, 2); % 平台1的测量数
base_feature = cell(base_len, 1);
for i = 1:base_len
    base_feature{i} = [1, i];
end
% 然后开始关联
p = 2; % 从第2个平台开始
delta = 0.8; % 关联度门限
index = 0;
new_feature_index = [];
base_feature_index = 1:base_len;
while p <= size(input, 1)
    index = index + size(input{p-1}, 2);
    fprintf("当前起始索引%d\n", index);
    % 取出对应的关联度矩阵
    this_len = size(input{p}, 2); % 此平台的量测个数
    base_feature_index = [base_feature_index, new_feature_index];
    matrix = all_membership(base_feature_index, index+1:index+this_len); % 这个有问题
    new_feature_index = [];
    R = zeros(size(matrix));
    % 按列看
    for i = 1:this_len
        % 先判断是否全为0
        if all(matrix(:, i) == 0)
            fprintf("全为0，平台%d测量%d是新的一类\n", p, i);
            new_feature_index = [new_feature_index, i + index];
            base_len = base_len + 1;
            base_feature{base_len} = book(i + index, :);
        else
            % 找到这一列最大的关联度
            [M, I] = max(matrix(:, i));
            if R(I, i) == 0 && M > delta
                R(:, i) = 1;
                R(I, :) = 1; % 证明这个已经关联上了 % 将行置为1
                base_feature{I} = [base_feature{I}; book(i + index, :)];
            elseif R(I, i) == 0 && M < delta
                % 添加为新的一类
                fprintf("R = 0, M < delta, 平台%d测量%d是新的一类\n", p, i);
                new_feature_index = [new_feature_index, i + index];
                base_len = base_len + 1;
                base_feature{base_len} = book(i + index, :);
            elseif R(I, i) == 1 && M < delta
                fprintf("R = 1, M < delta, 平台%d测量%d是新的一类\n", p, i);
                %             R(:, i) = 1;
                %             R(I, :) = 1; % 证明这个已经关联上了
                new_feature_index = [new_feature_index, i + index];
                base_len = base_len + 1;
                base_feature{base_len} = book(i + index, :);
            else
                fprintf("平台%d测量%d已经被关联过了\n", p, i)
            end
        end
    end
    p = p + 1;
end
fprintf("共分类%d组\n", base_len);
allElements = vertcat(tag{:});
totalCount = sum(cellfun(@(x) sum(ismember(x, 'y')), allElements));
fprintf("实际有%d组\n", totalCount+3);
result = isequal(base_len, totalCount+3);

%% 层次聚类算法————可用

% 将相似度矩阵转换为距离矩阵
distance_matrix = 1 - all_membership;
% 使用单链接（single-linkage）进行层次聚类
% linkage_matrix = linkage(squareform(distance_matrix), 'single');

% 计算聚类树的保持相似性系数
c = cophenet(linkage_matrix, similarity_matrix);

% 绘制聚类树的树状图
dendrogram(linkage_matrix);

% 输出保持相似性系数
disp(['Cophenetic correlation coefficient: ', num2str(c)]);
% 定义聚类个数的范围
min_clusters = 2;
max_clusters = 6;

% 初始化变量以保存最佳聚类结果和相应的评价准则值
best_T = [];
best_k = 0;
best_criteria_value = -Inf;
% 初始化评价指标数组
silhouette_vals = zeros(1, max_clusters);
calinski_vals = zeros(1, max_clusters);
DaviesBouldin_vals = zeros(1, max_clusters);

% 遍历不同的聚类个数
for k = min_clusters:max_clusters
    % 使用层次聚类算法将类分为k类
    Z = linkage(distance_matrix, 'average'); % 使用平均链接法
    T = cluster(Z, 'maxclust', k); % 将类分为k类
    %% Davies-Bouldin Index
    % 计算每个聚类的中心点
    centroids = zeros(k, size(distance_matrix, 2));
    for i = 1:k
        centroids(i, :) = mean(distance_matrix(T == i, :));
    end

    % 计算每个聚类的紧密度
    intra_cluster_distances = zeros(k, 1);
    for i = 1:k
        intra_cluster_distances(i) = mean(pdist2(distance_matrix(T == i, :), centroids(i, :)));
    end

    % 计算每个聚类与最近邻聚类的距离
    inter_cluster_distances = zeros(k, k);
    for i = 1:k
        for j = 1:k
            if i ~= j
                inter_cluster_distances(i, j) = mean(pdist2(distance_matrix(T == i, :), centroids(j, :)));
            end
        end
    end

    % 计算Davies-Bouldin Index
    db_index = 0;
    for i = 1:k
        max_db_value = -Inf;
        for j = 1:k
            if i ~= j
                db_value = (intra_cluster_distances(i) + intra_cluster_distances(j)) / inter_cluster_distances(i, j);
                if db_value > max_db_value
                    max_db_value = db_value;
                end
            end
        end
        db_index = db_index + max_db_value;
    end
    DaviesBouldin_vals(k) = db_index / k;

    %%
    % 计算轮廓系数
    if k > 1
        silhouette_vals(k) = mean(silhouette(distance_matrix, T));
    end
    
    % 计算Calinski-Harabasz指数
    eva = evalclusters(distance_matrix, T, 'CalinskiHarabasz');
    calinski_vals(k) = eva.CriterionValues;
    criteria_value = eva.CriterionValues;

    % 如果当前的评价准则值更好，则更新最佳聚类结果和相应的评价准则值
    if criteria_value > best_criteria_value
        best_T = T;
        best_k = k;
        best_criteria_value = criteria_value;
    end
end
% 绘制轮廓系数和Calinski-Harabasz指数随聚类个数变化的曲线
figure;
subplot(3, 1, 1);
plot(min_clusters:max_clusters, silhouette_vals(min_clusters:end));
xlabel('Number of clusters');
ylabel('Silhouette Coefficient');
title('Silhouette Coefficient vs. Number of Clusters');
subplot(3, 1, 2);
plot(min_clusters:max_clusters, calinski_vals(min_clusters:end) ./ best_criteria_value); % 归一化
xlabel('Number of clusters');
ylabel('Calinski-Harabasz Index');
title('Calinski-Harabasz Index vs. Number of Clusters');
subplot(3, 1, 3);
plot(min_clusters:max_clusters, DaviesBouldin_vals(min_clusters:end));
xlabel('Number of clusters');
ylabel('Davies-Bouldin Index');
title('Davies-Bouldin Index vs. Number of Clusters');

% 根据评价准则加权，获取综合结果
for i = min_clusters:max_clusters
    all_vals(i) = silhouette_vals(i) + calinski_vals(i) ./ best_criteria_value + 1 / DaviesBouldin_vals(i);
end
% 输出最优聚类结果和相应的评价准则值
[best_criteria_value, best_k] = max(all_vals);
disp(['Best number of clusters: ', num2str(best_k)]);
disp(['Best Calinski-Harabasz index: ', num2str(best_criteria_value)]);


% % 将相似度矩阵转换为距离矩阵
% distance_matrix = 1 - all_membership;
% % 使用层次聚类算法将类分为3类
% Z = linkage(distance_matrix, 'average'); % 使用平均链接法
% % dendrogram(Z); % 绘制树状图
% 
% % 尝试不同的聚类数
% max_clusters = 6; % 最大聚类数
% % 初始化评价指标数组
% silhouette_vals = zeros(1, max_clusters);
% calinski_vals = zeros(1, max_clusters);
% for k = 2:max_clusters
%     % 使用层次聚类算法将类分为k类
%     Z = linkage(distance_matrix, 'average'); % 使用平均链接法
%     T = cluster(Z, 'maxclust', k); % 将类分为k类
% 
%     fprintf("当前分类数为%d\n", k);
%     % 输出每个类的索引
%     for i = 1:k
%         class_i_indices = find(T == i);
%         class_i_indices_str = num2str(class_i_indices');
%         disp(['类别', num2str(i), '包含的类索引：', class_i_indices_str]);
%     end
%     disp("--------------------")
% end

%% 测试拍卖算法 不好用


%% 下方暂时不用
% % 计算模糊度
% ii = 0;
% for i = 1:size(res1, 1)
%     for j = i + 1:size(res1(i, :), 2)
%         ii = ii + 1;
%         distances(ii) = freq_membership2(res1(i, j));
%     end
% end
%
%
% combine_ = cell(size(res1, 1), 1);
% for i = 1 : size(res1, 1)
%     for j = 1:size(res1(i, :), 2)
%         if membership_value(i, j) > 0.5
%             % 放到cell中，格式可以是{1,1;2,1;3,2}也就是一个目标放在一起，表示哪个平台哪个特征
%             combine_{i} = [combine_{i}; book(i, :); book(j, :)];
%         end
%     end
%     % 去重
%     A = combine_{i};
%     % 将每一行转换为一个字符串
%     str_rows = cellfun(@(row) sprintf('%d%d', row(1), row(2)), num2cell(A, 2), 'UniformOutput', false);
%     % 对字符串进行去重
%     [~, idx, ~] = unique(str_rows);
%     % 获取去重后的行索引
%     combine_{i} = A(idx, :);
%     % 加上相似度
%     lamda = 0; % 将相似度设为0
%     for m = 1 : numel(combine_{i})
%         for n = m + 1: size(combine_{i}, 1)
%             index1 = find(all(book == combine_{i}(m, :), 2));
%             index2 = find(all(book == combine_{i}(n, :), 2));
%             lamda = lamda + membership_value(index1, index2);
%
%             disp(membership_value(index1, index2));
%         end
%     end
%     combine_{i, 2} = lamda;
% end
% % 对combine_去重
% str_data = cellfun(@mat2str, combine_(:,1), 'UniformOutput', false);
% [~, idx, ~] = unique(str_data);
% combine_ = combine_(idx,:);
% % 删除子集
% data = combine_(:, 1);
% % 检查每个元素是否为其他元素的子集
% to_remove = false(size(data, 1), 1);
% for i = 1:size(data, 1)
%     for j = 1:size(data, 1)
%         if i ~= j && isSubset(data{i}, data{j})
%             to_remove(i) = true;
%             break;
%         end
%     end
% end
%
% % 删除子集
% combine_ = combine_(~to_remove, :);
% % 按顺序合并，
% % 如果出现互斥测量，则认为是新的一类,
%
% % 如果是新平台的新测量，则合入
% base = combine_(1);
% for i = 2 : size(combine_)
%     cur = combine_(i);
%     flag = false; % cur是否合并的标志位
%     % 判断是否冲突 如果存在来自同一个平台的测向线，则认为冲突
%     for j = 1 : size(base, 1) % 对于每组特征
%         % 合并
%         W = cat(1, cell2mat(base(j)), cell2mat(cur));
%         % 去重
%         p = W(:, 1); % 提取平台号
%         unique_values = unique(p);
%         if numel(p) ~= numel(unique_values)
%             % 说明矩阵中有来自同一个平台不同测向线的数据，不能合并到当前中
%             disp("无法合并");
%         else
%             % 合并到base中
%             base(j) = [base(j), cur];
%             flag = true;
%         end
%     end
% end
%
%

end
% 计算基频、桨叶数和谐波数的隶属度函数
function membership_value1 = demon_match_degree(input)
numOfPlatForm = size(input, 1); % 观测平台个数

% 1. 提取对应demon谱 轴频 桨叶数 谐波个数
demon = cell(numOfPlatForm, 1);
for i = 1:numOfPlatForm
    for j = 1:size(input{i}, 2)
        demon{i}{end + 1} = cell2mat(input{i}{j}(3:5));
    end
end
% 2. 对轴频、桨叶数和谐波个数分别设置隶属度函数
gaussian_membership = @(x1, x2, delta) exp(-0.5*((x2 - x1) / (delta^2)).^2); % 计算2对1的隶属度 高斯分布
uniform_membership = @(x1, x2) trapmf(x2, [x1 - 1, x1 - 1, x1 + 1, x1 + 1]); % 桨叶数 均匀分布
trapezoidal_membership = @(x1, x2) trimf(x2, [x1 - 1, x1, x1 + 1]); % 谐波个数 % 三角形分布
% 3. 将所有特征整合一下
all_feature = [];
for i = 1:numOfPlatForm
    for j = 1:size(demon{i}, 2)
        % 对于这个平台的每个目标
        all_feature = [all_feature; demon{i}{1, j}];
    end
end
% 4. 计算模糊度
for i = 1:size(all_feature, 1)
    for j = i:size(all_feature)
        % 对于每个特征，计算它们之间的隶属度， 对每个特征分别计算
        cur1 = all_feature(i, :);
        cur2 = all_feature(j, :);

        r1 = gaussian_membership(cur1(1), cur2(1), 0.5);
        r2 = uniform_membership(cur1(2), cur2(2));
        r3 = trapezoidal_membership(cur1(3), cur2(3));

        membership_value1(i, j) = (r1 + r2 + r3) / 3;
    end
end

end

function membership_value2 = lineFre_match_degree(input)
numOfPlatForm = size(input, 1); % 观测平台个数
% 线谱处理
% 1. 提取线谱
fs = cell(numOfPlatForm, 1);
for i = 1:numOfPlatForm
    for j = 1:size(input{i}, 2)
        fs{i}{end + 1} = input{i}{j}{2};
    end
end
% 2. 扩展线谱
delta_f = 5; % 偏移为5Hz
Ffs = cell(size(fs)); % 带有频偏的频率 线谱特征集合
for i = 1:size(fs)
    for j = 1:size(fs{i}, 2)
        cur = cell2mat(fs{i}{j});
        new_cell = cell(1, 0);
        for k = 1:size(cur, 2)
            new_cell = [new_cell, [cur(k) - delta_f, cur(k) + delta_f]]; % 添加偏移
        end
        Ffs{i}{j} = new_cell;
    end
end
% 3. 构建隶属度函数
low_bound = 0.2; % 下界
up_bound = 0.6; % 上界
freq_membership1 = @(x) linsmf(x, [low_bound, up_bound]); % 偏大型梯形隶属度

% 4. 计算重叠相似度
alindex = 0;
res = cell(1, 1); % 模糊关系矩阵
res1 = zeros(1, 1); % 计算和
for ii = 1:size(Ffs, 1)
    for jj = 1:size(Ffs{ii}, 2)
        alindex = alindex + 1; % 作为列
        base_baseture = Ffs{ii}{jj}; % 每一行是一个目标的特征 如果平台1的特征更少一些
        index = 1; % 作为行
        for i = 1:size(Ffs, 1)
            for j = 1:size(Ffs{i}, 2)
                cur = Ffs{i}{j};
                for k = 1:size(base_baseture, 1)
                    res{index, k} = zeros(1, size(cur, 2));
                    for l = 1:size(cur, 2)
                        xx = calculate_overlap(cur{l}, {base_baseture{k, :}});
                        res{index, k}(l) = xx;
                    end
                    res1(index, alindex) = sum(res{index, k}) / size(res{index, k}, 2); % 将重叠度累加
                end
                index = index + 1;
            end
        end
    end
end
% 5. 计算模糊度
for i = 1:size(res1, 1)
    for j = i:size(res1(i, :), 2)
        membership_value2(i, j) = freq_membership1(res1(i, j));
    end
end

end
function feature_vector = generate_feature_vector(feature1)
% 解析输入特征
% 参考 《基于线谱和 ＤＥＭＯＮ 谱的水声目标分类》
num_lines = feature1{1}; % 线谱数量
line_freqs = feature1{2}; % 线谱频率
axis_freq = feature1{3}; % 轴频 是谐波的最大公约数
num_blades = feature1{4}; % 叶片数   叶片速率谱通常在叶频处强度最大，3-7叶桨叶频在3-7倍轴频
num_harmonics = feature1{5}; % 谐波个数

% 生成特征向量
% 1. 线谱频率
generated_line_freqs = line_freqs; % 先初始化为原始数据
if randi([1, 20]) == 1
    % 以10%的概率删除一个线谱频率
    if numel(generated_line_freqs) > 1
        idx = randi(numel(generated_line_freqs));
        generated_line_freqs(idx) = [];
    end
elseif randi([1, 20]) == 2
    % 以10%的概率增加一个线谱频率
    idx = randi(numel(generated_line_freqs)+1);
    generated_line_freqs = [generated_line_freqs(1:idx-1), {randi([100, 1000])}, ...
        generated_line_freqs(idx:end)];
end

% 转换为数组形式
generated_line_freqs = cell2mat(generated_line_freqs);

% 2. 线谱频率高斯分布 均值 方差 点数
generated_line_freqs = sort(generated_line_freqs+normrnd(0, 3, size(generated_line_freqs)));
num_lines = size(generated_line_freqs, 2);

% 3. 轴频、叶片数和谐波个数均匀分布
generated_axis_freq = axis_freq + 0.3 * randi([-1, 1]);
generated_num_blades = num_blades + datasample([-1, 0, 0, 0, 0, 1], 1);
generated_num_harmonics = num_harmonics + datasample([-2, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 2], 1);

% 构建特征向量
feature_vector = {num_lines, {generated_line_freqs}, generated_axis_freq, ...
    generated_num_blades, generated_num_harmonics};
end

function feature_vector = create_new_feature_vector()
num_lines = randi([1, 10]); % 1. 线谱数量
line_freqs = sort(randi([100, 1000], 1, num_lines)); % 2. 线谱频率
axis_freq = randi([2, 10]); % 3. 轴频
num_blades = randi([3, 7]); % 4. 叶片数
num_harmonics = randi([3, 8]); % 5. 谐波个数

% 构建特征向量
feature_vector = {num_lines, {line_freqs}, axis_freq, num_blades, num_harmonics};
end



%%
function res = nearest_value(x, array)
min_diff = inf;
nearest_val = [];
for i = 1:size(array{1}, 2)
    diff = norm(array{1}(i)-x);
    if diff < min_diff
        min_diff = diff;
        nearest_val = array{1}(i);
    end
end
res = nearest_val;
end

%%
function res = calculate_overlap(interval1, interval2)
a1 = interval1(1);
b1 = interval1(2);
res = 0;
for i = 1:size(interval2, 2)
    a2 = interval2{i}(1);
    b2 = interval2{i}(2);

    % 计算重叠部分的长度
    overlap_length = min(b1, b2) - max(a1, a2);
    % 计算重叠度
    overlap = overlap_length / (max(b1, b2) - min(a1, a2));

    % 处理没有重叠的情况
    if overlap < 0
        overlap = 0;
    end
    res = max(res, overlap);
end
% disp(res);
end
% 判断一个 cell 是否是另一个 cell 的子集
function is_sub = isSubset(cell1, cell2)
is_sub = all(ismember(cell1, cell2, 'rows'), 'all');
end

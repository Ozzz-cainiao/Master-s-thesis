%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\main.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-12
% 描述: 仿真1 算法可行性分析

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
var2d = 0.2^2; % 角度制  角度误差
Pd = 0.9; % 检测概率
lambda = 0.1; % 假设虚警率为 0.1
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
feature2 = {8, {225,380, 420, 460, 550, 620, 710, 820}, 4.7, 3, 5}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source2 = SoundSource('CW', feature2, 100, initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [2e3, 5e3]; % 初始位置目标2
velocity3 = [0, 0]; % 运动速度
acc3 = 0; % 加速度
feature3 = {4, {320, 455, 560, 750}, 6.5, 7, 8}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
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
tic;
%% 获取每个平台的每个目标信息
count1 = 0;
count2 = 0;

% parfor i = 1:numOfM
for i = 1 : numOfM
    T_R = []; % 真实新类编号
    T1 = []; % 添加目标号
    max_val = 0;
    tag = cell(numOfPlatForm, numOfSource);
    P_Featu = cell(numOfPlatForm, 1);
    for j = 1:numOfPlatForm % 遍历平台
        % 平台观测到的特征向量
        for k = 1:numOfSource % 遍历声源
            [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
            % 虚警
            % 判断是否检测到目标
            rand_num = rand;
            if rand_num <= Pd
                P_Featu{j}{end + 1} = generate_feature_vector(fre);
                tag{j, k} = int2str(k);
                if j == 1
                    max_val = max_val + 1;
                    T_R(end + 1) = max_val;
                    T1{end + 1} = int2str(k);
                else
                    if ~ismember(int2str(k), T1)
                        max_val = max_val + 1;
                        T1{end + 1} = int2str(k);
                        T_R(end + 1) = max_val;
                    else
                        for p = 1 : length(T1)
                            if isequal(T1{p}, int2str(k))
                                T1{end + 1} = int2str(k);
                                T_R(end + 1) = T_R(p);
                                break;
                            end
                        end    
                    end
                end   
                % 生成泊松随机数
                num_false_alarms = poissrnd(lambda); % 生成虚警次数          
                while  num_false_alarms > 0
                    max_val = max_val + 1;
                    T1{end + 1} = ['y',int2str(max_val)];
                    T_R(end + 1) = max_val;
                    num_false_alarms = num_false_alarms - 1;
                    P_Featu{j}{end + 1} = create_new_feature_vector();
                    tag{j, k} = [tag{j, k}, 'y'];
                end
            else
                continue;
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm
    [res1, res2] = specific(P_Featu, tag, T_R);
    count1 = count1 + res1;
    count2 = count2 + res2;
end
fprintf("正确率为%d\n", count2);
toc;
% 对特征向量进行区分的函数
% function [result] = specific(input, tag)
% 输入：各平台测量的特征，与设置的目标标记
% 输出：设置目标个数与真实目标个数是否一致
function [result1, result2] = specific(input, tag, T_R)
% 计算模糊度矩阵
membership_value1 = demon_match_degree(input);
membership_value2 = lineFre_match_degree(input);

% 加权两次隶属度矩阵
all_membership = (membership_value1 + membership_value2) / 2;
% for i = 1 : size(all_membership, 1)
%     for j = i + 1 : size(all_membership, 2)
%         all_membership(j, i) = all_membership(i, j);
%     end
% end
all_membership(all_membership < 0.2) = 0;
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
delta = 0.5; % 关联度门限
index = 0;
new_feature_index = [];
base_feature_index = 1:base_len;
TT = 1: base_len; % 用于存储聚类标签
while p <= size(input, 1)
    index = index + size(input{p-1}, 2);
%     fprintf("当前起始索引%d\n", index);
    % 取出对应的关联度矩阵
    this_len = size(input{p}, 2); % 此平台的量测个数
    base_feature_index = [base_feature_index, new_feature_index];
    matrix = all_membership(base_feature_index, index+1:index+this_len); % 这个有问题
    new_feature_index = [];
    R = zeros(size(matrix));
    % 尝试匈牙利算法做匹配
    if size(matrix, 1) > size(matrix, 2) % HungarianAlgorithm.m只能对列数≥行数的正确关联
        [~, zeta] = HungarianAlgorithm(-matrix');
        zeta = zeta';
    else
        [~, zeta] = HungarianAlgorithm(-matrix);
    end
    % 提取出对应的隶属度
    new_matrix = matrix .* zeta;
    new_matrix(new_matrix < delta) = 0;

    for i = 1 : this_len
        if all(new_matrix(:, i) == 0)
            fprintf("全为0，平台%d测量%d是新的一类\n", p, i);
            new_feature_index = [new_feature_index, i + index];
            base_len = base_len + 1;
            base_feature{base_len} = book(i + index, :);
            TT(end + 1) = base_len;
        else 
            % 找到这一列最大的关联度
            [~, I] = max(zeta(:, i));
            base_feature{I} = [base_feature{I}; book(i + index, :)];
            TT(end + 1) = I;
        end
    end
    % 下方是自己编写的分类函数，
%     % 按列看
%     for i = 1:this_len
%         % 先判断是否全为0
%         if all(matrix(:, i) == 0)
%             fprintf("全为0，平台%d测量%d是新的一类\n", p, i);
%             new_feature_index = [new_feature_index, i + index];
%             base_len = base_len + 1;
%             base_feature{base_len} = book(i + index, :);
%             TT(end + 1) = base_len;
%         else
%             % 找到这一列最大的关联度
%             [M, I] = max(matrix(:, i));
%             if R(I, i) == 0 && M >= delta
%                 R(:, i) = 1;
%                 R(I, :) = 1; % 证明这个已经关联上了 % 将行置为1
%                 base_feature{I} = [base_feature{I}; book(i + index, :)];
%                 TT(end + 1) = I;
%             elseif R(I, i) == 0 && M < delta
%                 % 添加为新的一类
%                 fprintf("R = 0, M < delta, 平台%d测量%d是新的一类\n", p, i);
%                 new_feature_index = [new_feature_index, i + index];
%                 base_len = base_len + 1;
%                 base_feature{base_len} = book(i + index, :);
%                 TT(end + 1) = base_len;
% %             elseif R(I, i) == 1 && M < delta
%             elseif R(I, i) == 1
%                 fprintf("R = 1, M < delta, 平台%d测量%d是新的一类\n", p, i);
%                 new_feature_index = [new_feature_index, i + index];
%                 base_len = base_len + 1;
%                 base_feature{base_len} = book(i + index, :);
%                 TT(end + 1) = base_len;
% %             else
% %                 fprintf("平台%d测量%d已经被关联过了\n", p, i)
%             end
%         end
%     end
    p = p + 1;
end
fprintf("共分类%d组\n", base_len);
% 使用 contains 函数检查包含 'y' 的元素数量
totalCount = sum(cellfun(@(x) ischar(x) && contains(x, 'y'), tag(:)));
fprintf("实际有%d组\n", totalCount+3);
result1 = isequal(T_R, TT);
average_silhouette1 = mean(silhouette(all_membership, TT));

%% 层次聚类算法————可用
for i = 1 : size(all_membership, 1)
    for j = i + 1 : size(all_membership, 2)
        all_membership(j, i) = all_membership(i, j);
    end
end
distance = 1 - all_membership;
Z = linkage(distance,'single');
% dendrogram(Z);
T = cluster(Z, 'cutoff', 1.2, 'criterion', 'distance');
% 假设 T 是层次聚类的结果向量
num_clusters = max(T);
disp(['数据被划分为 ', num2str(num_clusters), ' 个簇。']);

average_silhouette2 = mean(silhouette(all_membership, T));
disp(average_silhouette1)
disp(average_silhouette2)

result2 = isequal(num_clusters, totalCount+3);
% disp(['最好的silhouette_vals聚类数量为', num2str(best_k)]);
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
uniform_membership = @(x1, x2) trapmf(x2, [x1 - 2, x1-1, x1+1, x1 + 2]); % 桨叶数 均匀分布
trapezoidal_membership = @(x1, x2) trimf(x2, [x1 - 2, x1, x1 + 2]); % 谐波个数 % 三角形分布
% 3. 将所有特征整合一下
all_feature = [];
for i = 1:numOfPlatForm
    for j = 1:size(demon{i}, 2)
        % 对于这个平台的每个目标
        all_feature = [all_feature; demon{i}{1, j}];
    end
end
% 4. 计算模糊度
membership_value1 = zeros(size(all_feature, 1));
for i = 1:size(all_feature, 1)
    for j = i:size(all_feature, 1)
        % 对于每个特征，计算它们之间的隶属度， 对每个特征分别计算
        cur1 = all_feature(i, :);
        cur2 = all_feature(j, :);

        r1 = gaussian_membership(cur1(1), cur2(1), 1);
        r2 = uniform_membership(cur1(2), cur2(2));
        r3 = trapezoidal_membership(cur1(3), cur2(3));

        membership_value1(i, j) = (r1 + r2 + r3) / 3;
    end
end

end

%%
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
delta_f = 10; % 偏移为5Hz
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
                        res{index, k}(l) = calculate_overlap(cur{l}, {base_baseture{k, :}});
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

%%
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
generated_num_blades = num_blades + datasample([-1, 0, 0, 0, 0,0, 0, 0, 0, 1], 1);
generated_num_harmonics = num_harmonics + datasample([-1, -1, 0, 0, 0, 0, 0, 0, 1, 1], 1);

% 构建特征向量
feature_vector = {num_lines, {generated_line_freqs}, generated_axis_freq, ...
    generated_num_blades, generated_num_harmonics};
end

%%
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

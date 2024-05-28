%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\main4.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-05-01
% 描述: 实现与层次聚类算法的虚警性能对比 虚警概率 仿真2-2
% 输入:
% 输出:
%**************************************************************************

clc;
clear;
close all;
numOfM = 1000; % 控制蒙特卡洛的次数

%% 观测数据
T = 0.1; %观测周期
T_all = 0.1; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.2^2; % 角度制  角度误差
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
feature2 = {8, {225, 380, 420, 460, 550, 620, 710, 820}, 4.7, 3, 5}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
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

all_Pd = [1, 0.95, 0.9, 0.85, 0.8];
all_lambda = [0, 0.05, 0.1, 0.2,0.5, 0.8,1]; % 假设虚警率为 0.1
count1 = zeros(size(all_lambda));
count2 = zeros(size(all_lambda));
index = 0;
Pd = 0.9;
for lambda = all_lambda
    index = index + 1;
    % parfor i = 1:numOfM
%     angR = cell(numOfM, numOfPlatForm);
    for i = 1:numOfM
        T_R = []; % 真实新类编号
        T1 = []; % 添加目标号
        max_val = 0;
        tag = cell(numOfPlatForm, numOfSource);
        P_Featu = cell(numOfPlatForm, 1);
        
        for j = 1:numOfPlatForm % 遍历平台
            % 平台观测到的特征向量
            for k = 1:numOfSource % 遍历声源
                % 判断是否检测到目标
                rand_num = rand;
                if rand_num <= Pd
                    [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);

                    P_Featu{j}{end + 1} = generate_feature_vector(fre);
                    %                     angR{j}(i, end + 1) = angle + sqrt(var2d) * randn;
                    angR{j}(1, end + 1) = angle + sqrt(var2d) * randn;

                    tag{j, k} = int2str(k);
                    if j == 1
                        max_val = max_val + 1;
                        T_R(end+1) = max_val;
                        T1{end+1} = int2str(k);
                    else
                        if ~ismember(int2str(k), T1)
                            max_val = max_val + 1;
                            T1{end+1} = int2str(k);
                            T_R(end+1) = max_val;
                        else
                            for p = 1:length(T1)
                                if isequal(T1{p}, int2str(k))
                                    T1{end+1} = int2str(k);
                                    T_R(end+1) = T_R(p);
                                    break;
                                end
                            end
                        end
                    end
                    % 生成泊松随机数
                    num_false_alarms = poissrnd(lambda); % 生成虚警次数
                    while num_false_alarms > 0

                        max_val = max_val + 1;
                        T1{end+1} = ['y', int2str(max_val)];
                        T_R(end+1) = max_val; % 真实的聚类标记
                        num_false_alarms = num_false_alarms - 1;
                        P_Featu{j}{end + 1} = create_new_feature_vector();
                        tag{j, k} = [tag{j, k}, 'y'];
                        % angR{j}(i, end + 1) = randi([-180,180]);
%                         angR{i, j}(1, end + 1) = randi([-180,180]);
                        angR{j}(1, end + 1) = randi([-180,180]);

                    end
                else
                    continue;
                end
            end % for k = 1:numOfSource % 遍历声源
        end % for j = 1:numOfPlatForm
        [res1, res2] = specific(P_Featu, tag, T_R);
        count1(index) = count1(index) + res1;
        count2(index) = count2(index) + res2;
    end
end

toc;
fig = figure('Units', 'centimeters', 'Position', [20, 5, 12, 12 / 4 * 3]);
plot(all_lambda, count1./numOfM, 'o-')
hold on
plot(all_lambda, count2./numOfM, '*-');
title("本算法与层次聚类算法性能对比", 'FontSize', 12)
legend('模糊数学特征关联', '层次聚类算法', 'FontSize', 12, 'location', 'southeast')
xlabel('虚警率', 'FontSize', 12)
ylabel('聚类成功率', 'FontSize', 12)
ylim([0, 1]);

% fig = figure('Units', 'centimeters', 'Position', [20, 5, 12, 12 / 4 * 3]);
% plot(all_Pd, count1./numOfM, 'o-')
% hold on
% plot(all_Pd, count2./numOfM, '*-');
% plot(all_Pd, count3./numOfM,'b-');
% title("本算法与层次聚类算法性能对比", 'FontSize', 12)
% legend('模糊数学特征关联', '层次聚类算法', '纯方位分治贪心算法','FontSize', 12, 'location', 'southeast')
% xlabel('检测率', 'FontSize', 12)
% ylabel('聚类成功率', 'FontSize', 12)
% ylim([0, 1]);

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
TT = 1:base_len; % 用于存储聚类标签
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

    for i = 1:this_len
        if all(new_matrix(:, i) == 0)
            fprintf("全为0，平台%d测量%d是新的一类\n", p, i);
            new_feature_index = [new_feature_index, i + index];
            base_len = base_len + 1;
            base_feature{base_len} = book(i + index, :);
            TT(end+1) = base_len;
        else
            % 找到这一列最大的关联度
            [~, I] = max(zeta(:, i));
            base_feature{I} = [base_feature{I}; book(i + index, :)];
            TT(end+1) = I;
        end
    end
    p = p + 1;
end
fprintf("共分类%d组\n", base_len);
% 使用 contains 函数检查包含 'y' 的元素数量
totalCount = sum(cellfun(@(x) ischar(x) && contains(x, 'y'), tag(:)));
fprintf("实际有%d组\n", totalCount+3);
result1 = 0;
if isequal(T_R, TT)
    result1 = 1;
end
average_silhouette1 = mean(silhouette(all_membership, TT));

%% 层次聚类算法————可用
for i = 1:size(all_membership, 1)
    for j = i + 1:size(all_membership, 2)
        all_membership(j, i) = all_membership(i, j);
    end
end
distance = 1 - all_membership;
Z = linkage(distance, 'single');
% dendrogram(Z);
% T = cluster(Z, 'cutoff', 1.2, 'criterion', 'distance');
% num_clusters = max(T);
min_clu = 2;
max_clu = 6;
for k = min_clu:max_clu
    T(k, :) = cluster(Z, "maxclust", k);
    average_silhouette2(k) = mean(silhouette(all_membership, T(k, :)));
end
[~, num_clusters] = max(average_silhouette2);
% 假设 T 是层次聚类的结果向量

disp(['数据被划分为 ', num2str(num_clusters), ' 个簇。']);

% average_silhouette2 = mean(silhouette(all_membership, T));
% disp(average_silhouette1)
% disp(average_silhouette2)
result2 = 0;
if isequal(num_clusters, totalCount+3)
    result2 = 1;
end
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
uniform_membership = @(x1, x2) trapmf(x2, [x1 - 2, x1 - 1, x1 + 1, x1 + 2]); % 桨叶数 均匀分布
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
generated_num_blades = num_blades + datasample([-1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 1);
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
%% ==========================子函数===============================
%dcgt 基于分治贪心思想的联合多站目标关联定位
function varargout = dcgt(varargin)

%   此处显示详细说明
% INPUTS:
%   varargin{1}   - 测量元胞
%   varargin{2}   - 节点位置
%   varargin{3}   - 分布参数
%   varargin{4}   - 处理参数
%
% OUTPUTS:
%   varargout{1}  - 关联组合+位置
% 判断Z是否为空

warning('off')
Z = varargin{1};
% % 去除空的内部 cell
% x = cellfun(@(x) ~isempty(x), Z);
% Z = Z(~cellfun('isempty', x));
node = varargin{2};
var2 = varargin{3}(1); % 角度测量方差单位rad
PD = varargin{3}(2); % 检测概率
Fai = varargin{3}(3);
M = varargin{4}(1); % 选取最少线点数
Q = varargin{4}(2); % 列表最大长度
I = varargin{4}(3); % 并行次优条数
pNum = size(node, 1); % 节点数
angM = arrayfun(@(x)cat(2, cell2mat(x{1, 1}(:, 1))), Z, 'un', 0); % 角度特征元胞组

%%% ==========================基于分治思想的最优交点集合选取===============================
%%% Step1：生成交点
% angM(:)：将矩阵angM展开成一个列向量，其中每个元素是原矩阵中的一个cell
% cellfun(@length, ...)：cellfun函数用于对cell数组的每个元素应用指定的函数。
nl = cellfun(@length, angM(:)); % 各个平台测量数量 2 2 0
% 计算非零行的个数
nonZeroRowCount = nnz(nl ~= 0);
disp(['非零行的个数为：', num2str(nonZeroRowCount)]);
if nonZeroRowCount < 2
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
subs = arrayfun(@(x)0:x, nl, 'un', 0); % 生成序号 从0开始是为了漏检的显示
iter = cell(pNum, 1);
[iter{end:-1:1}] = ndgrid(subs{end:-1:1}); % 生成网格
temp = cellfun(@(x)x(:), iter, 'un', 0); % 中间变量
iter = num2cell(cat(2, temp{:})); % 所有组合
Niter = size(iter, 1);
temp = arrayfun(@(x) find(cell2mat(iter(x, :))), 1:Niter, 'un', 0); % 中间变量
iter1 = iter(cellfun(@length, temp) == 1, :); % 所有线的序数组
iter2 = iter(cellfun(@length, temp) == 2, :); % 所有两两相交的组合
K = cell(size(iter2, 1), 2); % 预分配
for k = 1:size(iter2, 1) % 一次循环替代嵌套
    locs1 = find(cell2mat(iter2(k, :))); % 所选平台
    theta = cellfun(@(x, y)x(y), angM(locs1), iter2(k, locs1), 'un', 0); % 读取角度
    theta = cell2mat(theta)';
    x1 = node(locs1, 1);
    y1 = node(locs1, 2);
    A = [-tand(theta), ones(2, 1)];
    B = y1 - x1 .* tand(theta);
    X = (A' * A) \ A' * B; % 目标位置X=[x;y]
    K(k, :) = num2cell(X'); % 记录交点矩阵
end
iter22 = cat(2, iter2, K); % 所选平台测量序号+交点矩阵

%%% Step2：对每条线选取最短的M条线段
W = {};
for k = 1:size(iter1, 1)
    locs21 = find(cell2mat(iter1(k, :))); % 所选平台序号
    locsM = iter1{k, locs21}; % 所选测量序号
    Num22 = find(cell2mat(iter22(:, locs21)) == iter1{k, locs21}); % 在iter22筛选出来的序号，选取的测向线
    point = iter22(Num22, :); % 筛选出来的测量序号+位置
    kk = 1;
    kknum = 1;
    if size(point, 1) > 1
        locs22 = cell(sum(1:size(point, 1)-1), 3); % 预分配
        while kk < size(point, 1)
            point1 = cell2mat(point(kk, pNum+1:pNum+2)).';
            for ii = kk + 1:size(point, 1)
                point2 = cell2mat(point(ii, pNum+1:pNum+2)).';
                locs22{kknum, 1} = Num22(kk); % iter22中行序号--对应点
                locs22{kknum, 2} = Num22(ii); % iter22中行序号--对应点
                locs22{kknum, 3} = norm(point2-point1); % 解算点之间的距离
                kknum = kknum + 1;
            end
            kk = kk + 1;
        end
        [~, locs23] = sort(cell2mat(locs22(:, 3))); % 按升序排列后的序数
        if length(locs23) > M
            W = cat(1, W, locs22(locs23(1:M), :)); % W中三列分为iter22中行序号；iter22中行序号；距离
        else
            W = cat(1, W, locs22(locs23(1:length(locs23)), :)); % iter22中的组合x和y中间的距离
        end
    else

    end

end
%%% Step3：定位

Num3 = size(W, 1);
if Num3 == 0
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
Lamdad = cell(Num3, 2);
for k = 1:Num3
    locs31 = iter22(cell2mat(W(k, 1:2)), :); % locs31所选测量序号，和交点位置
    [locs32, locs33] = find(cell2mat(locs31(:, 1:pNum))); % locs32行序数->无意义 locs33列序数->所选平台
    if length(unique(locs33)) < 3
        locs35 = arrayfun(@(x, y) locs31(x, y), locs32, locs33); % locs35测量序数，与locs33配合读取测量
        theta = cellfun(@(x, y) angM{x}(y), num2cell(locs33), locs35, 'un', 0);
        theta = cell2mat(theta);
        Lamdad{k, 1} = cat(2, locs33, cell2mat(locs35), theta); % 平台序号;测量序号;测量值
        Lamdad{k, 2} = nan; % 似然
    else
        [locs33, locs34] = unique(locs33); % locs34独一无二数的平台数的序数， locs33列序数更新
        locs32 = locs32(locs34); % locs32行序数更新
        locs35 = arrayfun(@(x, y) locs31(x, y), locs32, locs33); % locs35测量序数，与locs33配合读取测量
        theta = cellfun(@(x, y) angM{x}(y), num2cell(locs33), locs35, 'un', 0);
        theta = cell2mat(theta);
        x3 = node(locs33, 1);
        y3 = node(locs33, 2);
        A = [-tand(theta), ones(3, 1)];
        B = y3 - x3 .* tand(theta);
        Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
        thetaE = atan2d(Xd(2)-y3, Xd(1)-x3); % 估计目标位置到平台角度
        %%% 换成对数似然
        c = zeros(1, pNum);
        for ii = 1:pNum
            if ismember(ii, locs33)
                detla_l = 1;
                thetaii = theta(ismember(locs33, ii));
                thetaEii = thetaE(ismember(locs33, ii));
                esis = AngelDeal(thetaii, thetaEii) / 180 * pi;
                c(ii) = detla_l * (-log(PD*Fai/sqrt(2*pi*var2)) + 0.5 * (esis / sqrt(var2))^2);
            else
                u = 0;
                c(ii) = (u - 1) * log(1-PD);
            end
        end
        Lamdad{k, 1} = cat(2, locs33, cell2mat(locs35), theta); % 平台序号;测量序号;测量值
        Lamdad{k, 2} = sum(c); % 似然
    end
end

%% ==========================基于贪心思想的量测集合合并===============================
% 下面这一句 Lamdad_W可能为空，因为Lamdad可能都是nan值
Lamdad_W = Lamdad(~isnan(cell2mat(Lamdad(:, 2))), :); % 排除一个组合有多个同平台测量
if isempty(Lamdad_W)
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
Num4 = size(Lamdad_W, 1);
%%% 对获得组合按平台-测量序数分
Set = cell(1, pNum);
for ii = 1:pNum
    for jj = 1:nl(ii)
        locs41 = arrayfun(@(x) sum(Lamdad_W{x, 1}(:, 1) == ii & Lamdad_W{x, 1}(:, 2) == jj), 1:Num4, 'un', 0);
        Set{ii} = cat(1, Set{ii}, Lamdad_W(logical(cell2mat(locs41')), :));
    end
end
s = 2;
Z = Set{1, s-1};
W_Plus = {};
while s <= pNum
    Z1 = {};
    z = Set{1, s};
    NumZ = size(Z, 1);
    Numz = size(z, 1);
    R = zeros(NumZ, Numz);
    for ii = 1:NumZ
        for jj = 1:Numz
            Mea = cat(1, Z{ii}, z{jj}); % 在下方拼接
            locs51 = arrayfun(@(x) find(Mea(:, 1) == x), 1:pNum, 'un', 0); % 平台1：S对应再矩阵的序号
            locs52 = cellfun(@length, locs51); % 有多余1个测量的序号
            Mea2 = cellfun(@(x) Mea(x, :), locs51(locs52 > 1), 'un', 0); % 有多余1个测量的Mea
            locs53 = arrayfun(@(x) unique(Mea2{1, x}(:, 2)), 1:length(Mea2), 'un', 0); % 有多余1个测量的平台中 重复测量数量
            locs54 = cellfun(@length, locs53);
            if isempty(find(locs54 > 1, 1))
                [locs55, locs56] = unique(Mea(:, 1));
                if length(locs55) == pNum % 遍历所有组合->输出
                    if isempty(W_Plus) % 第一个值
                        W_Plus = cat(1, W_Plus, {Mea(locs56, :)});
                        R(ii, jj) = 1; % 成功融合且没有输出过的标记为1
                    else % 已有输出集
                        for kk = 1:length(W_Plus) % 对输出集遍历
                            if sum(W_Plus{kk} == Mea(locs56, :), 'all') == numel(Mea(locs56, :)) % 如果和输出集相等
                                R(ii, jj) = 2; % 已成功融合但输出过的标记为2
                            end
                        end
                        if R(ii, jj) ~= 2
                            W_Plus = cat(1, W_Plus, {Mea(locs56, :)});
                            R(ii, jj) = 1; % 成功融合且没有输出过的标记为1
                        end
                    end
                else
                    if isempty(Z1) % 第一个值
                        Z1 = cat(1, Z1, {Mea(locs56, :)});
                        R(ii, jj) = 3; % 成功融合但未遍历所有平台的标记为3
                    else
                        for kk = 1:length(Z1) % 对输出集遍历
                            %                             if sum(Z1{kk}==Mea(locs56,:),'all') == numel(Mea(locs56,:))% 如果和输出集相等
                            %%% 以下判断条件20230919修改，此时s=5时出现错误
                            [Lia, ~] = ismember(Z1{kk}(:, 1:2), Mea(locs56, 1:2), 'rows');

                            if length(find(Lia)) == size(Mea(locs56, :), 1) % 如果Mea已经在输出集里面
                                R(ii, jj) = 4; % 已成功融合但未遍历所有平台的标记为4
                            end
                        end
                        if R(ii, jj) ~= 4
                            Z1 = cat(1, Z1, {Mea(locs56, :)});
                            R(ii, jj) = 3; % 成功融合但未遍历所有平台的标记为3
                        end
                    end

                end
            end
        end
    end
    %%%更新关联下一个平台
    [locs61, locs62] = find(R);
    numZ = 1:NumZ;
    if sum(ismember(numZ, locs61)) ~= NumZ % 未关联的行
        locs63 = numZ(~ismember(numZ, locs61)); % 未更新的行号
        Z1 = cat(1, Z1, Z{locs63, 1});
    end
    numz = 1:Numz;
    if sum(ismember(numz, locs62)) ~= Numz % 未关联的列
        locs64 = numz(~ismember(numz, locs62)); % 未关联的列号
        Z1 = cat(1, Z1, z{locs64, 1});
    end
    Z = Z1;
    s = s + 1;
end
W_Plus = cat(1, W_Plus, Z);

%%% 计算位置和关联似然概率并保留最大的Q个组合
Lamda7 = cell(length(W_Plus), 1);
for k = 1:length(W_Plus)
    Wk = W_Plus{k};
    theta = Wk(:, 3);
    x6 = node(Wk(:, 1), 1);
    y6 = node(Wk(:, 1), 2);
    A = [-tand(theta), ones(size(Wk, 1), 1)];
    B = y6 - x6 .* tand(theta);
    Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
    thetaE = atan2d(Xd(2)-y6, Xd(1)-x6); % 估计目标位置到平台角度
    p = arrayfun(@(x, y) DistributedDeal(x/180*pi, y/180*pi, var2), theta, thetaE);
    Lamda7{k} = prod(p);
    %%% 换成对数似然
    c = zeros(1, pNum);
    for ii = 1:pNum
        if ismember(ii, Wk(:, 1))
            detla_l = 1;
            thetaii = theta(ismember(Wk(:, 1), ii));
            thetaEii = thetaE(ismember(Wk(:, 1), ii));
            esis = AngelDeal(thetaii, thetaEii) / 180 * pi;
            c(ii) = detla_l * (-log(PD*Fai/sqrt(2*pi*var2)) + 0.5 * (esis / sqrt(var2))^2);
        else
            u = 0;
            c(ii) = (u - 1) * log(1-PD);
        end
    end
    Lamda7{k} = sum(c);
end
W_Plus = cat(2, W_Plus, Lamda7);
[~, locs71] = sort(cell2mat(Lamda7), 'descend');
[~, locs71] = sort(cell2mat(Lamda7));
if length(locs71) > Q
    W_Plus7 = W_Plus(locs71(1:Q), :);
else
    W_Plus7 = W_Plus(locs71, :);
end
%%% ==========================基于贪心思想的量测总集划分===============================
%%%寻找不互斥的输出测量组
R8 = zeros(size(W_Plus7, 1));
for ii = 1:size(W_Plus7, 1)
    W81 = W_Plus7{ii, 1}(:, 1:2);
    for jj = 1:size(W_Plus7, 1)
        W82 = W_Plus7{jj, 1}(:, 1:2);
        locs81 = cat(1, W81, W82); % 对两个集合拼接
        locs82 = arrayfun(@(x) locs81(locs81(:, 1) == x, 2), unique(locs81(:, 1)), 'un', 0); % 平台测量数
        locs83 = cellfun(@(x) length(unique(x)) == length(x), locs82, 'un', 0); % 平台测量互斥性，如果1不互斥；如果0则互斥
        if length(find(cell2mat(locs83))) == length(locs82) % 如果不互斥的平台数==有测量平台数
            R8(ii, jj) = 1; % 不互斥的标记为1
        end
    end
end
% 判断W_Plus7是否为空
q = 2;
Psi = {{}; W_Plus7{q-1, 1}}; % 索引容易超出数组边界：W_Plus7可能为空
L = [0; 1];
P = [0; W_Plus7{q-1, 2}];
while q <= size(W_Plus7, 1)
    NumPsi = size(Psi, 1);
    Psi = cat(2, repmat(Psi, 2, 1), cat(1, cell(NumPsi, 1), repmat(W_Plus7(q, 1), NumPsi, 1))); % 兼容组合
    L = cat(2, repmat(L, 2, 1), cat(1, zeros(NumPsi, 1), ones(NumPsi, 1))); % 兼容组合索引
    P = sum(cat(2, repmat(P, 2, 1), cat(1, zeros(NumPsi, 1), W_Plus7{q, 2}*ones(NumPsi, 1))), 2); % 总似然
    locsdet = [];
    for ii = NumPsi + 1:NumPsi * 2
        Lq = L(ii, :);
        locs91 = find(Lq == 1)';
        if sum(Lq == 1) == 2 % 存在两个关联量则检查是否能同时输出
            if R8(locs91(1), locs91(2)) ~= 1 % 不可以同时输出
                locsdet = cat(1, locsdet, ii);
            end
        elseif sum(Lq == 1) > 2
            locs92 = [locs91(1:end-1), locs91(end) * ones(sum(Lq == 1)-1, 1)]; % 待处理的兼容组合
            if sum(arrayfun(@(x, y) R8(x, y) == 1, locs92(:, 1), locs92(:, 2))) ~= sum(Lq == 1) - 1 % 没有和所有兼容
                locsdet = cat(1, locsdet, ii);
            end
        end
    end
    Psi = Psi(~ismember(1:end, locsdet), :);
    L = L(~ismember(1:end, locsdet), :);
    P = P(~ismember(1:end, locsdet), :);
    if length(P) > I
        [~, locs93] = sort(P, 'descend');
        [~, locs93] = sort(P);
        Psi = Psi(locs93(1:I), :);
        L = L(locs93(1:I), :);
        P = P(locs93(1:I), :);
    end
    q = q + 1;
end

%%% ==========================求解位置和关联组合===============================
%%% 以下判断条件20230919修改，此时出现Psi(1,:)={}情况
if isempty(Psi{1, 1}) && ~isempty(Psi{2, 1})
    Psi_opt = Psi(2, :);
else
    Psi_opt = Psi(1, :);
end
% Psi_opt = Psi(1,:);
Xout = [];
Pos = zeros(length(Psi_opt), pNum);
for ii = 1:length(Psi_opt)
    if ~isempty(Psi_opt{ii})
        %%% 求解位置
        theta = Psi_opt{ii}(:, 3);
        x9 = node(Psi_opt{ii}(:, 1), 1);
        y9 = node(Psi_opt{ii}(:, 1), 2);
        A = [-tand(theta), ones(size(Psi_opt{ii}, 1), 1)];
        B = y9 - x9 .* tand(theta);
        Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
        Xout = cat(2, Xout, Xd);
        %%% 求解组合数
        Pos(ii, Psi_opt{ii}(:, 1)) = Psi_opt{ii}(:, 2);
    else
        Pos(ii, :) = nan;
    end
end
%%% ==========================求解关联组合===============================
Pos1 = Pos(~isnan(Pos));
Pos2 = reshape(Pos1, [], pNum);

%%% ==========================输出===============================

varargout{1} = [Pos2, Xout'];

end
%% 最小二乘法定位
function [EstX, EstY] = LSM(Zt, node)
theta = Zt;
theta = theta(~isnan(Zt))';
x1 = node(~isnan(Zt), 1);
y1 = node(~isnan(Zt), 2);
A = [-tand(theta), ones(length(x1), 1)];
B = y1 - x1 .* tand(theta);
X = (A' * A) \ A' * B; % 目标位置X=[x;y]
if isempty(X)
    EstX = inf;
    EstY = inf;
else
    EstX = X(1);
    EstY = X(2);
end
end

%% 角度处理
function A_X_Y = AngelDeal(X, Y)
A_XY = mod(X-Y, 360);
A_YX = mod(Y-X, 360);
A_X_Y = min(A_XY, A_YX);
end

%% 子函数--角度变成概率函数
function P = DistributedDeal(theta1, theta2, thetas2)
if theta1 - theta2 < -pi
    P = normpdf(theta1, theta2-2*pi, thetas2);
elseif theta1 - theta2 > pi
    P = normpdf(theta1, theta2+2*pi, thetas2);
else
    P = normpdf(theta1, theta2, thetas2);
end
end

%%
function [cost, x] = HungarianAlgorithm(C)
% The implementation of the Hungarian algorithm.
% Reference:
% https://csclab.murraystate.edu/~bob.pilgrim/445/munkres_old.html
% C: cost matrix
[n, m] = size(C);
k = min(n, m);
starMatrix = zeros(n, m);
primedMatrix = zeros(n, m);
coveredRow = zeros(n, 1);
coveredColumn = zeros(1, m);
% Step1: For each row of the matrix, find the smallest element and subtract
% it from every element in its row. Go to Step 2.
C1 = zeros(n, m);
parfor i = 1:n
    tempV = min(C(i, :));
    C1(i, :) = C(i, :) - tempV;
end
% Step2: Find a zero (Z) in the resulting matrix. If there is no starred
% zero in its row or column, star (Z). Repeat for each element in the
% matrix. Go to Step 3.
for i = 1:n
    for j = 1:m
        if C1(i, j) == 0 && (sum(starMatrix(i, :)) == 0) && (sum(starMatrix(:, j)) == 0)
            starMatrix(i, j) = 1;
        end
    end
end
flag = zeros(1, 5);
flag(1) = 1;
while flag(5) == 0
    % Step3: Cover each column containing a starred zero. If K columns are
    % covered the starred zeros describe a complete set of unique
    % assignments. In this case, Go to DOWN, otherwise, Go to Step 4.
    if flag(1) == 1
        parfor j = 1:m
            temp = sum(starMatrix(:, j));
            if temp > 0
                coveredColumn(j) = 1;
            end
        end
        if sum(coveredColumn) == k
            flag = zeros(1, 5);
            flag(5) = 1;
        else
            flag = zeros(1, 5);
            flag(2) = 1;
        end
    end
    % Step4: Find a noncovered zero and prime it. If there is no starred
    % zero in the row containing this primed zero. Go to Step 5. Otherwise,
    % cover this row and uncover the column containing the starred zero.
    % Continue in this manner until there are no uncovered zeros left. Save
    % the smallest uncovered value and Go to Step 6.
    if flag(2) == 1
        tempC = C1 + ones(n, 1) * coveredColumn + coveredRow * ones(1, m);
        [idx1, idx2] = find(tempC == 0);
        for j = 1:length(idx1)
            primeIdx1 = idx1(j);
            primeIdx2 = idx2(j);
            primedMatrix(primeIdx1, primeIdx2) = 1;
            if sum(starMatrix(primeIdx1, :)) == 0
                % Go to Step 5:
                flag = zeros(1, 5);
                flag(3) = 1;
                break;
            else
                coveredRow(primeIdx1) = 1;
                idx = find(starMatrix(primeIdx1, :) == 1);
                coveredColumn(idx) = 0;
            end
        end
        if flag(3) == 0
            m1 = coveredRow * ones(1, m) * inf;
            m1(find(isnan(m1) == 1)) = 0;
            m2 = ones(n, 1) * coveredColumn * inf;
            m2(find(isnan(m2) == 1)) = 0;
            tempC = C1 + m1 + m2;
            minValue = min(min(tempC));
            [smallestIdx1, smallestIdx2] = find(tempC == minValue);
            smallestIdx1 = smallestIdx1(1);
            smallestIdx2 = smallestIdx2(1);
            % Go to Step 6:
            flag = zeros(1, 5);
            flag(4) = 1;
        end
    end
    if flag(3) == 1
        % Step5: Construct a series of alternating primed and starred zeros
        % as follows. Let Z0 represent the uncovered primed zero found in
        % Step 4. Let Z1 denote the starred zero in the column of Z0 (if
        % any). Let Z2 denote the primed zero in the row of Z1 (there will
        % always be one). Continue until the series terminates at a primed
        % zero that has no starred zero in its column. Unstar each starred
        % zero of the series, star each primed zero of the series, erase
        % all primes and uncover every line in the matrix. Return to Step
        % 3.
        tempFlag = true;
        Z = [primeIdx1, primeIdx2];
        while tempFlag
            starIdx1 = find(starMatrix(:, primeIdx2) == 1);
            starMatrix(primeIdx1, primeIdx2) = 1;
            if isempty(starIdx1)
                tempFlag = false;
            else
                tempZ = [starIdx1(1), primeIdx2];
                Z = [Z; tempZ];
                starMatrix(tempZ(1), tempZ(2)) = 0;
                primeIdx1 = tempZ(1);
                primeIdx2 = find(primedMatrix(primeIdx1, :) == 1);
                primeIdx2 = primeIdx2(1);
                tempZ = [primeIdx1, primeIdx2];
                Z = [Z; tempZ];
            end
        end
        primedMatrix = zeros(n, m);
        coveredRow = zeros(n, 1);
        coveredColumn = zeros(1, m);
        % Return to Step 3:
        flag = zeros(1, 5);
        flag(1) = 1;
    end
    if flag(4) == 1
        % Step6: Add the value found in Step 4 to every element of each
        % covered row, and subtract it from every element of each uncovered
        % column. Return to Step 4 without altering any stars, primes, or
        % covered lines.
        C1 = C1 + coveredRow * ones(1, m) * minValue;
        uncoveredColumn = ones(1, m) - coveredColumn;
        C1 = C1 - ones(n, 1) * uncoveredColumn * minValue;
        % Return to Step 4:
        flag = zeros(1, 5);
        flag(2) = 1;
    end
end
% DONE: Assignment pairs are indicated by the positions of the starred
% zeros in the cost matrix. If C(i,j) is a starred zeros, then the element
% associated with row i is assigned to the element associated with column
% j.
x = starMatrix;
cost = 0;
[idx1, idx2] = find(starMatrix == 1);
for j = 1:length(idx1)
    cost = cost + C(idx1(j), idx2(j));
end


end
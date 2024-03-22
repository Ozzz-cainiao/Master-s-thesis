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
feature1 = {5, {460, 580, 650, 790, 880}, 160, 4, 3}; % 线谱数量， 线谱频率, 调制谱的基频是轴频，叶片数、谐波个数、水声目标识别
source1 = SoundSource('CW', feature1, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [0, 0]; % 运动速度
acc2 = 0; % 加速度
feature2 = {8, {380, 420, 460, 550, 620, 710, 790, 880}, 400, 3, 1}; % 线谱数量， 线谱频率
source2 = SoundSource('CW', feature2, 100, initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [2e3, 5e3]; % 初始位置目标2
velocity3 = [0, 0]; % 运动速度
acc3 = 0; % 加速度
feature3 = {4, {320, 455, 560, 730}, 420, 2, 2}; % 线谱数量， 线谱频率
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

numOfM = 1; % 控制蒙特卡洛的次数
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
                tag{j, k}= {'y', k};
                
             
            % 漏报
            elseif randi([1, 20]) == 2
                continue;
            % 正常
            else
                P_Featu{j}{end + 1} = generate_feature_vector(fre);
                tag{j, k}{end + 1} = k;
            end
            %             P_Featu{j}{end + 1} = generate_feature_vector(fre);
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm
    % 本次观测内
    specific(P_Featu);

    n = 10;

end
% 对特征向量进行区分的函数
% input: 行：平台数，列 特征向量个数
% 1. 获取线谱特征， 将线谱特征根据相对多普勒频移进行扩展，获取线谱特征集合
% 2. 根据线谱特征集合，构建模糊隶属度函数（梯形分布偏大型或中间型）
% 3. 根据模糊隶属度函数构建线谱模糊关系矩阵元素，利用元素构建模糊矩阵
function specific(input)

% 获取线谱特征，
fs = cell(size(input));
for i = 1:size(input)
    for j = 1:size(input{i}, 2)
        fs{i}{end + 1} = input{i}{j}{2};
    end
end

%% 扩展线谱
delta_f = 5; % 偏移为5Hz
Ffs = cell(size(fs)); % 带有频偏的频率 线谱特征集合
for i = 1:size(fs)
    for j = 1:size(fs{i}, 2)
        % 每个元素变为一个区间
%         disp(fs{i}{j});
        cur = cell2mat(fs{i}{j});
        new_cell = cell(1, 0);
        for k = 1:size(cur, 2)
            % 添加偏移
            new_cell = [new_cell, [cur(k) - delta_f, cur(k) + delta_f]];
        end
%         disp(new_cell);
        Ffs{i}{j} = new_cell;
    end
end
% 构建隶属度函数
low_bound = 0.3; % 下界
up_bound = 0.7; % 上界
% freq_membership1 = @(x) trapmf(x, [a{1}-2, a{1}, a{1}+1, a{1}+2]); % 梯形隶属度函数
freq_membership2 = @(x) linsmf(x, [low_bound, up_bound]); % 偏大型梯形隶属度
% 初始化已关联线谱特征子集 初始化为平台1的所有观测特征向量
base_baseture = Ffs{1}'; % 每一行是一个目标的特征

% 计算重叠相似度
index = 1;
res = cell(1, 1); % 模糊关系矩阵
res1 = cell(1, 1); % 计算和
for i = 2:size(Ffs, 1)
    for j = 1:size(Ffs{i}, 2)
        %         disp(Ffs{i}{j}); % 几个频率区间
        cur = Ffs{i}{j};
        % 遍历计算重叠度
        for k = 1:size(base_baseture, 1)
            res{index, k} = zeros(1, size(cur, 2));
            for l = 1:size(cur, 2)
                xx = calculate_overlap(cur{l}, base_baseture{k});
                res{index, k}(l) = xx;
            end
            res1{index, k} = sum(res{index, k}) / size(res{index, k}, 2); % 将重叠度累加
        end
        index = index + 1;
    end

end
% 计算隶属度
for i = 1:size(res, 1)
    for j = 1:size(res, 2)

    end
end


end

function feature_vector = generate_feature_vector(feature1)
% 解析输入特征
num_lines = feature1{1}; % 线谱数量
line_freqs = feature1{2}; % 线谱频率
axis_freq = feature1{3}; % 轴频
num_blades = feature1{4}; % 叶片数
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

% 2. 线谱频率高斯分布
generated_line_freqs = sort(generated_line_freqs+normrnd(0, 3, size(generated_line_freqs)));

num_lines = size(generated_line_freqs, 2);

% 3. 其他异常频率
% 在 1 到 1000 之间生成一个随机异常频率
% anomaly_freq = randi([1, 1000]);
% 4. 轴频、叶片数和谐波个数均匀分布
generated_axis_freq = axis_freq + randi([-1, 1]);
generated_num_blades = num_blades + randi([-1, 1]);
generated_num_harmonics = num_harmonics + randi([-1, 1]);

% 构建特征向量
feature_vector = {num_lines, {generated_line_freqs}, generated_axis_freq, ...
    generated_num_blades, generated_num_harmonics};
end

function feature_vector = create_new_feature_vector()
% 1. 线谱数量
num_lines = randi([1, 10]);

% 2. 线谱频率
line_freqs = sort(randi([400, 1000], 1, num_lines));

% 3. 轴频
axis_freq = randi([100, 500]);

% 4. 叶片数
num_blades = randi([1, 10]);

% 5. 谐波个数
num_harmonics = randi([1, 10]);

% 6. 其他异常频率
anomaly_freq = randi([1, 1000]);

% 构建特征向量
feature_vector = {num_lines, {line_freqs}, axis_freq, num_blades, num_harmonics};
end


% 计算b对a的隶属度
function res = match_degree(a, b)

% 定义隶属度函数
num_lines_membership = @(x) trapmf(x, [a{1} - 2, a{1}, a{1} + 1, a{1} + 2]);
freq_membership = @(x, center) trapmf(x, [center - 5, center - 1, center + 1, center + 5]); % 频率隶属度函数为以特征值为中心的高斯分布，方差为3,标准差就是√3

% freq_membership = @(x, center) normpdf(x, center, 9); % 频率隶属度函数为以特征值为中心的高斯分布，方差为3,标准差就是√3

% 计算b对a隶属度
b_num_lines_membership = num_lines_membership(b{1});

for i = 1:numel(b{2}{1})
    nv = nearest_value(b{2}{1}(i), a{2});
    b_freq_membership(i) = freq_membership(b{2}{1}(i), nv);
end

% 将隶属度加权
res = 0.2 * b_num_lines_membership + 0.8 * sum(b_freq_membership);
end
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

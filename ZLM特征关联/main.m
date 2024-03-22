%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\main.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-12
% 描述: 特征关联主程序，在这个程序内部实现灰色关联和模糊数学关联,
%       暂时不加传播时延，使用蒙特卡罗统计

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
feature3 = {5, {320, 455, 560, 730, 890}, 420, 2, 2}; % 线谱数量， 线谱频率
source3 = SoundSource('CW', feature3, 100, initial_position3, velocity3, F1, F2, acc3);
% sourceAll = [source1];
sourceAll = [source1, source2, source3];

%%
% 每条报文包括线谱频率、线谱幅度、线谱个数等K项目标特征
M = size(platFormAll, 2);
N = size(sourceAll, 2);
R = M * N;
K = 12; % 目标特征个数

% 将特征转换为矩阵
combined_matrix = zeros(N, K);
features = {feature1; feature2; feature3};
% 遍历每个 feature 单元格，提取数据并拼接
for i = 1:numel(features)
    % 获取当前 feature 单元格的数据
    current_feature = features{i};
    nested_cell = current_feature{2};

    combined_matrix(i, 1) = current_feature{1};
    % 最多存放8个线谱频率
    for j = 1:numel(nested_cell)
        combined_matrix(i, j+1) = nested_cell{j};
    end
    % 存放其他参数，从第10位开始
    for j = 10:K
        combined_matrix(i, j) = current_feature{j - 7};
    end

end
X = combined_matrix;

%% 计算参数的加权平均值──没有用到
Xsum = sum(X); % 按列求和
beta = zeros(1, K);
for j = 1:K
    beta(1, j) = Xsum(1, j) / Xsum(1, 1);
end

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

%% 开始生成观测特征向量
% for i = 1:100
%     res_feature{i, 1} = generate_feature_vector(feature1);
%     x = create_new_feature_vector();
% %     disp(x);
% 
% end

%% 获取每个平台的每个目标信息
% 获取目标方位量测
% 获取目标方位上的功率谱分布函数和线谱频率
% 利用不用观测站量测的线谱特征之间的模糊关系，获取已关联的线谱特征子集和待关联线谱特征的模糊关系矩阵
% 构建模糊关系矩阵，利用模糊关系矩阵进行目标定位批号二维关联

% 确保来自同一个平台的不会关联上
numOfM = 100; % 控制蒙特卡洛的次数
for i = 1:numOfM
    P_Featu = cell(numOfPlatForm, 1);
    for j = 1:numOfPlatForm % 遍历平台
        % 平台观测到的特征向量
        for k = 1:numOfSource % 遍历声源
%             % 虚警
%             if randi([1, 20]) == 1
%                 % 创造出一个不存在的目标特征向量
%                 P_Featu{j}{end + 1} = create_new_feature_vector();
%             % 漏报
%             elseif randi([1, 20]) == 2
%                 continue;  
%             % 正常
%             else
%                 P_Featu{j}{end + 1} = generate_feature_vector(feature1);
%             end
            P_Featu{j}{end + 1} = generate_feature_vector(feature1);
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm
    % 本次观测内
    specific(P_Featu);

    % 组成特征矩阵，
    % 基准特征


    n = 10;








end



% 将feature_matrix_1变为特征矩阵
% 遍历每个 feature 单元格，提取数据并拼接
for i = 1:numel(feature_matrix_1)
    % 获取当前 feature 单元格的数据
    current_feature = feature_matrix_1{i};
    nested_cell = current_feature{2};

    combined_matrix(i, 1) = current_feature{1};
    % 最多存放8个线谱频率
    for j = 1:numel(nested_cell)
        combined_matrix(i, j+1) = nested_cell{j};
    end
    % 存放其他参数，从第10位开始
    for j = 10:K
        combined_matrix(i, j) = current_feature{j - 7};
    end

end
Xrec = combined_matrix;

a = 10;

%% 灰色数据关联
%1. 去量纲化处理
xavg = sum(Xrec) / R; %公式3.1
sjsum = zeros(1, K);
for j = 1:K
    for i = 1:R
        sjsum(1, j) = sjsum(1, j) + (Xrec(i, j) - xavg(1, j))^2;
    end
end
sj = sqrt(sjsum/R);
Xi = zeros(R, K);
for i = 1:R
    for j = 1:K
        Xi(i, j) = (Xrec(i, j) - xavg(1, j)) / sj(1, j);
    end
end

%2. 关联系数与灰色关联度计算
Rab = zeros(R, R); %汇总
Rabrec = zeros(R, R); %每次收到
for i = 1:R
    Rabrec = cacldetla_plie(Xi, R, K, i);
    Rab(i, :) = Rabrec; % 每一行是
end

%% 模糊数学处理

%% 添加一个重新获取特征向量的函数
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
% if randi([1, 20]) == 1
%     % 以10%的概率删除一个线谱频率
%     if numel(generated_line_freqs) > 1
%         idx = randi(numel(generated_line_freqs));
%         generated_line_freqs(idx) = [];
%     end
% elseif randi([1, 20]) == 2
%     % 以10%的概率增加一个线谱频率
%     idx = randi(numel(generated_line_freqs)+1);
%     generated_line_freqs = [generated_line_freqs(1:idx-1), {randi([200, 1000])}, ...
%         generated_line_freqs(idx:end)];
% end

% 转换为数组形式
generated_line_freqs = cell2mat(generated_line_freqs);

% 2. 线谱频率高斯分布
generated_line_freqs = sort(generated_line_freqs + normrnd(0, 3, size(generated_line_freqs)));

num_lines = size(generated_line_freqs, 2);

% 3. 其他异常频率
% 在 1 到 1000 之间生成一个随机异常频率
% anomaly_freq = randi([1, 1000]);
% 4. 轴频、叶片数和谐波个数均匀分布
generated_axis_freq = axis_freq + randi([-1, 1]);
generated_num_blades = num_blades + randi([-1, 1]);
generated_num_harmonics = num_harmonics + randi([-1, 1]);

% 构建特征向量
feature_vector = {num_lines, generated_line_freqs, generated_axis_freq, ...
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

% 对特征向量进行区分的函数
% input: 行：平台数，列 特征向量个数
function specific(input)
% 创建一个相同大小的空cell矩阵，并将每个元素置为0
match_labels = cellfun(@(x) cell(size(x)), input, 'UniformOutput', false);
degree = cellfun(@(x) cell(size(x)), input, 'UniformOutput', false);
% 初始化一个标记号
label = 1;

% 逐个遍历每个元素
for i = 1:size(input, 1)  % 遍历每一行
    for j = 1:size(input{i}, 2)  % 遍历当前行的每个元素
        if isempty(match_labels{i}{j}) % 如果当前元素还没有被标记
            % 计算当前元素与其他行的匹配度
            base_feature = input{i}{j};
            match_labels{i}{j} = label;
            for k = i+1:size(input, 1)  % 从下一行开始遍历
                for l = 1:size(input{k}, 2)  % 遍历下一行的每个元素
                    % 计算匹配度，这里假设你有一个函数match_degree计算匹配度
                    degree = match_degree(base_feature, input{k}{l});
                    if degree > 0.35
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

% 将cell转换为数组
for i = 1 : size(match_labels)
    match_labels{i} = cell2mat(match_labels{i});
end
% 输出标记结果
disp(match_labels);


end
% 计算b对a的隶属度
function res = match_degree(a, b)

% 定义隶属度函数
num_lines_membership = @(x) trapmf(x, [a{1}-2, a{1}, a{1}+1, a{1}+2]);
freq_membership = @(x, center) trapmf(x, [center - 5, center - 1, center + 1, center+ 5]); % 频率隶属度函数为以特征值为中心的高斯分布，方差为3,标准差就是√3

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
        diff = norm(array{1}(i) - x);
        if diff < min_diff
            min_diff = diff;
            nearest_val = array{1}(i);
        end
    end
    res = nearest_val;
end

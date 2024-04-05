%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLMkmeans抗异常\mainKmeans2.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-04-04
% 描述: 对kmeans抗异常参量技术进行性能分析────程序1,计算只有1个异常参量，找到异常参量的正确率
% 输入:  
% 输出:  
%**************************************************************************

%%
clc
clear
close all
tic;
%% 观测数据
T = 0.1; %观测周期
T_all = 120; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
var2d = 0.2^2; % 角度制  角度误差
var2t = 2e-3^2;
%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([2e3, 0]);
platform3 = Platform([2e3, 2e3]);
platform4 = Platform([0, 2e3]);
platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 2e3, 0; 2e3, 2e3; 0, 2e3];
%% 运动模型
%匀速运动矩阵
F1 = [1, 0, T, 0; ...
    0, 1, 0, T; ...
    0, 0, 1, 0; ...
    0, 0, 0, 1];
%加速度矩阵
F2 = [0.5 * T^2, 0; ...
    0, 0.5 * T^2; ...
    T, 0; ...
    0, T];

%% 创建运动的声源对象
initial_position1 = [3e2, 5e2]; % 初始位置目标1
velocity1 = [0, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);
sourceAll = source1;

%% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
realwuT = cell(1, numOfPlatForm); % 存放真实的角度
t_obs = 10:T:T_num * T; % 截取10-50s的数据

%% 观测
% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, T_num+100); % 现在只用来存放方位信息
    realangR{j} = nan(numOfSource, T_num+100);
    sourceAll = source1;
    for k = 1:numOfSource % 遍历声源
        % 创建结构体数组
        numStructs = T_num + 100;
        myStructArray = repmat(struct('angle', nan, 'type', nan, 'fre', nan, 't_delay', nan), numStructs, 1);
        % 填充结构体数组
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle + sqrt(var2d) * randn, 'type', type, 'fre', fre, 't_delay', t_delay + sqrt(var2t) * randn);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                realangR{j}(k, t_Num) = angle;
                realwuT{j}(k, i) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle + sqrt(var2d) * randn, 'type', type, 'fre', fre, 't_delay', t_delay + sqrt(var2t) * randn);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn; % 这个结果是度
                realangR{j}(k, t_Num) = angle; % 这个结果是度
                realwuT{j}(k, i) = angle;
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

% 输入参数，当前测量的角度和时延 每一列是一个目标
numOfSource = size(target_info_matrix, 2);
numOfPlatForm = size(target_info_matrix, 1);
T_num = size(target_info_matrix{1, 1}, 1);
tMatrix = cell(numOfSource, 1);
aMatrix = cell(numOfSource, 1);
for i = 1:numOfSource
    % 针对不同平台 提取有用信息
    tMatrix{i} = zeros(T_num, numOfPlatForm);
    aMatrix{i} = zeros(T_num, numOfPlatForm);
    for j = 1:numOfPlatForm
        % 获取当前结构体数组
        oneTarget = target_info_matrix{j, i};
        % 提取结构体中的信息
        t_delayInfo = [oneTarget.t_delay]; %
        angInfo = [oneTarget.angle];
        % 将提取到的信息存储到矩阵中的相应列
        tMatrix{i}(:, j) = t_delayInfo;
        aMatrix{i}(:, j) = angInfo;
    end
end
%% 针对某点时刻进行解算
i = 1; % 目标序号
count = 0;
for t = 101 : 1100
% 使用 isnan 函数生成逻辑矩阵，其中非NaN元素为true，NaN元素为false
nanLogicalMatrix = ~isnan(tMatrix{i});
res = cell(3, 1);
loc = cell(3, 1);
% 使用 sum 函数按行求和，得到每行非NaN元素的数量
nonNaNCountPerRow = sum(nanLogicalMatrix, 2);
if nonNaNCountPerRow(t) >= 2
    % 获取该行中非NaN元素的索引
    nonNaNIndices = find(~isnan(tMatrix{i}(t, :)));
    % 提取出对应的平台时延和角度
    currentT = tMatrix{i}(t, nonNaNIndices);
    currentA = aMatrix{i}(t, nonNaNIndices);
    choseP = randi([1, numOfPlatForm]); % 随机选取一个测量单元
    
    mu = 1;        % 均值
    sigma = 0.5;     % 方差
    currentT(choseP) = currentT(choseP) + mu + sigma * randn; % 随机对此测量单元的时延添加值
    % choseP = 4;        
    % currentT(choseP) = currentT(choseP) + 1; % 随机对此测量单元的时延添加值

    nodeT = node(nonNaNIndices, :);
    nodeT = [nodeT, zeros(length(nonNaNIndices), 1)];

    % 计算TDOA/AOA
    [res{1}, loc{1, :}] = TA(currentT, currentA, nodeT);

    % 计算TDOA
    if size(node, 1) > 3
        [res{2}, loc{2, :}] = myTDOA(currentT, nodeT);
    else
        [res{2}, loc{2, :}] = TDOA(currentT, nodeT, 4);
    end

    % 计算AOA
    [res{3}, loc{3, :}] = AOA(currentA, nodeT);
end

%% 将结果综合起来
Res = res{1}; % 5
Res = [Res; res{2}]; % 4
Res = [Res; res{3}]; % 6
rowsToRemove = any(isnan(Res), 2) | any(isinf(Res), 2); % 找到包含nan或inf的行号
Res = Res(~rowsToRemove, :); % 去除包含nan或inf的行

Loc = loc{1}; % 5
Loc = [Loc; loc{2}]; % 4
Loc = [Loc; loc{3}]; % 6
Loc = Loc(~rowsToRemove, :);

%% 计算聚集度
rho = 200;
gamma = 0.005;
[respie] = jujidu(Res, Loc, rho, gamma);
% disp("Finish")

%% 找到异常的参量
indicesInRes = find(ismember(Res, respie)); % 使用 find 查找子集元素在父数组中的索引
halfIndex = numel(indicesInRes) / 2;
firstHalfIndices = indicesInRes(1:halfIndex); % 找到保留的行号
locpie = Loc(firstHalfIndices, :);

% 先将这些cell拼接
LLoc = unique(horzcat(Loc{:}));
llocpie = unique(horzcat(locpie{:}));

% 使用 setdiff 找到在 Res 中但不在 respie 中的元素
nonSubsetElementsInRes = setdiff(LLoc, llocpie);
fprintf("异常参量是: %s\n", nonSubsetElementsInRes);
temp = char(nonSubsetElementsInRes);
if ~isempty(temp) && (str2double(temp(2)) == choseP)
    count = count + 1;
end
end
fprintf("正确率为%f\n", count ./ 1000);
toc;
%% 实现TDOA算法 这个程序专用 在内部集成了4平台选3的功能
function [res, loc] = myTDOA(currentT, node)
% 开始4选3
if size(node, 1) > 3
    combinationsT = nchoosek(currentT, 3);
    % 使用nchoosek函数生成所有可能的组合的索引
    combinationIndices = nchoosek(1:size(node, 1), 3);
    for index = 1:size(combinationsT, 1)
        timeDelay = combinationsT(index, :);
        nodeT = node(combinationIndices(index, :), :);
        [res(index, :), ~]= TDOA(timeDelay, nodeT, 4);
        % 自我获取组合
        loc{index, :} = arrayfun(@(x)"t"+num2str(x), combinationIndices(index, :));
    end
end
end
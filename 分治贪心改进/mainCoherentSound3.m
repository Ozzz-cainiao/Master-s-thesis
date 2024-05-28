%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainCoherentSound.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-05-01
% 描述: 验证我分治贪心关联算法的可行性
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
T_all = 400; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.2^2; % 角度制  角度误差
var2t = 0.01^2; % 时延误差
% pd = 0.9; % 检测概率
% 虚警期望

%% 运动模型
% 这是什么模型？
% 运动方程 x = x_last + v * t + 0.5 * a * t^2;
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

%% 布放目标
% initial_position1 = [2e3+dts*dt, 4e3]; % 初始位置目标1
% initial_position1 = [5e3, 4e3]; % 初始位置目标1
initial_position1 = [4e3, 4e3]; % 初始位置目标1

velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);
% initial_position2 = [6e3, 3e3]; % 初始位置目标2
initial_position2 = [7e3, 4e3]; % 初始位置目标2

% initial_position2 = [7e3, 3e3+dts/dt]; % 初始位置目标2
velocity2 = [0, 10]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('CW', 2e3, 100, initial_position2, velocity2, F1, F2, acc2);

sourceAll = [source1, source2];
% sourceAll = [source1, source2, source3, source4, source5];

%% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);
% platform5 = Platform([5e3, 1e4]);
% platFormAll = [platform1, platform2, platform3, platform4];
% platFormAll = [platform1, platform2, platform3, platform4,platform5];
platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4; ];
% node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4; 5e3, 1e4];

%% 观测
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
timeR = cell(1, numOfPlatForm); % 存放时延
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
realwuT = cell(1, numOfPlatForm); % 存放无时延的真实的角度

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    sourceAll = [source1, source2];
    angR{j} = nan(T_num+100, numOfSource); % 现在只用来存放方位信息
    timeR{j} = nan(T_num+100, numOfSource);
    for k = 1:numOfSource % 遍历声源
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn;
                timeR{j}(t_Num, k) = t_delay + sqrt(var2t) * randn;
                %                 realangR{j}(k, t_Num) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn; % 这个结果是度
                timeR{j}(t_Num, k) = t_delay + sqrt(var2t) * randn;
            end
        end % for i = 1: T_num
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
% figure('Units', 'centimeters', 'Position', [10, 10, 12, 11.24 / 15 * 15]);
weight = 14; % 宽 单位厘米
figure('Units', 'centimeters', 'Position', [10, 10, 10, 10 / 4 * 3]); % 左下宽高

hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站','FontSize', 10,'Location', 'northwest')  %'Location', 'eastoutside' ,
title('目标实际运动轨迹', 'FontSize', 10);
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 10)
ylabel('北向坐标/m', 'FontSize', 10)


%%
t_obs = 7:T:T_num * T; % 截取数据
angM = cell(length(t_obs), numOfPlatForm);
% timeM = zeros(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    %{
        在给定时间步长内，sort函数用于将角度数据中的NaN值移动到数组的末尾，
        然后从最小值开始排序非NaN值。
    %}
    %     angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    % 变换 上个语句不是没有很好的实现先排序后取非nan值的功能
    %     angM(iii, :) = arrayfun(@(s) sort(angR{s}(t_obs(1) / T + iii - 1, ~isnan(angR{s}(t_obs(1) / T + iii - 1, :)))), 1:numOfPlatForm, 'un', 0);
    % 去除掉排序
    angM(iii, :) = arrayfun(@(s) angR{s}(t_obs(1) / T + iii - 1, ~isnan(angR{s}(t_obs(1) / T + iii - 1, :))), 1:numOfPlatForm, 'un', 0);
end

% 初始化存放索引的 cell 数组
idxList = cell(size(angM));

% 生成索引列表
for iii = 1:length(t_obs)
    for s = 1:numOfPlatForm
        elements = cell2mat(angM(iii, s));
        A = angR{s}(t_obs(1) / T + iii - 1, :);
        for i = 1:numel(elements)
            % 检查元素是否在数组中
            tf = ismember(A, elements(i));

            if any(tf)
                % 找到元素，获取索引
                index = find(tf, 1, 'first'); % 只获取第一个匹配的索引
                idxList{iii, s}(end +1) = index;
            else
                %                 indices(i) = NaN; % 如果元素不在数组中，索引设置为NaN
            end
        end
    end
end

% 初始化存放数据的矩阵
timeM = cell(size(angM));

% 提取数据
for iii = 1:length(t_obs)
    for s = 1:numOfPlatForm
        % 获取当前位置的索引列表
        idx = idxList{iii, s};
        % 根据索引列表提取数据
        timeM{iii, s} = timeR{s}(t_obs(1) / T + iii - 1, idx);
    end
end

%% 分治贪心关联
% 传入参数 角度 平台数 平台位置 目标数量
% [outTimeM, choose] = calcR1(sourceAll, timeM, angM, numOfPlatForm, node, numOfSource, t_obs, T);
[outTimeM, choose] = calcR4(sourceAll, timeM, angM, numOfPlatForm, node, numOfSource, t_obs, T); % 这个是加上了yita=1作为门限，可用

outTimeM = outTimeM';
toc;
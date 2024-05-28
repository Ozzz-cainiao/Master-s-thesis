%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainCoherentSound.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-05-01
% 描述: 分治贪心改进算法仿真2
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
T_all = 200; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.2^2; % 角度制  角度误差
var2t = 0.01^2; % 时延误差
pd = 1; % 检测概率

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
initial_position1 = [4e3, 7e3]; % 初始位置目标1

% initial_position1 = [4e3, 7e3]; % 初始位置目标1
velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [10, 0]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [5e3, 2e3]; % 初始位置目标2
velocity3 = [0, 10]; % 运动速度
acc3 = 0; % 加速度
source3 = SoundSource('CW', [2e3], [100], initial_position3, velocity3, F1, F2, acc3);
initial_position4 = [3e3, 8e3]; % 初始位置目标2
% initial_position4 = [4e3, 1.5e3]; % 初始位置目标2

velocity4 = [0, 10]; % 运动速度
acc4 = 0; % 加速度
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);
initial_position5 = [8e3, 5e3]; % 初始位置目标2
velocity5 = [0, -10]; % 运动速度
acc5 = 0; % 加速度
source5 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position5, velocity5, F1, F2, acc5);

% sourceAll = [source1, source2];
sourceAll = [source1, source2, source3, source4, source5];

%% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);
% platform5 = Platform([5e3, 1e4]);
% platFormAll = [platform1, platform2, platform3, platform4];
% platFormAll = [platform1, platform2, platform3, platform4,platform5];
platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4;];
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
lamda = 2; % 考虑虚警
% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    sourceAll = [source1, source2, source3, source4, source5];
    angR{j} = nan(T_num+100, numOfSource); % 现在只用来存放方位信息
    timeR{j} = nan(T_num+100, numOfSource);
    for k = 1:numOfSource % 遍历声源
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(i, k) = angle + sqrt(var2d) * randn;
                timeR{j}(i, k) = t_delay + sqrt(var2t) * randn;
                %                 realangR{j}(k, t_Num) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(i, k) = angle + sqrt(var2d) * randn; % 这个结果是度
                timeR{j}(i, k) = t_delay + sqrt(var2t) * randn;
            end
        end % for i = 1: T_num
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
% figure('Units', 'centimeters', 'Position', [10, 10, 12, 11.24 / 15 * 15]);
weight = 14; % 宽 单位厘米
figure('Units', 'centimeters', 'Position', [10, 10, 12, 12 / 4 * 3]); % 左下宽高

hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.','DisplayName', ['目标', num2str(i)]);
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100,'DisplayName', '观测站');
legend %'Location', 'eastoutside' ,
title('目标实际运动轨迹', 'FontSize', 10);
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 10)
ylabel('北向坐标/m', 'FontSize', 10)

%%
% t_obs = 5; % 截取数据

t_obs = 5:T:T_num * T; % 截取数据
angM = cell(length(t_obs), numOfPlatForm);
% timeM = zeros(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
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

% %% 探测节点以及目标位置布置场景
% birthPlace = zeros(numOfSource, 2);
% for i = 1:numOfSource
%     birthPlace(i, 1) = sourceAll(i).Position(1, 1);
%     birthPlace(i, 2) = sourceAll(i).Position(1, 2);
% end
% Ylim    = [0,10e3];
% xgrid = 0:10:10e3;
% y = cell(numOfPlatForm, 1);
% % fig = figure('Units', 'centimeters', 'Position', [20, 5, 20, 11.24 / 15 * 15]);
% fig = figure;
% for s = 1:numOfPlatForm
%     theta = angR{s}(55, :)';
%     xp = node(s, 1);
%     yp = node(s, 2);
%     y{s} = cell2mat(arrayfun(@(x) (repmat(x-xp, length(theta), 1)).*tand(theta)+repmat(yp, length(theta), 1), xgrid, 'un', 0)); % 在反向线上的点舍去
%     locsp2 = arrayfun(@(x) y{s}(x, :) <= Ylim(1) | y{s}(x, :) >= Ylim(2), 1:length(theta), 'un', 0); % 超过视距的点
%     y{s}(cell2mat(arrayfun(@(v) v, cell2mat(locsp2'), 'un', 0))) = nan; % 超过视距的点舍去
%     figure(fig)
%     hold on
%     h = arrayfun(@(x) plot(xgrid, y{s}(x, :), '--', 'Color', '#808080'), 1:length(theta));
%     hold off
% end
% figure(fig)
% hold on
% s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% s2 = scatter(birthPlace(:, 1), birthPlace(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100);
% legend([h(end), s1, s2], '方位测量', '观测站', '目标', 'FontSize', 12)
% hold off
% set(gca, 'Box', 'on')
% xlabel('东向坐标/m', 'FontSize', 12)
% ylabel('北向坐标/m', 'FontSize', 12)

%% 分治贪心关联
% 传入参数 角度 平台数 平台位置 目标数量
[outTimeM, choose] = calcR2(sourceAll, timeM, angM, numOfPlatForm, node, numOfSource, t_obs, T);
% [outTimeM, choose] = calcR4(sourceAll, timeM, angM, numOfPlatForm, node, numOfSource, t_obs, T);

outTimeM = outTimeM';
toc;
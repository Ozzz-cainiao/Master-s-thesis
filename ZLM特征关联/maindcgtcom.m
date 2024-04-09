%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\maindcgtcom.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-04-08
% 描述: 用于和特征关联比较性能的纯方位算法
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
T_all = 0.1; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.5^2; % 角度制  角度误差
pd = 0.9; % 检测概率
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

%% 低杂波双目标四平台
% 布放目标
initial_position1 = [7e3, 8e3]; % 初始位置目标1
velocity1 = [0, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
feature1 = {5, {460, 580, 650, 790, 880}, 3.9, 6, 6}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source1 = SoundSource('CW', 1, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 8.1e3]; % 初始位置目标2
velocity2 = [0, 0]; % 运动速度
acc2 = 0; % 加速度
feature2 = {8, {225, 380, 420, 460, 550, 620, 710, 820}, 4.7, 3, 5}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source2 = SoundSource('CW', 1, 100, initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [7e3, 7.9e3]; % 初始位置目标2
velocity3 = [0, 0]; % 运动速度
acc3 = 0; % 加速度
feature3 = {4, {320, 455, 560, 750}, 6.5, 7, 8}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source3 = SoundSource('CW', 1, 100, initial_position3, velocity3, F1, F2, acc3);

initial_position4 = [6e3, 5e3]; % 初始位置目标2
velocity4 = [0, 0]; % 运动速度
acc4 = 0; % 加速度
feature4 = {6, {260, 330, 440, 550}, 5.2, 7, 6}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source4 = SoundSource('CW', 1,100, initial_position4, velocity4, F1, F2, acc4);

initial_position5 = [2e3, 1e3]; % 初始位置目标2
velocity5 = [0, 0]; % 运动速度
acc5 = 0; % 加速度
feature5 = {7, {110, 300, 400, 500,600,700,800}, 6, 6, 7}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source5 = SoundSource('CW', 1, 100, initial_position5, velocity5, F1, F2, acc5);


initial_position6 = [8e3, 3e3]; % 初始位置目标2
velocity6 = [0, 0]; % 运动速度
acc6 = 0; % 加速度
feature6 = {7, {110, 300, 400, 500,600,700,800}, 6, 6, 7}; % 线谱数量，线谱频率, 轴频，桨叶数、谐波个数
source6 = SoundSource('CW', 1, 100, initial_position6, velocity6, F1, F2, acc6);
% sourceAll = [source1, source2];
sourceAll = [source1, source2, source3, source4, source5, source6];

% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];
% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
realwuT = cell(1, numOfPlatForm); % 存放真实的角度

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
%     sourceAll = [source1, source2];
sourceAll = [source1, source2, source3, source4, source5, source6];

    angR{j} = nan(numOfSource, T_num+100); % 现在只用来存放方位信息
    realangR{j} = nan(numOfSource, T_num+100);
    for k = 1:numOfSource % 遍历声源
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(k, i) = angle + sqrt(var2d) * randn;
%                 angR{j}(k, i) = angle;


            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                angR{j}(k, i) = angle + sqrt(var2d) * randn; % 这个结果是度
            end
        end % for i = 1: T_num
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
figure
hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend( '目标1', '目标2','观测站', 'FontSize', 12)
title('目标实际运动轨迹');
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
%%
t_obs = T:T:T_num * T; % 截取10-50s的数据
angM = cell(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
end

%% 探测节点以及目标位置布置场景
birthPlace = zeros(numOfSource, 2);
for i = 1:numOfSource
    birthPlace(i, 1) = sourceAll(i).Position(1, 1);
    birthPlace(i, 2) = sourceAll(i).Position(1, 2);
end
%% 分治贪心关联
% 调用709函数 传入参数 角度 平台数 平台位置 目标数量
[outLoctionCAX, outLoctionCAY, outLoctionSPCX, outLoctionSPCY] = calcAll(angM, numOfPlatForm, node, numOfSource, t_obs, T);

% 计算定位结果的平均值
resX = nanmean(outLoctionSPCX, 2);
resY = nanmean(outLoctionSPCY, 2);

figure('Units', 'centimeters', 'Position', [20, 5, 16, 9]);
for s = 1:size(node, 1)
    theta = angR{s}(:, 1);
    xp = node(s, 1);
    yp = node(s, 2);
    % 计算射线的终点
    end_x = xp + 12e3 * cosd(theta);
    end_y = yp + 12e3 * sind(theta);
    % 绘制射线
    hold on;
    h= arrayfun(@(i) plot([xp, end_x(i)], [yp, end_y(i)], '--', 'Color', '#808080'), 1:length(theta));
    hold off;
end
% 限制坐标范围
xlim([-1e3, 11e3]);
ylim([-1e3, 11e3]);
hold on
s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
s2 = scatter(birthPlace(:, 1), birthPlace(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100);
s3 = scatter(resX, resY,'bs','LineWidth', 1, 'SizeData', 100);
legend([h(end), s1, s2, s3], '方位测量', '观测站', '目标', '目标定位结果', 'FontSize', 12, 'Location', 'eastoutside')
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
disp("Finish");
toc;

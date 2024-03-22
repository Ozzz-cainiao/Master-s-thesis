%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTestTDOAAOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-22
% 描述: 测试TA1函数的问题，查看为什么定位精度较低, 没有误差的话，定位结果没有问题
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.1; %观测周期
T_all = 5; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.0^2; % 角度制  角度误差
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

%% 布放目标
initial_position1 = [4e3, 7e3]; % 初始位置目标1
velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [0, 10]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('CW', 2e3, 100, initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [2e3, 5e3]; % 初始位置目标2
velocity3 = [0, 10]; % 运动速度
acc3 = 0; % 加速度
source3 = SoundSource('CW', 2e3, 100, initial_position3, velocity3, F1, F2, acc3);
% sourceAll = [source1];

sourceAll = [source1];

% sourceAll = [source1, source2, source3];
% sourceAll = [source1, source2, source3, source4, source5];

%% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];

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
    %     realangR{j} = nan(numOfSource, T_num+100);
    timeR{j} = nan(T_num+100, numOfSource);
    for k = 1:numOfSource % 遍历声源
        % 创建结构体数组
        numStructs = T_num + 100;
        myStructArray = repmat(struct('angle', nan, 'type', nan, 'fre', nan, 't_delay', nan), numStructs, 1);
        % 填充结构体数组
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                %                 angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                angR{j}(i, k) = angle + sqrt(var2d) * randn;
                timeR{j}(i, k) = t_delay;
                %                 realangR{j}(k, t_Num) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(i, k) = angle + sqrt(var2d) * randn; % 这个结果是度
                timeR{j}(i, k) = t_delay;
                %                 realangR{j}(k, t_Num) = angle; % 这个结果是度
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
figure
hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '观测站', 'FontSize', 12)
title('目标实际运动轨迹');
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)

%% 测试TA函数
t_obs = T:T:T_num * T; % 截取数据
angM = cell(length(t_obs), numOfPlatForm);
timeM = cell(length(t_obs), numOfPlatForm);
% timeM = zeros(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) angR{s}(t_obs(1) / T + iii - 1, ~isnan(angR{s}(t_obs(1) / T + iii - 1, :))), 1:numOfPlatForm, 'un', 0);
    timeM(iii, :) = arrayfun(@(s) timeR{s}(t_obs(1) / T + iii - 1, ~isnan(timeR{s}(t_obs(1) / T + iii - 1, :))), 1:numOfPlatForm, 'un', 0);
end

i = 1;
for iii = 1:length(t_obs)
    c_time = cell2mat(timeM(iii, :));
    c_angle = cell2mat(angM(iii, :));
    [res, ~] = TA1(c_time, c_angle, node);
    TA_res{iii, i} = res;
    TA_resX(iii, i) = res(1);
    TA_resY(iii, i) = res(2);
end

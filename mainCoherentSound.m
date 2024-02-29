%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainCoherentSound.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-01-18
% 描述: 在这个程序中实现相干声源的关联与区分
% 输入:  
% 输出:  
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.1; %观测周期
T_all = 10; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 1^2; % 角度制  角度误差
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
velocity2 = [10, 10]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('CW', 2e3, 100, initial_position2, velocity2, F1, F2, acc2);

sourceAll = [source1, source2];
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
    angR{j} = nan(numOfSource, T_num+100); % 现在只用来存放方位信息
%     realangR{j} = nan(numOfSource, T_num+100);
    timeR{j} = nan(numOfSource, T_num+100);
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
                angR{j}(k, t_Num) = angle + 0;
                timeR{j}(k, t_Num) = t_delay;
%                 realangR{j}(k, t_Num) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + 0; % 这个结果是度
                timeR{j}(k, t_Num) = t_delay;
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
legend( '目标1', '目标2','观测站', 'FontSize', 12)
title('目标实际运动轨迹');
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
%%
t_obs = 6:T:T_num * T; % 截取数据
angM = cell(length(t_obs), numOfPlatForm);
% timeM = zeros(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    %{
        在给定时间步长内，sort函数用于将角度数据中的NaN值移动到数组的末尾，
        然后从最小值开始排序非NaN值。
    %}

    angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);

end

% 初始化存放索引的 cell 数组
idxList = cell(size(angM));

% 生成索引列表
for iii = 1:length(t_obs)
    for s = 1:numOfPlatForm
        % 获取当前平台的角度数据索引
        idxList{iii, s} = find(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))));
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
        timeM{iii, s} = timeR{s}(idx, t_obs(1) / T + iii - 1);
    end
end

%% 分治贪心关联
% 传入参数 角度 平台数 平台位置 目标数量
[outTimeM, choose] = calcR(timeM, angM, numOfPlatForm, node, numOfSource, t_obs, T);
outTimeM = outTimeM';

%% 测试TDOA函数



%% 调用TDOA函数
res = cell(size(outTimeM));
for row = 1 : size(outTimeM, 1)
    % 根据这个choose选中的平台的线序号来定位
    for col = 1 : size(outTimeM, 2)
        time = outTimeM{row, col}; % 时间
        pos = node(find(~isnan(time)), :);
        pos = [pos, zeros(size(pos, 1), 1)];
        time = time(find(~isnan(time)));
        % 调用TDOA
        if isempty(time)
            continue;
        else
            [res{row, col}, ~]= TDOA(time, pos, 4);
        end

    end
end



n = 10
%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTest.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-22
% 描述: 作为面向对象编程的主程序，在这个程序中实现对各个类的实例化
%       实现抗异常参量的程序验证
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.1; %观测周期
T_all = 50; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 0.2^2; % 角度制  角度误差
var2t = 2e-4^2;
%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([2e3, 0]);
platform3 = Platform([2e3, 2e3]);
platform4 = Platform([0, 2e3]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 2e3, 0; 2e3, 2e3; 0, 2e3];
% platFormAll = [platform1, platform2, platform3];
% node = [0, 0; 2e3, 0; 2e3, 2e3;];
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

%% 创建运动的声源对象
initial_position1 = [3e2, 3e2]; % 初始位置目标1
velocity1 = [0, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
% source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);

% sourceAll = [source1, source2, source3, source4, source5];
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

%% 画出目标运动的实际轨迹
figure
hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('target1', '观测平台');
title('目标实际运动轨迹');
% for i = 1: [2, 5]
%     plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% legend('target2', 'target5', 'Location', 'eastoutside');
% title('目标实际运动轨迹');

%% 假设对目标进行了区分

% % 初始化一个新的cell数组，用于存储提取的结果
% lfmCellArray = cell(1, 4);
% reallfmCellArray = cell(1, 4);
% fajksd = cell(1, 4);
% % 循环遍历每个cell
% for i = 1:numel(angR)
%     % 提取指定行元素，然后存储到新的cell数组中
%     lfmCellArray{i} = [angR{i}(2, :); angR{i}(5, :)];
%     reallfmCellArray{i} = [realangR{i}(2, :); realangR{i}(5, :)];
%     fajksd{i} = [realwuT{i}(2, :); realwuT{i}(5, :)];
% end
% 
% angM = cell(length(t_obs), numOfPlatForm);
% realangM = cell(length(t_obs), numOfPlatForm);
% fasdhjkf = cell(length(t_obs), numOfPlatForm);
% for iii = 1:length(t_obs)
%     angM(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan(sort(lfmCellArray{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
%     %     angM(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan(sort(lfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0); % 把这个观测平台全为nan的值删掉
%     %     angMb(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan((lfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0); % 把这个观测平台全为nan的值删掉
%     %     realangM(iii, :) = arrayfun(@(s) reallfmCellArray{s}(~isnan(sort(reallfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0);
%     realangM(iii, :) = arrayfun(@(s) reallfmCellArray{s}(~isnan(sort(reallfmCellArray{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
%     fasdhjkf(iii, :) = arrayfun(@(s) fajksd{s}(~isnan(sort(fajksd{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
% end

%% 先把正确的解算结果计算一下 但是也要加时空关联
% AOATime(realangM, node, 2, t_obs, T);

%% 加误差的结果
% out = AOATime(angM, node, 2, t_obs, T);
%% 使用TDOA和TDOA/AOA定位
TALo(target_info_matrix, node);
%%
disp("程序执行结束！")
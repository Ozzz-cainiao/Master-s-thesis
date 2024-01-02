%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTest.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-22
% 描述: 作为面向对象编程的主程序，在这个程序中实现对各个类的实例化
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
var2d = 0^2; % 角度制  角度误差

%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];
% node = [0, 0; 1e4, 0; 1e4, 1e4;];



%% 创建运动的声源对象
initial_position1 = [3e3, 3e3]; % 初始位置目标1
velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
% source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [4e3, 5e3]; % 初始位置目标2
velocity2 = [0, -10]; % 运动速度
acc2 = 0; % 加速度
% source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, acc2);
source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [5e3, 5e3]; % 初始位置目标2
velocity3 = [0, 10]; % 运动速度
acc3 = 0; % 加速度
% source3 = SoundSource('CW', [1e3, 2e3], [100, 50], initial_position3, velocity3, acc3);
source3 = SoundSource('CW', [2e3], [100], initial_position3, velocity3, F1, F2, acc3);

initial_position4 = [2e3, 7e3]; % 初始位置目标2
velocity4 = [10, 10]; % 运动速度
acc4 = 0; % 加速度
% source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, acc4);
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);

initial_position5 = [8e3, 8e3]; % 初始位置目标2
velocity5 = [-10, 0]; % 运动速度
acc5 = 0; % 加速度
source5 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position5, velocity5, F1, F2, acc5);

sourceAll = [source1, source2, source3, source4, source5];

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
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                realangR{j}(k, t_Num) = angle;
                realwuT{j}(k, i) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn; % 这个结果是度
                realangR{j}(k, t_Num) = angle; % 这个结果是度
                realwuT{j}(k, i) = angle;
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源

end % for j = 1:numOfPlatForm

%% 先拿出来两个目标
% 获取第2列和第5列的所有元素
twotarget_info_matrix = cell(size(target_info_matrix, 1), 2);

% 将第2列的元素赋值给新的cell矩阵的第1列
twotarget_info_matrix(:, 1) = target_info_matrix(:, 2);

% 将第5列的元素赋值给新的cell矩阵的第2列
twotarget_info_matrix(:, 2) = target_info_matrix(:, 5);


%% 画出目标运动的实际轨迹
figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);
hold on
% for i = 1:numOfSource
%     plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% legend('target1','target2','target3','target4','target5');
% title('目标实际运动轨迹');
for i = [2, 5]
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('target2', 'target5', 'Location', 'eastoutside');
title('目标实际运动轨迹');

%% 假设对目标进行了区分

% 初始化一个新的cell数组，用于存储提取的结果
lfmCellArray = cell(1, 4);
reallfmCellArray = cell(1, 4);
fajksd = cell(1, 4);
% 循环遍历每个cell
for i = 1:numel(angR)
    % 提取指定行元素，然后存储到新的cell数组中
    lfmCellArray{i} = [angR{i}(2, :); angR{i}(5, :)];
    reallfmCellArray{i} = [realangR{i}(2, :); realangR{i}(5, :)];
    fajksd{i} = [realwuT{i}(2, :); realwuT{i}(5, :)];
end

angM = cell(length(t_obs), numOfPlatForm);
realangM = cell(length(t_obs), numOfPlatForm);
fasdhjkf = cell(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan(sort(lfmCellArray{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    %     angM(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan(sort(lfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0); % 把这个观测平台全为nan的值删掉
    %     angMb(iii, :) = arrayfun(@(s) lfmCellArray{s}(~isnan((lfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0); % 把这个观测平台全为nan的值删掉
    %     realangM(iii, :) = arrayfun(@(s) reallfmCellArray{s}(~isnan(sort(reallfmCellArray{s}(:, iii))), iii), 1:numOfPlatForm, 'un', 0);
    realangM(iii, :) = arrayfun(@(s) reallfmCellArray{s}(~isnan(sort(reallfmCellArray{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    fasdhjkf(iii, :) = arrayfun(@(s) fajksd{s}(~isnan(sort(fajksd{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
end

%% 先把正确的解算结果计算一下 但是也要加时空关联
% AOATime(realangM, node, 2, t_obs, T);

%% 加误差的结果
% out = AOATime(angM, node, 2, t_obs, T);
%% 使用TDOA和TDOA/AOA定位
% TALo(twotarget_info_matrix, node);
%% 分治贪心关联
% 调用709函数 传入参数 角度 平台数 平台位置 目标数量
% 传出 2*5000
[outLoctionCAX, outLoctionCAY, outLoctionSPCX, outLoctionSPCY] = calcAll(angM, numOfPlatForm, node, numOfSource, t_obs, T);

%% ==========================绘图区===============================

% % 迭代绘制轨迹
% for i = 2:length(t_obs)
%     % 更新绘图数据
%     set(h, 'XData', x(1:i), 'YData', y(1:i));
% 
%     % 刷新图形
%     drawnow;
% 
%     % 添加延时，以调整动画速度
%     pause(0.1);
% end



%%
disp("程序执行结束！")

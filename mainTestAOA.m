%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTestAOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-23
% 描述: 在这个程序中实现粗关联和时空关联算法
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
var2d = 1.5^2; % 角度制  角度误差

%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];
% node = [0, 0; 1e4, 0; 1e4, 1e4;];

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
                %                 target_info_matrix{j, k}(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay); %
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                realangR{j}(k, t_Num) = angle;
                realwuT{j}(k, i) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                %                 target_info_matrix{j, k, t_Num} = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
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
figure
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
legend('target2', 'target5');
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
    realangM(iii, :) = arrayfun(@(s) reallfmCellArray{s}(~isnan(sort(reallfmCellArray{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    fasdhjkf(iii, :) = arrayfun(@(s) fajksd{s}(~isnan(sort(fajksd{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
end

%% LSM
% for k = 1:2
%     res{k} = nan(size(fasdhjkf, 1), 2); % 时间*1的结果
%     for i = 1:size(fasdhjkf, 1)
%         Zt = nan(1, size(fasdhjkf, 2));
%         for j = 1:size(fasdhjkf, 2)
%             Zt(j) = fasdhjkf{i, j}(k);
%         end
%         [res{k}(i, 1), res{k}(i, 2)] = LSM(Zt, node);
%     end
% end
% figure
% hold on
% for i = 1:2
%     plot(res{i}(:, 1), res{i}(:, 2), '.');
% %     plot(Tres{i}(:, 1), Tres{i}(:,2),'*');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% hold off
% legend('目标1LSM','目标2LSM')
% title("LSM定位结果")

%% 先把正确的解算结果计算一下 但是也要加时空关联
% AOATime(realangM, node, 2, t_obs, T);

%% 加误差的结果
% out = AOATime(angM, node, 2, t_obs, T);
%% 使用TDOA/AOA定位
TALo(twotarget_info_matrix, node);
%% 分治贪心关联
% 调用709函数 传入参数 角度 平台数 平台位置 目标数量
% 传出 2*5000
[outLoctionCAX, outLoctionCAY, outLoctionSPCX, outLoctionSPCY] = calcAll(angM, numOfPlatForm, node, numOfSource, t_obs, T);


%%
disp("程序执行结束！")

%% 两两组合解算目标点
function [EstX, EstY] = AOA1(Zt, node)
% 添加一下判角的条件
% 找到非NaN值的索引
[~, col] = find(~isnan(Zt));
% 使用索引从第二个矩阵中取出相应位置的数
node = node(col, :);
Zt = Zt(:, col);
len = size(node, 1);
k = 0;
for i = 1:len
    for j = i + 1:len
        k = k + 1;
        pos1 = node(i, :);
        pos2 = node(j, :);
        alpha1 = Zt(i);
        alpha2 = Zt(j);
        res(k, :) = AngleCross(pos1, pos2, alpha1, alpha2);
    end
end
EstX = mean(res(:, 1));
EstY = mean(res(:, 2));
end

%% 这个输入的角度是度
function res = AngleCross(pos1, pos2, angle1, angle2)
pos_x1 = pos1(1);
pos_y1 = pos1(2);
pos_x2 = pos2(1);
pos_y2 = pos2(2); %GPS测得两个平台位置
% 将角度转换为弧度
angle1 = angle1 / 180 * pi;
angle2 = angle2 / 180 * pi;
L = sqrt((pos_x1 - pos_x2)^2+(pos_y1 - pos_y2)^2); %基线长度

beta = (atan((pos_y2 - pos_y1)/(pos_x2 - pos_x1))); %基线与正东方向夹角 以pi为单位

R1 = L * abs(cos(angle2+beta)/sin(angle2-angle1));
R2 = L * abs(cos(angle1+beta)/sin(angle2-angle1)); %计算得到的平台到目标的距离

xx1 = pos_x1 + R1 * sin(angle1);
xx2 = pos_x2 + R2 * sin(angle2);
yy1 = pos_y1 + R1 * cos(angle1);
yy2 = pos_y2 + R2 * cos(angle2);

res(1) = (xx1 + xx2) / 2;
res(2) = (yy1 + yy2) / 2;
end



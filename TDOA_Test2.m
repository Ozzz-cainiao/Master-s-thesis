%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA_Test2.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-02-29
% 描述: 自己编写一个TDOA测试函数，测试自己TDOA函数的性能
% 输入:  
% 输出:  
%**************************************************************************

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

sourceAll = [source1];
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
realwuT = cell(1, numOfPlatForm); % 存放无时延的真实的角度

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, T_num+100); % 现在只用来存放方位信息
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
                timeR{j}(k, i) = t_delay;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + 0; % 这个结果是度
                timeR{j}(k, i) = t_delay;
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

%% 开始计算TDOA
pos = [node, zeros(size(node, 1), 1)];
res = cell(T_num, numOfSource);
timeM = zeros(T_num, numOfPlatForm);
for iii = 1:T_num
    for jjj = 1 : numOfPlatForm
        timeM(iii, jjj) = timeR{jjj}(1, iii);
        
    end
    [res{iii, 1}, ~]= TDOA(timeM(iii,:), pos, 4);
end




%%
% 目标位置（单位：米）
target_position = [50, 50];

% 信号传播速度（假设单位为 m/s）
v = 1500; % 光速

% 接收器的位置（假设单位为米）
receiver_positions = [
    0, 0; % 接收器1的位置，假设在原点
    1000, 0; % 接收器2的位置，假设在 x 轴上距离原点100米
    0, 1000; % 接收器3的位置，假设在 y 轴上距离原点100米
    1000, 1000 % 接收器4的位置，假设在 x 和 y 轴上距离原点各100米
];

% 计算目标到各接收器的距离
d1 = norm(target_position - receiver_positions(1,:)); % 接收器1
d2 = norm(target_position - receiver_positions(2,:)); % 接收器2
d3 = norm(target_position - receiver_positions(3,:)); % 接收器3
d4 = norm(target_position - receiver_positions(4,:)); % 接收器4

% 计算信号传播时间
t1 = d1 / v;
t2 = d2 / v;
t3 = d3 / v;
t4 = d4 / v;

disp(['目标位置：(', num2str(target_position(1)), ', ', num2str(target_position(2)), ')']);
disp(['接收器1接收时间：', num2str(t1), '秒']);
disp(['接收器2接收时间：', num2str(t2), '秒']);
disp(['接收器3接收时间：', num2str(t3), '秒']);
disp(['接收器4接收时间：', num2str(t4), '秒']);
% 信号传播时间
dt1 = t1 - 0;
dt2 = t2 - 0;
dt3 = t3 - 0;
dt4 = t4 - 0;

% 计算目标位置
A = 2 * (receiver_positions(2,:) - receiver_positions(1,:));
B = 2 * (receiver_positions(3,:) - receiver_positions(1,:));
C = 2 * (receiver_positions(4,:) - receiver_positions(1,:));

D1 = v * dt1;
D2 = v * dt2;
D3 = v * dt3;
D4 = v * dt4;

target_position = (A(1:2)*D1 + B(1:2)*D2 + C(1:2)*D3 + D4*(A(2)*B(1) - A(1)*B(2))) / (A(1)*B(2) - A(2)*B(1));

disp(['计算得到的目标位置：(', num2str(target_position(1)), ', ', num2str(target_position(2)), ')']);
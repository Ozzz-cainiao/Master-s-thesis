%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\FOA\FOAPlus.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-19
% 描述: SZY 2.3.4仿真
% 输入:
% 输出:
%**************************************************************************
clc
clear
close all

%% 观测数据
T = 0.5; % 观测周期
dt = T;
T_all = 200; % 观测总时间
T_num = T_all / T;
t = 0:T:T_num * T;
var2d = 1.5^2; % 角度制  角度误差
var2f = 0.2; % 观测的频率误差

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

%% 创建观测平台对象
platform1 = Platform([0, 0]);
platform2 = Platform([0, 1e3]);
platform3 = Platform([1e3, 0]);
platform4 = Platform([1e3, 1e3]);

%% 创建运动的声源对象
initial_position1 = [-1200, 800]; % 初始位置目标1
velocity1 = [8, -6]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
% source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
source1 = SoundSource('CW', 1e3, 100, initial_position1, velocity1, F1, F2, acc1);

%% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
platFormAll = [platform1, platform2, platform3, platform4];
sourceAll = source1;
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource, T_all+1); % cell矩阵
angR = cell(numOfPlatForm, 1);
Fre = cell(numOfPlatForm, 1);

%% 观测
for i = 1:T_num
    % 获取每个平台的每个目标信息
    for j = 1:numOfPlatForm % 遍历平台
        for k = 1:numOfSource % 遍历声源
            if i == 1
                %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);
                angR{j} = nan(T_all, numOfSource); % 现在只用来存放方位信息
                [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn;
                % 观测目标是频率
                Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延

            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn; % 这个结果是度
                % 观测目标是频率
                Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm % 遍历平台
end % for i = 1:T_num

%% 1. 修改计算多普勒频率的公式 修改platform
%% 2. 计算目标到各个观测平台的致近点
%% 3。 计算目标定位的运动方程

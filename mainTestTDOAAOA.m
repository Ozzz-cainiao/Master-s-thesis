%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTest.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-26
% 描述: 实现基于方位-时延差信息联合的数据关联
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.5; %观测周期
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


initial_position2 = [7e3, 5e3]; % 初始位置目标2
velocity2 = [-15, 15]; % 运动速度
acc2 = 0; % 加速度
% source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, acc2);
source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [5e3, 5e3]; % 初始位置目标2
velocity3 = [10, 0]; % 运动速度
acc3 = 0; % 加速度
% source3 = SoundSource('CW', [1e3, 2e3], [100, 50], initial_position3, velocity3, acc3);
source3 = SoundSource('CW', [2e3], [100], initial_position3, velocity3, F1, F2, acc3);

initial_position4 = [2e3, 7e3]; % 初始位置目标2
velocity4 = [0, 10]; % 运动速度
acc4 = 0; % 加速度
% source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, acc4);
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);

initial_position5 = [3e3, 5e3]; % 初始位置目标2
velocity5 = [-10, 10]; % 运动速度
acc5 = 0; % 加速度
source5 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position5, velocity5, F1, F2, acc5);

sourceAll = [source1, source2, source3, source4, source5];

%% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource, T_all+1); % cell矩阵
angR = cell(1, numOfPlatForm);
t_obs = 0:T:T_num * T + 20;

%% 观测

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, length(t_obs)); % 现在只用来存放方位信息
    for i = 1:T_num
        for k = 1:numOfSource % 遍历声源
            if i == 1
                %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);

                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn; % 这个结果是度
                % 如果是TDOA定位的话会获得对方的时延
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm % 遍历平台


    % 在这里拆分目标


    % 在这里实时定位
    % 时空关联

    % 这个10是随机设置的


end % for i = 1:T_num






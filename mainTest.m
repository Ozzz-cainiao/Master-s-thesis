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
T = 1; %观测周期
T_all = 50; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
var2d   = 1.5^2 ;

%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];

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
source3 = SoundSource('CW', [1e3, 2e3], [100, 50], initial_position3, velocity3, F1, F2, acc3);

initial_position4 = [2e3, 7e3]; % 初始位置目标2
velocity4 = [0, 10]; % 运动速度
acc4 = 0; % 加速度
% source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, acc4);
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);

sourceAll = [source1, source2, source3, source4];

%%

% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource, T_all+1); % cell矩阵
angR = cell(numOfPlatForm, 1);

%% 观测
for i = 1:T_num
    % 获取每个平台的每个目标信息
    for j = 1:numOfPlatForm % 遍历平台
        for k = 1:numOfSource % 遍历声源
            if i == 1
                %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);
                angR{j} = nan(T_all, numOfSource); % 现在只用来存放方位信息
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k));
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d)* randn;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k));
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d)* randn;
            end
        end
    end
end
% % 下方是初始的信息获取
% for i = 1:T_num
%     % 获取每个平台的每个目标信息
%     for j = 1:numOfPlatForm % 遍历平台
%         for k = 1:numOfSource % 遍历声源
%             if i == 1
%                 %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);
%                 angR{j} = nan(T_all, numOfSource); % 现在只用来存放方位信息
% 
%                 [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k));
%                 t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
%                 target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
%                 %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
%                 angR{j}(t_Num, k) = angle;
%             else
%                 sourceAll(k) = sourceAll(k).updatePosition(dt);
%                 [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k));
%                 t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
%                 target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
%                 %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
%                 angR{j}(t_Num, k) = angle;
%             end
%         end
%     end
% end
%% 
a = 10
% 显示目标信息矩阵
% for i = 1 : 10
%     i
%     disp(target_info_matrix{1, i, 1});
% end
% for i = 1 : 10
%     i
%     disp(angR{1}(i, 1));
% end
% for i = 1 : 4
%     i
%     disp(target_info_matrix{1, 9, i});
% end

%% 将传感器数据格式化为数组
% % 定义四个结构体
% sensorData(1).angle = 43.421747348254513;
% sensorData(1).type = 'CW';
% sensorData(1).fre = 2000;
%
% sensorData(2).angle = 36.235899172122004;
% sensorData(2).type = 'LFM';
% sensorData(2).fre = 0;
%
% sensorData(3).angle = 45.763144826916495;
% sensorData(3).type = 'CW';
% sensorData(3).fre = [1000 2000];
%
% sensorData(4).angle = 74.618253631672147;
% sensorData(4).type = 'CW';
% sensorData(4).fre = [1000 2000];
%
% % 输出格式化后的结构数组
% disp(sensorData);

%% 数据对齐
% 确保不同传感器的数据在时间和空间上是对齐的
% 假装是对齐的 ，不要给自己增加实时处理的麻烦！

%% 数据合并
% 将来自不同传感器的数据按照时间和空间坐标存储在一个统一的数据结构中，
% 以便稍后的分析决策
% 基于时间的融合或基于特征的融合

%% 基于特征的融合
% CW

% LFM

% NOISE

%% 特征提取和融合

%% 多参量融合，看看怎么多参量融合

%% 分层关联

%% 实现纯方位交汇定位

%% 实现双曲面交汇定位

%% 实现时延差/方位联合定位

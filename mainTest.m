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
clc
clear
close all
%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];

%% 创建运动的声源对象

initial_position1 = [3e3, 3e3]; % 初始位置目标1
initial_position2 = [7e3, 5e3]; % 初始位置目标2

velocity1 = [100, 15]; % 运动速度（假设在 x 轴上匀速运动）
velocity2 = [-15, 15]; % 运动速度（假设在 x 轴上匀速运动）

source1 = SoundSource('CW', [2e3, 5e3], [100, 50], initial_position1, velocity1);
source2 = SoundSource('LFM', [1e3, 2e3], [100, 50], initial_position2, velocity2);

sourceAll = [source1, source2];
%% 模拟数据

time_steps = 10; % 假设有 10 个时间步长
delta_time = 1; % 每个时间步长的时间间隔

% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：目标，维度3：时刻
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource, time_steps + 1); % cell矩阵

for i = 1:time_steps + 1
    source1 = source1.updatePosition(delta_time - 1);
    disp(source1.Position)
    source2 = source2.updatePosition(delta_time - 1);

    % 获取每个平台的每个目标信息
    for j = 1:numOfPlatForm % 遍历平台
        for k = 1 : numOfSource % 遍历声源
            [angle, distance, t_delay] = platFormAll(j).getTargetInfo(sourceAll(k));
            t_Num = round(t_delay / delta_time) + i; % 放到此时刻传播时延之前的时刻
            target_info_matrix{j, k, t_Num} = struct('angle', angle, 'distance', distance);
        end
    end
end
% for i = 1 : 10
%     i
%     disp(target_info_matrix{1, 1, i});
% end
% 显示目标信息矩阵
disp('Target Information Matrix:');
disp(target_info_matrix);
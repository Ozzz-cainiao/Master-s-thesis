%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\alarm_negatives.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-12
% 描述: 这个程序是仿真虚警和漏报的demo程序
% 输入:  
% 输出:  
%**************************************************************************

clc;
clear;
close all;
% 模拟虚警、漏报和正常观测到目标的情况
num_measurements = 100; % 总共的观测次数
prob_false_alarm = 0.1; % 虚警概率
prob_missed_detection = 0.05; % 漏报概率

% 创建一个 cell 数组来存储观测结果
measurements = cell(num_measurements, 1);

for i = 1:num_measurements
    % 生成随机数，根据虚警和漏报概率确定观测结果
    if rand < prob_false_alarm
        % 虚警，随机生成2到5个虚假目标的方位
        num_targets = randi([2, 5]); % 随机生成目标个数
        directions = rand(1, num_targets) * 360; % 随机生成目标方位角度
        measurements{i} = directions; % 存储到cell数组中
    elseif rand < prob_false_alarm + prob_missed_detection
        % 漏报，未观测到目标，当前 cell 存储 NaN
        measurements{i} = NaN;
    else
        % 正常情况下观测到一个真实目标的方位，当前 cell 存储一个方位角度
        measurements{i} = rand * 360;
    end
end
% 绘制观测结果图
figure;
hold on;
for i = 1:num_measurements
    if iscell(measurements{i})
        directions = measurements{i};
        for j = 1:length(directions)
            plot(i, directions(j), 'Marker','square');
        end
    elseif isnan(measurements{i})
        plot(i, 400, 'x', 'Color', 'r');
    else
        plot(i, measurements{i}, 'Marker','*');
    end
end
hold off;
title('Simulation of False Alarms, Missed Detections, and Normal Detections');
xlabel('Measurement Number');
ylabel('Direction (degrees)');
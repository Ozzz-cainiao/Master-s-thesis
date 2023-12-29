%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\FOA\track.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-22
% 描述: 输入k, b,和点的坐标，计算点到直线的距离
% 输入:  
% 输出:  
%**************************************************************************

clc
clear 
close all


% 设置初始位置和速度
initialPosition = [-400, 800];
velocity = 8;  % m/s

% 设置目标点
targetPoint1 = [0, 0];  % 更改为你想要的目标点
targetPoint2 = [0, 0];  % 更改为你想要的目标点
targetPoint3 = [0, 0];  % 更改为你想要的目标点
targetPoint4 = [0, 0];  % 更改为你想要的目标点
% 直线的斜率
slope = -0.5;
% 设置仿真参数
simulationTime = 500;  % 仿真时间（秒）
timeStep = 0.01;  % 时间步长（秒）

% 初始化数组来存储位置信息
timePoints = 0:timeStep:simulationTime;
positions = zeros(length(timePoints), 2);

% 模拟目标运动
currentPosition = initialPosition;
for i = 1:length(timePoints)
    % 计算下一个位置
    currentPosition(1) = currentPosition(1) + velocity*timeStep* cos(atan(slope));  % x方向
    currentPosition(2) = currentPosition(2) + velocity*timeStep* sin(atan(slope));  % y方向
    
    % 存储当前位置
    positions(i, :) = currentPosition;
end

% 计算目标点到轨迹上每个点的距离
distances = sqrt(sum((positions - targetPoint).^2, 2));

% 找到距离最小的点的索引
[minDistance, minIndex] = min(distances);

% 绘制直角坐标系和目标运动轨迹
figure;
plot(positions(:, 1), positions(:, 2), 'b-', 'LineWidth', 2);
hold on;
plot(initialPosition(1), initialPosition(2), 'ro', 'MarkerSize', 10);
plot(targetPoint(1), targetPoint(2), 'ks', 'MarkerSize', 10);  % 目标点用方块表示
plot(positions(minIndex, 1), positions(minIndex, 2), 'go', 'MarkerSize', 10);
title('目标运动仿真');
xlabel('X轴位置（m）');
ylabel('Y轴位置（m）');
grid on;
axis equal;

% 显示坐标轴
line([min(positions(:, 1)), max(positions(:, 1))], [0, 0], 'Color', 'k');
line([0, 0], [min(positions(:, 2)), max(positions(:, 2))], 'Color', 'k');

% 显示图例
legend('运动轨迹', '初始位置', '目标点', '最近点', 'Location', 'Best');

% 显示结果
disp(['目标点坐标: (', num2str(targetPoint(1)), ', ', num2str(targetPoint(2)), ')']);
disp(['最近点坐标: (', num2str(positions(minIndex, 1)), ', ', num2str(positions(minIndex, 2)), ')']);
disp(['最短距离: ', num2str(minDistance), ' 米']);

k = 0;
b = 1;
x0 = 3;
y0 = 5;

distance = pointToLineDistance(k, b, x0, y0);
disp(distance);


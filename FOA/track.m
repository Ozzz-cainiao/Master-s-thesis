%% FOA 目标运动轨迹设置


clc
clear
close all

% 设置初始位置和速度
initialPosition = [-500, 1000];
velocity = 8;  % m/s

% 设置目标点
targetPoint = [0, 1000];  % 更改为你想要的目标点
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


%%
clc
clear
close all
% 初始位置
initialPosition = [-500, 1000];

% 直线的斜率
slope = -0.5;

% 速度
velocity = 10;

% 模拟的时间
simulationTime = 100;

% 时间步长为1秒
time = 0:1:simulationTime;

% 根据直线方程计算y坐标
y = velocity * sin(atan(-0.5))* (time - time(1)) + initialPosition(2);

% 根据速度计算x坐标
x = velocity * cos(atan(-0.5))* (time - time(1)) + initialPosition(1);

% 绘制运动轨迹
figure;
plot(x, y, '-o', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
scatter(initialPosition(1), initialPosition(2), 100, 'red', 'filled', 'MarkerEdgeColor', 'black');
xlabel('X (m)');
ylabel('Y (m)');
title('Target Trajectory Simulation');
grid on;
legend('Target Trajectory', 'Initial Position', 'Location', 'Best');
hold off;

%%
% 已知直线的起始位置和斜率
x0 = 1; % 起始位置的x坐标
y0 = 2; % 起始位置的y坐标
slope = 3; % 斜率

% 计算一般方程的系数
A = -slope;
B = 1;
C = slope * x0 - y0;

% 从一般方程中获取y轴上的截距
y_intercept = -C / B;

% 显示结果
disp(['直线在y轴上的截距为: ' num2str(y_intercept)]);


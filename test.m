% 设置初始位置和速度
initialPosition = [-500, 1000];
velocity = 8;  % m/s

% 设置目标点
targetPoint = [0, -0];  % 更改为你想要的目标点

% 设置仿真参数
simulationTime = 100;  % 仿真时间（秒）
timeStep = 0.1;  % 时间步长（秒）

% 初始化数组来存储位置信息
timePoints = 0:timeStep:simulationTime;
positions = zeros(length(timePoints), 2);

% 模拟目标运动
currentPosition = initialPosition;
for i = 1:length(timePoints)
    % 计算下一个位置
    currentPosition(1) = currentPosition(1) + velocity * cos(atan(-0.5));  % x方向
    currentPosition(2) = currentPosition(2) + velocity * sin(atan(-0.5));  % y方向
    
    % 存储当前位置
    positions(i, :) = currentPosition;
end

% 初始化夹角余弦值数组
cosineAngles = zeros(length(timePoints), 1);

% 计算每个时刻的夹角余弦值
for i = 1:length(timePoints)
    % 计算目标运动方向向量
    directionVector = [positions(i, 1) - initialPosition(1), positions(i, 2) - initialPosition(2)];
    
    % 计算观测点指向目标的向量
    observationVector = targetPoint - positions(i, :);
    
    % 计算夹角余弦值
    cosineAngles(i) = dot(directionVector, observationVector) / (norm(directionVector) * norm(observationVector));
end

% 绘制直角坐标系和目标运动轨迹
figure;
plot(timePoints, cosineAngles, 'b-', 'LineWidth', 2);
title('目标运动夹角余弦值');
xlabel('时间（秒）');
ylabel('夹角余弦值');
grid on;

% 显示结果
disp('夹角余弦值数组:');
disp(cosineAngles');

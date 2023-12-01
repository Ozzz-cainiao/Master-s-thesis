% 参数定义
c = 1500; % 声速（米/秒）
sensor1_pos = [0, 0]; % 第一个传感器位置
sensor2_pos = [1000, 0]; % 第二个传感器位置
theta1 = 45; % 从第一个传感器到目标的方位角（度）
theta2 = 135; % 从第二个传感器到目标的方位角（度）
t1 = 1; % 第一个传感器接收到信号的时间
t2 = 1.5; % 第二个传感器接收到信号的时间

% 时间差
delta_t = t2 - t1; 

% 从时间差计算距离差
distance_diff = c * delta_t;

% 将方位角转换为弧度
theta1_rad = deg2rad(theta1);
theta2_rad = deg2rad(theta2);

% 使用三角关系和方位角来估计目标位置
% 注意：这里的解可能不唯一，实际情况可能需要更复杂的方法来解决
x_target = (sensor1_pos(1) + sensor2_pos(1))/2 + distance_diff/2 * (tan(theta1_rad) - tan(theta2_rad));
y_target = (sensor1_pos(2) + sensor2_pos(2))/2 + distance_diff/2 * (1/tan(theta1_rad) - 1/tan(theta2_rad));

% 显示目标位置
fprintf('Estimated target position: (%.2f, %.2f)\n', x_target, y_target);



%% 
clc
clear all

syms x1 y1 x2 y2 c xs ys theta t1 t2 ts a b d m n p

eq1 = (x1 - xs)^2 + (y1 - ys)^2 - c^2 * (t1 - ts)^2;
eq2 = (x2 - xs)^2 + (y2 - ys)^2 - c^2 * (t2 - ts)^2;
eq3 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2) + 2 * sqrt((x1 - xs)^2 + (y1 - ys)^2) * sqrt((x1 - x2)^2 + (y1 - y2)^2) * cos(theta);

a = x1 - xs;
b = y1 - ys;
d = t1 - ts;
m = x2 - x1;
n = y2 - y1;
p = t2 - t1;
eq1_revised = a^2 + b^2 - c^2 * d^2;
eq3_revised = (x2 - xs)^2 + (y2 - ys)^2 - (a^2 + b^2 + m^2 + n^2) + 2 * sqrt(a^2 + b^2) * sqrt(m^2 + n^2) * cos(theta);
% eq = [eq1, eq2, eq3];
eq = [eq1_revised, eq2, eq3_revised];
solutions = solve(eq, [xs, ys, ts]);



%% 
% MATLAB 代码示例：设置 CW 信号参数

% 基本参数
f = 1e3;           % 信号频率，单位：赫兹（Hz）
fs = 10e3;         % 采样频率，单位：赫兹（Hz），通常为信号频率的5-10倍
duration = 1;       % 信号持续时间，单位：秒（s）
amplitude = 1;      % 信号幅度
T = 3;
% 计算采样点数量
nSamples = fs * duration;

% 生成时间向量
t = (0:nSamples-1)/fs;

% 生成 CW 信号
signal = amplitude * sin(2 * pi * f * t);

% 绘制信号图
plot(t, signal);
xlabel('Time (s)');
ylabel('Amplitude');
title('Continuous Wave (CW) Signal');

% 可选：保存信号到文件
% audiowrite('cw_signal.wav', signal, fs);

%% 
% 参数设置
Fs = 1000;                  % 采样频率
T = 1;                      % 总仿真时间
t = 0:1/Fs:T-1/Fs;          % 时间向量
f = 10;                     % 信号频率
speed = 30;                 % 目标速度 (单位: 米/秒)
initial_position = 0;       % 初始位置

% 目标位置随时间变化
position = initial_position + speed * t;

% 生成CW脉冲信号
signal = cos(2 * pi * f * t);

% 可视化信号和目标位置
subplot(2,1,1);
plot(t, signal);
title('CW脉冲信号');
xlabel('时间 (秒)');
ylabel('振幅');

subplot(2,1,2);
plot(t, position);
title('目标位置');
xlabel('时间 (秒)');
ylabel('位置 (米)');

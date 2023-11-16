% 设置参数
fs = 100e6; % 采样率（Hz）
T = 1e-6; % 信号的持续时间（秒）
beta = 1e6; % 调频斜率（Hz/秒）

% 生成时间向量
t = 0:1/fs:T;

% 生成 LFM 信号
s = exp(1j * pi * beta * t.^2);

% 绘制信号波形
plot(t, real(s));
xlabel('时间 (s)');
ylabel('实部');
title('LFM 信号');

filepath = mfilename('fullpath');
filepath = filepath+".m";

disp(filepath);
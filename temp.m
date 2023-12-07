%% 
clc
clear

syms x1 y1 x2 y2 c xs ys theta t1 t2 ts a b d m n p

eq1 = (x1 - xs)^2 + (y1 - ys)^2 - c^2 * (t1 - ts)^2;
eq2 = (x2 - xs)^2 + (y2 - ys)^2 - c^2 * (t2 - ts)^2;
eq3 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2) + 2 * sqrt((x1 - xs)^2 + (y1 - ys)^2) * sqrt((x1 - x2)^2 + (y1 - y2)^2) * cos(theta);

% a = x1 - xs;
% b = y1 - ys;
% d = t1 - ts;
% m = x2 - x1;
% n = y2 - y1;
% p = t2 - t1;
eq1_revised = a^2 + b^2 - c^2 * d^2;
eq3_revised = (x2 - xs)^2 + (y2 - ys)^2 - (a^2 + b^2 + m^2 + n^2) + 2 * sqrt(a^2 + b^2) * sqrt(m^2 + n^2) * cos(theta);
% eq = [eq1, eq2, eq3];
eq12 = eq1 - eq2;
eq13 = eq1 - eq3;
eq23 = eq2 - eq3;
eq = [eq1, eq2, eq3];
solutions = solve(eq, [xs, ys, ts]);


%% 生成对应的信号


fs = 48000; % 采样频率 (Hz)
duration = 1; % 信号持续时间 (秒)
t = 0:1/fs:duration-1/fs; % 时间向量

% 生成线谱信号 (350 Hz, 400 Hz, 450 Hz, 500 Hz)
signal = sin(2*pi*350*t) + sin(2*pi*400*t) + sin(2*pi*450*t) + sin(2*pi*500*t);

% 生成初步噪声
noise = randn(size(t));

% 设计一个带通滤波器 (300-600 Hz)
[b, a] = butter(4, [300, 600]/(fs/2), 'bandpass');
filtered_noise = filter(b, a, noise);

% 计算信号和噪声的功率
signal_power = rms(signal)^2;
noise_power = rms(filtered_noise)^2;

% 调整噪声的功率以实现-20 dB的信噪比
desired_snr = -20; % dB
desired_noise_power = signal_power / (10^(desired_snr / 10));
adjusted_noise = filtered_noise * sqrt(desired_noise_power / noise_power);

% 叠加信号和噪声
combined_signal = signal + adjusted_noise;

% 绘制信号和噪声叠加后的波形
plot(t, combined_signal);
title('Combined Signal with SNR = -20 dB');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;



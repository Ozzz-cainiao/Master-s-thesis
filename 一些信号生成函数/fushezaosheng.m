%% chatgpt生成水下目标辐射噪声信号

% 最终生成的信号将满足你的要求，
% 其频率范围为1Hz-10000Hz，
% 低频段为6 ～12 dB/oct 的正斜率，
% 高频段为－6dB/oct 左右的负斜率，
% 峰值出现在100 ～1 000 Hz 之间。

clc
clear
%生成高斯白噪声
N=100000;   %采样点数
n=1:N;      %将采样点数存储为1:N的数组，方便后续绘图
x1=randn(1,N);  %生成长度为N的高斯白噪声信号

%定义LMS自适应FIR滤波器参数
Mu=0.005;   %步长，控制滤波器收敛速度和稳定性
N1=128;     %滤波器长度，即滤波器的阶数
F1=0;       %低通滤波器的截止频率1
F2=10000;   %低通滤波器的截止频率2
Fp1=100;    %低通滤波器的通带频率
Fp2=1000;    %低通滤波器的通带频率
Fs=50000;   %采样频率
fc=[F1,Fp1,Fp2,F2];  %定义滤波器的频率响应的三个点（截止、通带和截止频率）
M=[0,1,0.5,1];      %对应三个点的几何响应
dev=[0.01,0.01,10^(0.01/20),0.01];   %通带和截止频率处的幅频波动

[b,a]=firpm(N1 - 1,fc/(Fs/2),M,dev);   %设计低通滤波器，返回滤波器系数b和a

%使用LMS滤波器对高斯白噪声进行滤波处理
[y1,e1]=firlms(x1,a,Mu,N1);   %firlms函数求解

%绘制高斯白噪声和滤波后的信号
subplot(2,1,1);
plot(n,x1);
title('高斯白噪声');
subplot(2,1,2);
plot(n,y1);
title('LMS自适应FIR滤波器处理后的信号');   %绘制两幅图的标题
% 
% 
% %生成高斯白噪声
% N=100000;   %采样点数
% n=1:N;      %将采样点数存储为1:N的数组，方便后续绘图
% x1=randn(1,N);  %生成长度为N的高斯白噪声信号
% 
% %定义LMS自适应FIR滤波器参数
% Mu=0.005;   %步长，控制滤波器收敛速度和稳定性
% N1=128;     %滤波器长度，即滤波器的阶数
% F1=1;       %低通滤波器的截止频率1
% F2=10000;   %低通滤波器的截止频率2
% Fp=1000;    %低通滤波器的通带频率
% Fs=50000;   %采样频率
% fc=[F1,Fp,F2];  %定义滤波器的频率响应的三个点（截止、通带和截止频率）
% M=[0,1,0];      %对应三个点的几何响应
% dev=[0.01,10^(0.01/20),0.01];   %通带和截止频率处的幅频波动
% 
% [b,a]=firpm(N1 - 1,fc/(Fs/2),M,dev);   %设计低通滤波器，返回滤波器系数b和a
% 
% %使用LMS滤波器对高斯白噪声进行滤波处理
% [y1,e1]=firlms(x1,a,Mu,N1);   %firlms函数求解
% 
% %绘制高斯白噪声和滤波后的信号
% subplot(2,1,1);
% plot(n,x1);
% title('高斯白噪声');
% subplot(2,1,2);
% plot(n,y1);
% title('LMS自适应FIR滤波器处理后的信号');   %绘制两幅图的标题

%% 仅设计LMS滤波器
clc
clear
% 设置滤波器参数
fs = 44100; % 采样率为44100Hz
f1 = 1; % 低截止频率为1Hz
f2 = 10000; % 高截止频率为10000Hz
N = 512; % 系统阶数
m = N/2; % 每次迭代更新的系数数
mu = 0.001; % 自适应步长

% 生成目标信号 
t = 0:1/fs:10; % 生成10秒的时间点 
x = sin(2*pi*200*t)+ sin (2*pi*500*t) + sin (2*pi*800*t); % 生成三个正弦信号
d = x + 0.1*randn(size(t)); % 加入噪声

% 初始化滤波器系数和缓存数组
h = zeros(N,1); % 滤波器系数
xk = zeros(N,1); % 缓存输入信号
yk = zeros(N,1); % 缓存滤波器输出

% 迭代更新滤波器系数
for k = m+1:length(t) % 从第m+1个样点开始迭代
xk(1:m) = xk(N-m+1:N); % 准备新的输入数据
xk(m+1:N) = x(k-m:k-1); % 将接收到的新数据加入缓存
yk(k) = h.'*xk; % 计算滤波器输出
e(k) = d(k) - yk(k); % 计算误差信号
h = h + mu*e(k)*conj(xk); % 更新滤波器系数
end

% 绘制频率响应曲线 
f = linspace(0,fs/2,1000); % 生成频率点 
H = freqz(h,1,f,fs); % 计算滤波器的频率响应 
HdB = 20*log10(abs(H)); % 将幅度转换为dB 
HdB = HdB - max(HdB); % 归一化到0dB 
figure; % 打开一个新窗口 
plot (f,HdB); % 绘制频率响应曲线 
xlim([0,10000]); % 设置X轴范围 
ylim([-12,0]); % 设置Y轴范围 
xlabel('Frequency (Hz)'); % 设置X轴标签 
ylabel('Amplitude (dB)'); % 设置Y轴标签 
title('Frequency Response of LMS Adaptive FIR Filter '); % 设置标题

% 绘制原始信号和滤波器输出信号 
figure; % 打开一个新窗口 
subplot(2,1,1); % 设置上下两个子图 
plot (t,x); % 绘制原始信号 
hold on; % 保持当前绘图状态 
plot(t,d); % 绘制带噪声的信号 
xlabel(' Time (s)'); % 设置X轴标签 
ylabel('Amplitude'); % 设置Y轴标签 
legend ('Original Signal ','Noisy Signal'); % 设置图例 
title('Input Signal'); % 设置标题 
subplot(2,1,2); % 切换到下一个子图 
plot(t,yk); % 绘制滤波器输出信号 
xlabel('Time (s)'); % 设置X轴标签 
ylabel('Amplitude'); % 设置Y轴标签 
title(' Filtered Signal'); % 设置标题


% % 绘制原始信号和滤波器输出信号 频谱图
% figure; % 打开一个新窗口 
% subplot(2,1,1); % 设置上下两个子图 
% plot (t,x); % 绘制原始信号 
% hold on; % 保持当前绘图状态 
% plot(t,d); % 绘制带噪声的信号 
% xlabel(' Time (s)'); % 设置X轴标签 
% ylabel('Amplitude'); % 设置Y轴标签 
% legend ('Original Signal ','Noisy Signal'); % 设置图例 
% title('Input Signal'); % 设置标题 
% subplot(2,1,2); % 切换到下一个子图 
% plot(t,yk); % 绘制滤波器输出信号 
% xlabel('Time (s)'); % 设置X轴标签 
% ylabel('Amplitude'); % 设置Y轴标签 
% title(' Filtered Signal'); % 设置标题

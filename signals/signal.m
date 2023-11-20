%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\signals\signal.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-20
% 描述: 在这个文件中仿真舰船辐射噪声的信号
% 输入:  
% 输出:  
%**************************************************************************


%% 参考文献
% [1] 李琴;苑秉成;张文娟; 舰船辐射噪声建模及仿真模拟器的实现[J]. 舰船科学技术, 2010(04 vo 32): 121-124+133.


%% 线谱
% 3种
% 100Hz以下是与叶片数和螺旋桨数有关的 f = m * n * s; 频率 谐波次数 螺旋桨叶片数 螺旋桨转速r/s
% 一般都是高速情况下用小螺旋桨，低速的用大螺旋桨，小螺旋桨一般适用吨位较小的船舶，大约为800rpm每分以上，
% 大螺旋桨300-450转每分钟，如果发动机功率较小的中型货轮一般都是300转。  5-15转每秒 
% 潜艇7叶浆，低转速；大型船舶5叶桨/3叶浆，低转速
% 100-1000Hz 因船型而异，可根据一般情况模拟设置k个频率
% 线谱超过连续谱的值10-25分贝
% demon看谐波
% 
c = 1500; % 声速
numOfBlades = 5; % 桨叶数 3，4，5，7
vOfBlades = 5;   % 转速
orderOfHarmonic = 3; % 谐波次数
dolpper = (x * 1.8 / 3.6) * 0.5 / c; % 多普勒频移系数 参考《水 声 被 动 目 标 特 征 提 取 和 分 类 方 法 研 究》
frequencies = zeros(10); % 频率
for i = 1 : orderOfHarmonic
    frequencies(i) = numOfBlades * vOfBlades * i; % 计算谐波频率
end

% 设置采样率和信号时长
Fs = 32000;             % 采样率 (samples per second)
duration = 5;           % 信号时长 (seconds)

% 创建时间轴
t = 0:1/Fs:duration-1/Fs;   % 时间轴 (seconds)

% 初始化叠加信号
y = zeros(size(t));

% 定义频率、幅度和初始相位

amplitudes = [1 0.8 0.6 0.4 0.2 0.1 0.05 0.03 0.02 0.01];               % 幅度
phases = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4 2*pi 9*pi/4];    % 初始相位

% 创建正弦信号并进行叠加
for i = 1:numel(frequencies)
    freq = frequencies(i);
    amp = amplitudes(i);
    phase = phases(i);
    
    % 创建当前频率的正弦信号
    x = amp * sin(2*pi*freq*t + phase);
    
    % 将当前信号叠加到总信号上
    y = y + x;
end

% 绘制叠加后的信号
plot(t, y);
xlabel('时间 (秒)');
ylabel('振幅');
title('叠加正弦信号');
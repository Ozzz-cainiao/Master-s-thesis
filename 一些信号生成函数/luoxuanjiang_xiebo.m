%% 螺旋桨谐波仿真
%定义螺旋桨参数
blades = 5; %叶片数
speed = 3; %转速，单位为转每秒
harmonic = 4; %谐波次数

%计算谐波频率
fundamental_freq = blades*speed/(2*pi); %基频，单位为Hz
harmonic_freq = harmonic*fundamental_freq; %谐波频率，单位为Hz

%计算谐波波长
sound_speed = 1500; %水中声速，单位为m/s
harmonic_wavelength = sound_speed/harmonic_freq; %谐波波长，单位为m

%计算谐波声级
source_level = 160; %源强，单位为dB re 1uPa @ 1m
harmonic_sound_pressure = 10^((source_level - 20*log10(harmonic_wavelength/4*pi) - 8)/10); %谐波声压级，单位为Pa
harmonic_sound_pressure_ref = 2e-5; %参考声压级，单位为Pa
harmonic_sound_level = 20*log10(harmonic_sound_pressure/harmonic_sound_pressure_ref); %谐波声级，单位为dB re 20uPa

%输出结果
fprintf('基频为%.2f Hz时，第%d次谐波的频率为%.2f Hz，波长为%.2f m，声级为%.2f dB re 20uPa\n', fundamental_freq, harmonic, harmonic_freq, harmonic_wavelength, harmonic_sound_level);


%% 
clc
clear all

%定义螺旋桨参数
blades = 5; %叶片数
speed = 20; %转速，单位为转每秒
harmonic = 1:5; %谐波次数

%计算谐波频率
fundamental_freq = blades*speed/(2*pi); %基频，单位为Hz
harmonic_freq = harmonic*fundamental_freq; %谐波频率，单位为Hz

%计算谐波声级
source_level = 160; %源强，单位为dB re 1uPa @ 1m
harmonic_sound_pressure = 10.^((source_level - 20*log10(harmonic_freq) - 8)/10); %谐波声压级，单位为Pa
harmonic_sound_pressure_ref = 2e-5; %参考声压级，单位为Pa
harmonic_sound_level = 20*log10(harmonic_sound_pressure/harmonic_sound_pressure_ref); %谐波声级，单位为dB re 20uPa

%绘制图表
figure;
plot(harmonic_freq, harmonic_sound_level);
xlabel('Harmonic Frequency (Hz)');
ylabel('Harmonic Sound Level (dB re 20uPa)');
title('Propeller Harmonic Noise');

%输出结果
fprintf('基频为%.2f Hz时，第1~5次谐波的声级为：\n', fundamental_freq);
fprintf('%.2f dB re 20uPa @ %.2f Hz\n', [harmonic_sound_level; harmonic_freq]);

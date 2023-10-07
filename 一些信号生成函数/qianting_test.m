%% 使用chatgpt生成的仿真潜艇辐射噪声的程序
% 鱼雷噪声信号仿真程序
Fs = 44100;  % 采样频率
N = 2^16;    % 采样点数

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

% 生成0-100Hz的线谱
f1 = 20;        % 线谱频率
n1 = 4;         % 谐波次数
f = (0:N-1)/N*Fs;   % 频率序列
H1 = zeros(1, N/2+1);
H1(1:n1:end) = 1./[1:n1:length(H1)];
figure 
plot(H1)
H1 = H1 / max(H1);  % 归一化
H1_amp = db2mag(15 + mag2db(abs(H1))); % 线谱增益
H1 = H1_amp .* exp(1j*angle(H1)); % 线谱增益与相位
h1 = real(ifft([H1, conj(H1(end-1:-1:2))]));
h1 = h1(1:N/2+1);
h1 = h1 / max(h1);  % 归一化
y1 = fftfilt(h1, randn(1, N));
figure 
plot(y1)


% 生成三个在100-1000Hz范围内的随机频率的线谱
f2 = [randi([100, 1000]), randi([100, 1000]), randi([100, 1000])];
H2 = zeros(1, N/2+1);
for i = 1:length(f2)
    H2(f >= f2(i)-10 & f <= f2(i)+10) = rand;
end
H2_amp = db2mag(15 + mag2db(abs(H2))); % 线谱增益
H2 = H2_amp .* exp(1j*angle(H2)); % 线谱增益与相位
H2 = H2 / max(H2);  % 归一化
h2 = real(ifft([H2, conj(H2(end-1:-1:2))]));
h2 = h2(1:N/2+1);
h2 = h2 / max(h2);  % 归一化
y2 = fftfilt(h2, randn(1, N));

% 生成1-5000Hz的高斯噪声
y3 = randn(1, N);

% 合并三个部分的信号
y = y1 + y2 + y3;

% 播放生成的信号
soundsc(y, Fs);

% 绘制信号的时域波形和频谱图
figure;
subplot(211);
plot((0:N-1)/Fs, y);
xlabel('Time (s)');
ylabel('Amplitude');
title('Time domain waveform');
subplot(212);
semilogx(f, 20*log10(abs(fft(y))));
xlim([1, Fs/2]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Frequency spectrum');

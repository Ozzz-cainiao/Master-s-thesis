close all
clear
clc
%% 观测平台位置



%% CV 模型+高斯噪声 kalman滤波

% 时间步长
dt = 1;

% 状态转移矩阵
A = [1, dt; 0, 1]; % 

% 观测矩阵
H = [1, 0]; % 位置可观测，速度不可观测

% 过程噪声协方差
Q = [1, 0; 0, 3];

% 观测噪声协方差
R = 1;

t = 0: dt: 100; % 时间点
v = 5; % 目标真实的速度
x_real = v * t; % 目标真实的位置
x_ob = x_real + randn(size(t)) * sqrt(R); % 目标观测的位置

% 初始状态估计
x_est = [0; 0]; % [位置； 速度]

% 初始误差协方差
P = eye(2);

% 数据（用您的观测数据替换这里的数据）
observations = [1.2, 2.3, 3.4, 4.5, 5.5]; % 示例观测数据
res = zeros(size(observations, 2), 2);
i = 1;
% 卡尔曼滤波
% for z = observations
%     % 预测
%     x_pred = A * x_est;
%     P_pred = A * P * A' + Q;
% 
%     % 更新
%     K = P_pred * H' * inv(H*P_pred*H'+R);
%     x_est = x_pred + K * (z - H * x_pred);
%     P = (eye(2) - K * H) * P_pred;
% 
%     % 打印或存储结果
%     disp(x_est);
%     res(i,:) = x_est;
%     i = i + 1;
% end
for z = x_ob
    % 预测
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;

    % 更新
    K = P_pred * H' * inv(H*P_pred*H'+R);
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(2) - K * H) * P_pred;

    % 打印或存储结果
    disp(x_est);
    res(i,:) = x_est;
    i = i + 1;
end
figure
plot(res(:, 1));

%% 自己照着chatGPT 的提示写的



%%
Q = [1, 0; 0, 1];       %过程误差，Q矩阵
R = [0.1, 0; 0, 0.1];   %观测误差，R矩阵
A = [1, 1; 0, 1];       %A矩阵
H = [1, 0; 0, 1];       %H矩阵
I = [1, 0; 0, 1];       %单位矩阵
epoch = 30;
P = [1, 0; 0, 1];
X = ones(epoch, 2);     %生成位置和速度矩阵
X(1, :) = [0, 1];       %初始的实际位置和速度（从位置0开始，假定以匀速1运动）
Xa = ones(epoch, 2);    %用a表示头上的小帽子，后验
Xa(1, :) = [0, 1];
Xa_ = ones(epoch, 2);   %先验
Xa_(1, :) = [0, 1];     %初始化先验
Z = ones(epoch, 2);     %测量的位置和速度
Z(1, :) = [0, 1];       %初始测量的位置和速度
for k = 2:epoch
    %更新误差
    w1 = normrnd(0, sqrt(Q(1, 1))); %位置过程误差
    w2 = normrnd(0, sqrt(Q(2, 2))); %速度过程误差
    W = [w1, w2];
    v1 = normrnd(0, sqrt(R(1, 1))); %观测位置过程误差
    v2 = normrnd(0, sqrt(R(2, 2))); %观测速度过程误差
    V = [v1, v2];
    %更新实际位置、实际速度
    X(k, :) = (A * X(k-1, :)' + W')';
    %更新测量位置、测量速度
    Z(k, :) = (H * X(k, :)' + V')';
    %预测
    P_ = A * P * A' + Q;
    Xa_(k, :) = (A * Xa(k-1, :)')'; %先验
    %校正
    K = P_ * H' * inv((H * P_)*H'+R);
    P = (I - K * H) * P_;
    Xa(k, :) = (Xa_(k, :)' + K * (Z(k, :)' - H * Xa_(k, :)'))';
end
%通过以下三个值比较平均绝对误差。mae_Xa 小于mae_Xa_和mae_Z，说明融合有效
mae_Xa = sum(abs(Xa(:, 1)-X(:, 1)));
mae_Xa_ = sum(abs(Xa_(:, 1)-X(:, 1)));
mae_Z = sum(abs(Z(:, 1)-X(:, 1)));
plot(1:epoch, X(:, 1), 'r*-', 1:epoch, Z(:, 1), '-b*', 1:epoch, Xa_(:, 1), '-g*', 1:epoch, Xa(:, 1), '-k*', 'LineWidth', 2)

hold on
xlabel step
ylabel 位置
legend('实际位置', '测量位置', '先验估计位置', '后验估计位置')




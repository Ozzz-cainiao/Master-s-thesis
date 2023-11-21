close all
clear
clc
 
%%
Q = [1 0;0 1]; %过程误差，Q矩阵
R = [0.1 0;0 0.1]; %观测误差，R矩阵
A = [1 1;0 1]; %A矩阵
H = [1 0;0 1]; %H矩阵
I = [1 0;0 1]; %单位矩阵
epoch = 30; 
P = [1 0;0 1];
X = ones(epoch, 2); %生成位置和速度矩阵
X(1,:) =  [0 1];  %初始的实际位置和速度（从位置0开始，假定以匀速1运动）
Xa = ones(epoch, 2);  %用a表示头上的小帽子，后验
Xa(1,:) = [0 1];
Xa_ = ones(epoch, 2);  %先验
Xa_(1,:) = [0 1];  %初始化先验
Z = ones(epoch, 2); %测量的位置和速度
Z(1,:) = [0 1]; %初始测量的位置和速度
for k = 2:epoch
    %更新误差
    w1 = normrnd(0,sqrt(Q(1,1))); %位置过程误差
    w2 = normrnd(0,sqrt(Q(2,2))); %速度过程误差
    W = [w1 w2];
    v1 = normrnd(0,sqrt(R(1,1))); %观测位置过程误差
    v2 = normrnd(0,sqrt(R(2,2))); %观测速度过程误差
    V = [v1 v2];
    %更新实际位置、实际速度
    X(k,:) = (A*X(k-1,:)'+ W')';
    %更新测量位置、测量速度
    Z(k,:) = (H*X(k,:)' + V')';
    %预测
    P_ = A*P*A' + Q;
    Xa_(k,:) = (A*Xa(k-1,:)')'; %先验
    %校正
    K = P_*H'*inv((H*P_)*H' + R);
    P = (I -  K*H)*P_;
    Xa(k,:) = (Xa_(k,:)' + K*(Z(k,:)' - H*Xa_(k,:)'))';
end
%通过以下三个值比较平均绝对误差。mae_Xa 小于mae_Xa_和mae_Z，说明融合有效
mae_Xa = sum(abs(Xa(:,1)-X(:,1)));
mae_Xa_ = sum(abs(Xa_(:,1)-X(:,1)));
mae_Z = sum(abs(Z(:,1)-X(:,1)));
plot(1:epoch, X(:,1), 'r*-', 1:epoch, Z(:,1), '-b*', 1:epoch, Xa_(:,1), '-g*', 1:epoch, Xa(:,1), '-k*','LineWidth',2)
 
hold on
xlabel step
ylabel 位置
legend('实际位置', '测量位置', '先验估计位置', '后验估计位置')
%% 某版本  不太可用
% %状态转移矩阵
% F = [1, T, 0, 0; ...
%     0, 1, 0, 0; ...
%     0, 0, 1, T; ...
%     0, 0, 0, 1];
% H = [1, 0, 0, 0; ...
%     0, 0, 1, 0];
% %过程噪声
% B = [T^2 / 2, 0; ...
%     T, 0; ...
%     0, T^2 / 2; ...
%     0, T]; %过程噪声分布矩阵
% v = sigma_u^2; %x方向的过程噪声向量//相当于Q
% V = B * v * B';
% % %观测噪声??
% % W = B * noise_x;
% 
% %------Data initial-------%
% X_real = zeros(4, N);
% X = zeros(4, N);
% 
% Z1 = zeros(2, N);
% X_EKF1 = zeros(4, N);
% % P1 = zeros(4,4,N);
% % K1 = zeros(4,2,N);
% % Hj1 = zeros(2,4,N);
% Z2 = zeros(2, N);
% Z_polar2 = zeros(2, N);
% X_EKF2 = zeros(4, N);
% % P2 = zeros(4,4,N);
% % K2 = zeros(4,2,N);
% % Hj2 = zeros(2,4,N);
% 
% X_CC = zeros(4, N);
% X_BC = zeros(4, N);
% bias = zeros(8, N, M);
% 
% %-------Track Initial-------%
% X_real(:, 1) = [Rx, vx, Ry, vy]'; %x: km,km/s
% 
% X_EKF1(:, 1) = [Rx, 0, Ry, 0];
% X_EKF2(:, 1) = [Rx, 0, Ry, 0];
% X_CC(:, 1) = [Rx, 0, Ry, 0];
% X_BC(:, 1) = [Rx, 0, Ry, 0];
% 
% %Monto-carlo
% for m = 1:M
%     noise_x = randn(2, N) .* sigma_x; %过程噪声
%     noise_z1 = randn(2, N) .* sigma_z; %观测噪声
%     noise_z2 = randn(2, N) .* sigma_z;
% 
%     %构造 真实轨迹X 与 观测轨迹Z
%     for n = 2:N
%         if n == 30
%             X_real(2, n-1) = 1;
%         end
%         X_real(:, n) = F * X_real(:, n-1);
%     end
%     X = X_real + B * noise_x;
%     Z1 = H * X + noise_z1 - [x1, 0; 0, y1] * ones(2, N);
%     Z2 = H * X + noise_z1 - [x2, 0; 0, y2] * ones(2, N);
% 
%     %这里可以写成function的形式
%     P_BC = P1;
%     for n = 2:N
%         x_predict = F * X_EKF1(:, n-1); %状态一步预测
%         p_predict = F * P1 * F' + V; %协方差一步预测
%         S = H * p_predict * H' + R1; %新息协方差
%         K1 = p_predict * H' / S; %增益
%         X_EKF1(:, n) = x_predict + K1 * (Z1(:, n) - H * x_predict + [x1; y1]); %状态更新方程
%         P1 = (eye(4) - K1 * H) * p_predict; %协方差更新方程 %后面一半要不要？
% 
%         x_predict2 = F * X_EKF2(:, n-1); %状态一步预测
%         p_predict2 = F * P2 * F' + V; %协方差一步预测
%         S2 = H * p_predict2 * H' + R2; %新息协方差
%         K2 = p_predict2 * H' / S2; %增益
%         X_EKF2(:, n) = x_predict2 + K2 * (Z2(:, n) - H * x_predict2 + [x2; y2]); %状态更新方程
%         P2 = (eye(4) - K2 * H) * p_predict2; %协方差更新方程 %后面一半要不要？
% 
%         P_CC = inv(inv(P1)+inv(P2));
%         X_CC(:, n) = P_CC * (P1 \ X_EKF1(:, n) + P2 \ X_EKF2(:, n));
% 
%         P_BC = (eye(4) - K2 * H) * F * P_BC * F' * (eye(4) - K1 * H)';
%         X_BC(:, n) = X_EKF2(:, n) + (P2 - P_BC) / (P1 + P2 - 2 * P_BC) * (X_EKF1(:, n) - X_EKF2(:, n));
%     end
% end
% 定义状态转移函数
f = @(x,dt) [x(1) + x(2)*cos(x(3))*dt;    % 目标的x坐标
              x(2) + x(2)*sin(x(3))*dt;    % 目标的y坐标
              x(3)];                       % 目标的方位角

% 定义观测函数
h = @(x) x(3);  % 观测值为目标的方位角

% 初始化参数
dt = 0.1;  % 时间间隔
Q = diag([0.1, 0.1, 0.01]);  % 过程噪声协方差
R = 0.1;  % 观测噪声方差
x0 = [0; 0; 0];  % 初始状态：目标位置和方位角
P0 = eye(3);  % 初始协方差矩阵

% 使用UKF进行状态估计
ukf = unscentedKalmanFilter(f,h,x0,P0,'ProcessNoise',Q,'MeasurementNoise',R);

% 模拟目标运动并生成观测数据
t = 0:dt:10;  % 时间向量
true_state = [2*t; 2*ones(size(t)); 0.1*t];  % 真实状态：目标的x坐标、y坐标和方位角
observed_state = true_state(3,:) + randn(size(t))*sqrt(R);  % 添加高斯噪声的观测值

% 使用UKF进行状态估计
estimated_state = zeros(3,length(t));
for i = 1:length(t)
    correct(ukf,observed_state(i));  % 使用观测值更新状态估计
    predict(ukf);  % 预测下一时刻的状态
    estimated_state(:,i) = ukf.State;  % 保存估计的状态
end

% 绘制结果
figure;
subplot(3,1,1);
plot(t,true_state(1,:),'b-',t,estimated_state(1,:),'g--');
legend('True x','Estimated x');
xlabel('Time');
ylabel('X position');
title('Target Tracking using UKF');

subplot(3,1,2);
plot(t,true_state(2,:),'b-',t,estimated_state(2,:),'g--');
legend('True y','Estimated y');
xlabel('Time');
ylabel('Y position');

subplot(3,1,3);
plot(t,true_state(3,:),'b-',t,estimated_state(3,:),'g--');
legend('True angle','Estimated angle');
xlabel('Time');
ylabel('Angle');

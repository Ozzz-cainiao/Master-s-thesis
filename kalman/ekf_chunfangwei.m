%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EKF在纯方位目标跟踪中的应用实例子
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
close all
T=1;                    %雷达扫描周期
N=120/T;                %总的采样次数
X=zeros(4,N);           %存放实际位置、速度信息
Z=zeros(1,N);           %存放测量角度信息
X(:,1)=[200,10,300,5];  %初始化目标状态
x0=50;
y0=100;
Xstation=[50,100];      %雷达的位置
F=[1,T,0,0;
	0,1,0,0;
	0,0,1,T;
	0,0,0,1];            %状态矩阵
G=[T^2/2 0;
	T 0;
	0 T^2/2;
	0 T];                    %状态噪声矩阵
delta_w=1e-3;
R=2;                                   %测量噪声方差
Q=delta_w*diag([0.5 1]);                        %均方差
P=eye(4);                                       %协方差矩阵初始化
for i=2:N
	X(:,i)=F*X(:,i-1)+G*sqrtm(Q)*randn(2,1);
end
for i=1:N
	Z(:,i)=ffun(X(:,i),Xstation)+sqrtm(R)*randn(1,1);
end
X_ekf=zeros(4,N);                          %EKF初始化。
X_ekf(:,1)=X(:,1);
I=eye(4);
for i=2:N
	Xn=F*X_ekf(:,i-1);
	Zn=ffun(Xn,Xstation);
	d=Dist(Xn,Xstation);
	dd=d^2;
	H=[(y0-Xn(3))/dd,0,(Xn(1)-x0)/dd,0];
	P0=F*P*F'+G*Q*G';
	Kk=P0*H'/(H*P0*H'+R);
	X_ekf(:,i)=Xn+Kk*(Z(i)-Zn);
	P=(I-Kk*H)*P0;
end
%误差分析
for i=1:N
	Err_KalmanFilter(i)=Dist(X(:,i),X_ekf(:,i));
end
figure
hold on;box on;grid on;
plot(X(1,:),X(3,:),'-r*');
plot(X_ekf(1,:),X_ekf(3,:),'-b*');
legend('真实值','EKF估计值');
xlabel('横坐标X/m');
ylabel('纵坐标Y/m');
title('真实值和估计值结果图')

figure
grid on;
plot(Err_KalmanFilter,'-ks','MarkerFace','r');
xlabel('时间(s)');
ylabel('位置估计偏差(m)');
title('估计值与真实值的偏差');


figure
hold on;box on;grid on;
plot(Z,'-ks','MarkerFace','r');
plot(Z+sqrtm(R)*randn(1,1),'-bo','MarkerFace','b');
legend('真实角度','加入噪声的角度');
xlabel('时间(s)');
ylabel('角度值(°)');
title('真实角度和引入噪声的角度比较');


%%

% 初始化
dt = 0.1; % 时间步长
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % 状态转移矩阵
H = [1 0 0 0; 0 1 0 0]; % 观测矩阵
Q = 0.1 * eye(4); % 过程噪声协方差
R = 1 * eye(2); % 观测噪声协方差

% 初始状态估计和协方差矩阵
x = [0; 0; 0; 0]; % 初始状态估计
P = eye(4); % 初始协方差矩阵

% 模拟数据
t = 0:dt:10;
true_position = [2 * t; 2 * t]; % 真实位置，假设在半径为2的圆周上匀速运动
measurements = true_position + sqrt(R) * randn(2, length(t)); % 加入观测噪声的测量值

% 手动实现卡尔曼滤波
filtered_position = zeros(2, length(t));
for i = 1:length(t)
    % 预测
    x = A * x;
    P = A * P * A' + Q;

    % 更新
    K = P * H' / (H * P * H' + R);
    x = x + K * (measurements(:, i) - H * x);
    P = (eye(4) - K * H) * P;

    filtered_position(:, i) = x(1:2);
end

% 绘图
figure;
plot(true_position(1, :), true_position(2, :), 'b', measurements(1, :), measurements(2, :), 'rx', filtered_position(1, :), filtered_position(2, :), 'g');
legend('真实位置', '测量值', '滤波估计');
xlabel('x');
ylabel('y');
title('基于CV模型的二维坐标卡尔曼滤波');
axis equal;



function d=Dist(x1,x2)
	if length(x2)==2
		d=sqrt((x1(1)-x2(1))^2+(x1(3)-x2(2))^2);
	else
		d=sqrt((x1(1)-x2(1))^2+(x1(3)-x2(3))^2);
	end
end

function cita=ffun(X1,X0)
	if(X1(3,1)-X0(1,2)>=0)      %y1-y0>0
		if(X1(1,1)-X0(1,1)>0)   %x1-x2>0 第一象限
			cita=atand(abs((X1(3,1)-X0(1,2))/(X1(1,1)-X0(1,1))));
		elseif(X1(1,1)-X0(1,1)==0)   %y轴上半轴
			cita=asind(1);
		else                      %x1-x3<0 第二象限
			cita=90+atand(abs((X1(3,1)-X0(1,2))/(X1(1,1)-X0(1,1))));
		end
	else
		if(X1(1,1)-X0(1,1)>0)   %x1-x2>0 第四象限
			cita=270+atand(abs((X1(3,1)-X0(1,2))/(X1(1,1)-X0(1,1))));
		elseif(X1(1,1)-X0(1,1)==0)   %y轴下半轴
			cita=270;
		else                      %x1-x3<0 第三象限
			cita=180+atand(abs((X1(3,1)-X0(1,2))/(X1(1,1)-X0(1,1))));
		end
	end
end
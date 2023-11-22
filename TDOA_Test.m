%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA_Test.m
% 版本: v1.0
% 作者: 网络资源
% 联系方式: https://blog.csdn.net/weixin_44606638/article/details/106390708
% 日期: 2023-11-21
% 描述: 要求一：编写两个函数TDOA_CHAN和TDOA_Taylor得到位置的估计。
%       要求二：用RMSE实现两种算法的性能比较, 得到两种算法的RMSE曲线对比图，横坐标为噪声方差，纵坐标为RMSE。
% 输入:
% 输出:theta其实是(x, y)
%**************************************************************************

%% TDOA部分：
% %% the simulation of TDOA localization algorithm
clear;
clc;
%定义四个参与基站的坐标位置
BS1 = [0, 0];
BS2 = [5000, 0];
BS3 = [5000, 5000];
BS4 = [0, 5000];
%BS5=[600,500];
%移动台MS的初始估计位置
MS = [3500, 1500];
std_var = [1e-2, 5e-2, 1e-1, 5e-1, 1]; %范围
%A=[BS1;BS2;BS3;BS4]; %矩阵A包含4个初始坐标
A = [BS1; BS2; BS3; BS4];
number = 10000;
for j = 1:length(std_var) %循环
    error1 = 0; %初始误差置为0
    error2 = 0; %初始误差置为0
    std_var1 = std_var(j); %令std_var1等于当前数组的值
    for i = 1:number %多次循环
        %r1=A-ones(4,1)*MS;
        r1 = A - ones(4, 1) * MS;
        r2 = (sum(r1.^2, 2)).^(1 / 2); % 目标到观测平台的距离
        %r=r2(2:end,:)-ones(3,1)*r2(1,:)+std_var1*randn(3,1); %表示从[2,i]开始MS与基站i和基站1的距离差
        r = r2(2:end, :) - ones(3, 1) * r2(1, :) + std_var1 * randn(3, 1); % 目标到2，3，4基站与1基站之间的距离差
        sigma = std_var1^2;
        theta1 = TDOACHAN(A, r, sigma); % 调用TDOACHAN函数
        theta2 = TDOATaylor(A, r, sigma); %调用TDOATalor函数
        error1 = error1 + norm(MS-theta1)^2; %移动台MS估计位置与计算的到的距离的平方
        error2 = error2 + norm(MS-theta2)^2; %移动台MS估计位置与计算的到的距离的平方
    end
    RMSE1(j) = (error1 / number)^(1 / 2); %均方根误差
    RMSE2(j) = (error2 / number)^(1 / 2); %均方根误差
end
% plot
semilogx(std_var, RMSE1, '-O', std_var, RMSE2, '-s') % x轴取对数，X轴范围是1e-2到1,Y轴的范围是变动的
xlabel('The standard deviation of measurement noise (m)');
ylabel('RMSE');
legend('TDOA-CHAN', 'TDOA-Taylor');

%% TDOA - CHAN:
function theta = TDOACHAN(A, p, sigma)
% A is the coordinate of BSs
%A是BSS的坐标
% p is the range measurement
%P是范围测量
% sigma is the the variance of TDOA measurement
%sigma是TDOA测量的方差
[m, ~] = size(A); %size得到A的行列数赋值给[m,~]，~表示占位，就是只要行m的值！
k = sum(A.^2, 2); %矩阵A每个元素分别平方，得到新矩阵，再行求和，作为矩阵K (x^2+y^2) 到原点的平方和
G1 = [A(2:end, :) - ones(m-1, 1) * A(1, :), p]; %得到Xm1,Ym1,Rm1,的值，m取值[2,i],构建矩阵Ga
h1 = 1 / 2 * (p.^2 - k(2:end, :) + ones(m-1, 1) * k(1, :)); %构建矩阵h
Q = diag(ones(m-1, 1)*sigma); %构建TDOA的协方差矩阵
% initial estimate
theta0 = inv(G1'*inv(Q)*G1) * G1' * inv(Q) * h1; %通过一次WLS算法进行求解，
s = A(2:end, :) - ones(m-1, 1) * theta0(1:2, :)';
d = sum(s.^2, 2); %矩阵s每个元素分别平方，得到新矩阵，在行求和，最为矩阵d
B1 = diag(d.^(1 / 2));
cov1 = B1 * Q * B1;
% first wls
theta1 = inv(G1'*inv(cov1)*G1) * G1' * inv(cov1) * h1; %进行第一次WLS计算
cov_theta1 = inv(G1'*inv(cov1)*G1); %得到theta1的协方差矩阵
% second wls
G2 = [1, 0; 0, 1; 1, 1]; %构建G'
h2 = [(theta1(1, 1) - A(1, 1))^2; (theta1(2, 1) - A(1, 2))^2; theta1(3, 1)^2]; %构建h'
B2 = diag([theta1(1, 1) - A(1, 1), theta1(2, 1) - A(1, 2), theta1(3, 1)]); %构建b'
cov2 = 4 * B2 * cov_theta1 * B2; %得到误差矢量的协方差矩阵。
theta2 = inv(G2'*inv(cov2)*G2) * G2' * inv(cov2) * h2; %运用最大似然估计得到
theta = theta2.^(1 / 2) + [A(1, 1); A(1, 2)]; %得到MS位置的估计值坐标，以及符号
theta = theta'; %转换为（x,y）形式
end

%% TDOA - Talayor:
function theta = TDOATaylor(A, p, sigma)
% A is the coordinate of BSs
% p is the range measurement
% sigma is the the variance of TOA measurement
% initial estimate
theta0 = TDOACHAN(A, p, sigma); %调用TDOACHAN得到一个初始的估计位置
delta = norm(theta0); %得到范数
while norm(delta) > 1e-2 %得到足够小的值
    [m, ~] = size(A); %size得到A的行列数赋值给[m,~]，~表示占位，就是只要行m的值！
    d = sum((A - ones(m, 1) * theta0).^2, 2);
    R = d.^(1 / 2);
    G1 = ones(m-1, 1) * (A(1, 1) - theta0(1, 1)) / R(1, 1) - (A(2:m, 1) - theta0(1, 1)) ./ R(2:m, :);
    G2 = ones(m-1, 1) * (A(1, 2) - theta0(1, 2)) / R(1, 1) - (A(2:m, 2) - theta0(1, 2)) ./ R(2:m, :);
    G = [G1, G2]; %构建Gt
    h = p - (R(2:m, :) - ones(m-1, 1) * R(1, :)); %构建Ht
    Q = diag(ones(m-1, 1)*sigma); %TDOA测量值的协方差矩阵
    delta = inv(G'*inv(Q)*G) * G' * inv(Q) * h; %加权最小二乘解
    theta0 = theta0 + delta'; %累加
end
theta = theta0;

end

%% 另一个TDOA-Chan算法仿真————xyz三维的
% 参考链接： https://blog.nowcoder.net/n/28a962c491264ff187cf2027961b2511
clc;
clear;
close all;

stations = [0, 0, 0; -20, 20, 30; 20, 20, -40; 20, 10, -20; 20, 30, 40]; %基站位置
c = 1500; %光速

X = -100:100; %生成目标位置
Y = 3 * X .* sin(X);
N = length(X);
Locations = zeros(N, 3);
tds = zeros(N, length(stations)-1);
for i = 1:N
    tds(i, :) = TD(stations*1e3, [X(i), Y(i), 0]*1e3, c);
end

for i = 1:N
    Locations(i, :) = TDOA1(stations*1e3, tds(i, :), c) / 1e3;
end

figure(1);
plot(X, Y, 'r*');
hold on;
plot(Locations(:, 1), Locations(:, 2), 'b--o');
legend('真实位置', '定位位置');
xlabel('Km');
ylabel('Km');

% 根据目标位置，生成观测样本（时差信息）
function [tds] = TD(stations, T, c)
N = length(stations) - 1;
tds = zeros(1, N);
rs = zeros(1, N);
r0 = sqrt((T(1) - stations(1, 1))^2+(T(2) - stations(1, 2))^2+(T(3) - stations(1, 3))^2);
for i = 1:N
    rs(i) = sqrt((T(1) - stations(i+1, 1))^2+(T(2) - stations(i+1, 2))^2+(T(3) - stations(i+1, 3))^2);
    tds(1, i) = (rs(i) - r0) / c;
end
end

% 根据时差，基站位置反推目标位置，即Chan算法的核心实现
function [location] = TDOA1(stations, tds, c)
N = length(stations) - 1;
%%%%%%%%%%%%%%%
A = zeros(N-1, 3);
for i = 1:N
    A(i, 1) = stations(i+1, 1) - stations(1, 1);
    A(i, 2) = stations(i+1, 2) - stations(1, 2);
    A(i, 3) = stations(i+1, 3) - stations(1, 3);
end
%%%%%%%%%%%%%%%%%%%
B1 = (-tds * c)'; %[N-1,1],delta r
D1 = zeros(N, 1);
for i = 1:N
    D1(i, 1) = stations(i+1, 1)^2 + stations(i+1, 2)^2 + stations(i+1, 3)^2;
end
L = (1 / 2) * (D1 - (stations(1, 1)^2 + stations(1, 2)^2 + stations(1, 2)^2) - (B1.^2));

A2 = A(1:3, :);
L2 = L(1:3, :);
B2 = B1(1:3, :);

a = det(A2);
a1 = (det([B2, A2(:, 2:3)])) / a;
a2 = (det([A2(:, 1), B2, A2(:, 3)])) / a;
a3 = (det([A2(:, 1:2), B2])) / a;
b1 = (det([L2, A2(:, 2:3)])) / a;
b2 = (det([A2(:, 1), L2, A2(:, 3)])) / a;
b3 = (det([A2(:, 1:2), L2])) / a;

D = a1^2 + a2^2 + a3^2 - 1; %A
c1 = (b1 - stations(1, 1));
c2 = (b2 - stations(1, 2));
c3 = (b3 - stations(1, 3));
E = a1 * c1 + a2 * c2 + a3 * c3; %B
F = c1^2 + c2^2 + c3^2; %C

G = sqrt(E^2-D*F);

r01 = (-E + G) / D;
r02 = (-E - G) / D;
location1 = [a1 * r01 + b1; a2 * r01 + b2; a3 * r01 + b3];
location2 = [a1 * r02 + b1; a2 * r02 + b2; a3 * r02 + b3];

tdsex1 = TD(stations, location1, c);
tdsex2 = TD(stations, location2, c);

if (abs(tdsex1(4)-tds(4)) < 1e-8) % 引入时差信息，剔除伪解
    location = location1;
else
    location = location2;
end
end

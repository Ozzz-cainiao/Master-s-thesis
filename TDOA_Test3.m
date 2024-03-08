%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA_Test3.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-01
% 描述: 测试两个TDOA算法，对于固定点是否可用
% 输入:  
% 输出:  
%**************************************************************************

%% 
clc
close all
clear
% 目标位置（单位：米）
target_position = [200, 300];

% 信号传播速度（假设单位为 m/s）
v = 1500; % 声速

% 接收器的位置（假设单位为米）
node = [
    0, 0; 
    1e3, 0; 
    0, 1e3;
    1e3, 1e3 
];

% 计算目标到各接收器的距离
d = zeros(size(node, 1), 1);
t = zeros(size(node, 1), 1);
a = zeros(size(node, 1), 1);
for i = 1 : size(node, 1)
    d(i) = norm(target_position - node(i,:));
    t(i) = d(i) / v;
    relatiPos = target_position - node(i, :);
    a(i) = atan2d(relatiPos(2), relatiPos(1));
end
% [res, ~] = TA1(t(1:2), a(1: 2)', node(1: 2, :));
% res = TDOACHAN(node, t, 0);
nodeT = [node, zeros(4, 1)];
[res, ~] = TDOA(t', nodeT, 4);
n = 10;
%% TDOA - CHAN:
function res = TDOACHAN(A, p, sigma)
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
res0 = inv(G1'*inv(Q)*G1) * G1' * inv(Q) * h1; %通过一次WLS算法进行求解，
s = A(2:end, :) - ones(m-1, 1) * res0(1:2, :)';
d = sum(s.^2, 2); %矩阵s每个元素分别平方，得到新矩阵，在行求和，最为矩阵d
B1 = diag(d.^(1 / 2));
cov1 = B1 * Q * B1;
% first wls
res1 = inv(G1'*inv(cov1)*G1) * G1' * inv(cov1) * h1; %进行第一次WLS计算
cov_theta1 = inv(G1'*inv(cov1)*G1); %得到theta1的协方差矩阵
% second wls
G2 = [1, 0; 0, 1; 1, 1]; %构建G'
h2 = [(res1(1, 1) - A(1, 1))^2; (res1(2, 1) - A(1, 2))^2; res1(3, 1)^2]; %构建h'
B2 = diag([res1(1, 1) - A(1, 1), res1(2, 1) - A(1, 2), res1(3, 1)]); %构建b'
cov2 = 4 * B2 * cov_theta1 * B2; %得到误差矢量的协方差矩阵。
res2 = inv(G2'*inv(cov2)*G2) * G2' * inv(cov2) * h2; %运用最大似然估计得到
res = res2.^(1 / 2) + [A(1, 1); A(1, 2)]; %得到MS位置的估计值坐标，以及符号
res = res'; %转换为（x,y）形式
end

%% TDOA - Talayor:
function res = TDOATaylor(A, p, sigma)
% A is the coordinate of BSs
% p is the range measurement
% sigma is the the variance of TOA measurement
% initial estimate
res0 = TDOACHAN(A, p, sigma); %调用TDOACHAN得到一个初始的估计位置
delta = norm(res0); %得到范数
while norm(delta) > 1e-2 %得到足够小的值
    [m, ~] = size(A); %size得到A的行列数赋值给[m,~]，~表示占位，就是只要行m的值！
    d = sum((A - ones(m, 1) * res0).^2, 2);
    R = d.^(1 / 2);
    G1 = ones(m-1, 1) * (A(1, 1) - res0(1, 1)) / R(1, 1) - (A(2:m, 1) - res0(1, 1)) ./ R(2:m, :);
    G2 = ones(m-1, 1) * (A(1, 2) - res0(1, 2)) / R(1, 1) - (A(2:m, 2) - res0(1, 2)) ./ R(2:m, :);
    G = [G1, G2]; %构建Gt
    h = p - (R(2:m, :) - ones(m-1, 1) * R(1, :)); %构建Ht
    Q = diag(ones(m-1, 1)*sigma); %TDOA测量值的协方差矩阵
    delta = inv(G'*inv(Q)*G) * G' * inv(Q) * h; %加权最小二乘解
    res0 = res0 + delta'; %累加
end
res = res0;

end

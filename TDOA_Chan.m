%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA_Chan.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: TDOA-Chan算法仿真，根据时差，基站位置反推目标位置，即Chan算法的核心实现
%       参考https://blog.nowcoder.net/n/28a962c491264ff187cf2027961b2511
% 输入: stations：n * 3; 
%       tds 1 * n; 
%       c：声速
% 输出:
%**************************************************************************

%%
function [location] = TDOA_Chan(stations, tds, c)
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

%% 根据目标位置，生成观测样本（时差信息）
%       n 为平台数
% INPUT：
%       stations n * 3; 
%       T: n * 1; 
%       c: soundspeed
% OUTPUT: 
%       tds: 1 * n
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
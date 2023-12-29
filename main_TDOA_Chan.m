%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\main_TDOA_Chan.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: TDOA-Chan算法仿真
%       参考https://blog.nowcoder.net/n/28a962c491264ff187cf2027961b2511
% 输入:
% 输出:
%**************************************************************************

clc;
clear;
close all;

stations = [0, 0, 0; ...
            -20, 20, 30; ...
            20, 20, -40; ...
            20, 10, -20; ...
            20, 30, 40]; %基站位置
c = 1500;

X = -100:100; %生成目标位置
Y = 3 * X .* sin(X);
N = length(X);
Locations = zeros(N, 3);
tds = zeros(N, length(stations)-1);
for i = 1:N
    tds(i, :) = TD(stations*1e3, [X(i), Y(i), 0]*1e3, c);
end

for i = 1:N
    Locations(i, :) = TDOA_Chan(stations*1e3, tds(i, :), c) / 1e3;
end

figure(1);
plot(X, Y, 'r*');
hold on;
plot(Locations(:, 1), Locations(:, 2), 'b--o');
legend('真实位置', '定位位置');
xlabel('Km');
ylabel('Km');

%% 根据目标位置，生成观测样本（时差信息）
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


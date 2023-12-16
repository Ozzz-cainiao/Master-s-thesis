%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA\TDOA_ZCB.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-16
% 描述: TDOA 三平台误差分析 ZCB 
% 输入:  
% 输出:  
%**************************************************************************

%% 三平台
clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c
f1 = c * (t2 - t1 - (n2 - n1) * ts) - sqrt((x2 - xs)^2+(y2 - ys)^2) + sqrt((x1 - xs)^2+(y1 - ys)^2);
f2 = c * (t3 - t2 - (n3 - n2) * ts) - sqrt((x3 - xs)^2+(y3 - ys)^2) + sqrt((x2 - xs)^2+(y2 - ys)^2);

A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys)];
A_Mt = [diff(f1, t1), diff(f1, t2), diff(f1, t3); ...
    diff(f2, t1), diff(f2, t2), diff(f2, t3)];
A_Mc = [diff(f1, c); diff(f2, c)];
A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1)];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2)];
A_M3 = [diff(f1, x3), diff(f1, y3); diff(f2, x3), diff(f2, y3)];

M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Mc = matlabFunction(A_Mc);
M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
M3 = matlabFunction(A_M3);
clearvars -except M Mt Mc M1 M2 M3

% x = 0:10:2000;
% y = (0:10:2000)';
x = -1000:5:1000;
y = (-1000:5:1000)';
% [x1, y1, x2, y2, x3, y3] = deal(300, 300, 1700, 300, 1000, 1700); %% 平台位置
[x1, y1, x2, y2, x3, y3] = deal(-500, 500, 0, -500, 500, 500); %% 平台位置

[lenx, leny] = deal(length(x), length(y));

errornor = [2^2, 2^2, 0.001^2,5^2]; % x y ts^2  c^2误差  假设只有测时误差  已经是平方误差了
errorsum = zeros(lenx, leny);
c = 1500;
for ii = 1:lenx
    for jj = 1:leny
        m1 = M1(x1, x(ii), y1, y(jj));
        m2 = M2(x2, x(ii), y2, y(jj));
        m3 = M3(x3, x(ii), y3, y(jj));
        m = M(x1, x2, x3, x(ii), y1, y2, y3, y(jj));
        mt = Mt(c);
        error = (m' * m) \ m' * (m1 * diag([errornor(1), errornor(2)]) * m1' ...
            + m2 * diag([errornor(1), errornor(2)]) * m2' ...
            + m3 * diag([errornor(1), errornor(2)]) * m3' ...
            + mt * diag([errornor(3), errornor(3), errornor(3)]) * mt') * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('TDOA平面角解算误差');
clim([0, 50]); colormap jet
shading interp;view(0,90)

% 使用nanmean计算非NaN值的平均值
averageValue = nanmean(errorsum, 'all');
disp(averageValue);
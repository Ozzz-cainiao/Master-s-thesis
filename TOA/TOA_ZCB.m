%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA\TOA_ZCB.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-16
% 描述: 臧传斌 论文  TOA
% 输入:  
% 输出:  
%**************************************************************************

%% 双平台
clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c ta
% f1 = c * (t1 - n1 * ts - ta) - sqrt((x1 - xs)^2+(y1 - ys)^2);
% f2 = c * (t2 - n2 * ts - ta) - sqrt((x2 - xs)^2+(y2 - ys)^2);

f1 = c * (t1 - ta) - sqrt((x1 - xs)^2+(y1 - ys)^2);
f2 = c * (t2 - ta) - sqrt((x2 - xs)^2+(y2 - ys)^2);

A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1)];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2)];
A_Mc = [diff(f1, c); diff(f2, c)];
A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys)];
A_Mt = [diff(f1, t1), diff(f1, t2); diff(f2, t1), diff(f2, t2)];

M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
Mc = matlabFunction(A_Mc);
M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);

clearvars -except M Mt M1 M2 Mc

x = 0:10:2000;
y = (0:10:2000)';

% [x1, y1, x2, y2] = deal(300, 1000, 1700, 1000); %% 平台位置
[x1, y1, x2, y2] = deal(500, 1000, 1500, 1000); %% 平台位置

[lenx, leny] = deal(length(x), length(y));
% errornor = [0,0,0.01,0];
% errornor = [0, 0, 0, 100];
% errornor = [0,0,0.001];
% errornor = [0.01,0.01,0];
% errornor = [2^2,2^2,0.001^2,0];      % dx^2 dy^2 t1^2 c^2 误差
% errornor = [2^2,2^2,0.001^2,1.5^2];      % dx^2 dy^2 t1^2 c^2 误差
errornor = [2^2, 2^2, 0.002^2, 1.5^2]; % dx^2 dy^2 t1^2 c^2 误差

errorsum = zeros(lenx, leny);
c = 1500;
[n1, n2, t1, t2, ta, ts] = deal(1, 1, 2, 2, 0, 1);
for ii = 1:lenx
    for jj = 1:leny
        m1 = M1(x1, x(ii), y1, y(jj));
        m2 = M2(x2, x(ii), y2, y(jj));
%         mc = Mc(n1, n2, t1, t2, ta, ts);
        mc = Mc(t1, t2, ta);

        m = M(x1, x2, x(ii), y1, y2, y(jj));
        mt = Mt(c);
        %         error = (m' * m) \ m' * (m1 * diag([errornor(1), errornor(2)]) * m1' ...
        %             +m2 * diag([errornor(1), errornor(2)]) * m2' ...
        %             +mt * diag([errornor(3), errornor(3)]) * mt' ...
        %             +mc * diag([errornor(4)]) * mc' ...
        %             ) * m / (m' * m);

        error = (m' * m) \ m' * (m1 * diag([errornor(1), errornor(2)]) * m1' ...
            +m2 * diag([errornor(1), errornor(2)]) * m2' ...
            +mt * diag([errornor(3), errornor(3)]) * mt' ...
            +mc * diag([errornor(4)]) * mc' ...
            ) * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('TOA平面角解算误差');
clim([0, 50]); colormap jet
shading interp;view(0,90)
mean2(abs(errorsum(1:100, :))) % 除去基线行的平均测量误差
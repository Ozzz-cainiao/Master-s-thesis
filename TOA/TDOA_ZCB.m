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
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c z1 z2 z3 z4 zs

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

x = 0:2:2000;
y = (0:2:2000)';
[x1, y1, x2, y2, x3, y3] = deal(300, 300, 1700, 300, 1000, 1700); %% 平台位置
[lenx, leny] = deal(length(x), length(y));

% errornor = [2^2, 2^2, 0.001^2, 1.5^2]; % x y ts^2  c^2误差  假设只有测时误差  已经是平方误差了
% errornor = [2^2, 2^2, 0.003^2, 1.5^2]; % x y ts^2  c^2误差  假设只有测时误差  已经是平方误差了
errornor = [2^2, 2^2, 0.001^2, 3^2]; % x y ts^2  c^2误差  假设只有测时误差  已经是平方误差了

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
            +m2 * diag([errornor(1), errornor(2)]) * m2' ...
            +m3 * diag([errornor(1), errornor(2)]) * m3' ...
            +mt * diag([errornor(3), errornor(3), errornor(3)]) * mt') * m / (m' * m);
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

averageValue = nanmean(errorsum, 'all');
fprintf('%.2f\n', averageValue); % 显示2位小数

%% 加上z
clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c z1 z2 z3  zs
% f1 = c * (t2 - t1 - (n2 - n1) * ts) - sqrt((x2 - xs)^2+(y2 - ys)^2+(z2 - zs)^2) + sqrt((x1 - xs)^2+(y1 - ys)^2+(z1 - zs)^2);
% f2 = c * (t3 - t2 - (n3 - n2) * ts) - sqrt((x3 - xs)^2+(y3 - ys)^2+(z3 - zs)^2) + sqrt((x2 - xs)^2+(y2 - ys)^2+(z2 - zs)^2);
f1 = c * (t2 - t1) - sqrt((x2 - xs)^2+(y2 - ys)^2+(z2 - zs)^2) + sqrt((x1 - xs)^2+(y1 - ys)^2+(z1 - zs)^2);
f2 = c * (t3 - t1) - sqrt((x3 - xs)^2+(y3 - ys)^2+(z3 - zs)^2) + sqrt((x1 - xs)^2+(y1 - ys)^2+(z1 - zs)^2);
A_M = [diff(f1, xs), diff(f1, ys), diff(f1, zs); diff(f2, xs), diff(f2, ys), diff(f2, zs)];
A_Mt = [diff(f1, t1), diff(f1, t2), diff(f1, t3); ...
    diff(f2, t1), diff(f2, t2), diff(f2, t3)];
A_Mc = [diff(f1, c); diff(f2, c)];
A_M1 = [diff(f1, x1), diff(f1, y1), diff(f1, z1); diff(f2, x1), diff(f2, y1), diff(f2, z1)];
A_M2 = [diff(f1, x2), diff(f1, y2), diff(f1, z2); diff(f2, x2), diff(f2, y2), diff(f2, z2)];
A_M3 = [diff(f1, x3), diff(f1, y3), diff(f1, z3); diff(f2, x3), diff(f2, y3), diff(f2, z3)];
M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Mc = matlabFunction(A_Mc);
M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
M3 = matlabFunction(A_M3);
clearvars -except M Mt Mc M1 M2 M3

x = 0:20:2000;
y = (0:20:2000)';
[x1, y1, x2, y2, x3, y3, z1, z2, z3] = deal(300, 300, 1700, 300, 1000, 1700, 0, 0, 0); %% 平台位置

[lenx, leny] = deal(length(x), length(y));
errornor = [2^2, 2^2, 1^2, 0.001^2, 1.5^2]; % x y z ts^2  c^2误差  假设只有测时误差  已经是平方误差了
errorsum = zeros(lenx, leny);
c = 1500;
zs = 0; % 已知目标深度
[n1, n2, n3, ts] = deal(0, 0, 0, 0);
for ii = 1:lenx
    for jj = 1:leny
        t1 = sqrt((x1 - x(ii))^2+(y1 - y(jj))^2+(z1 - zs)^2) / c;
        t2 = sqrt((x2 - x(ii))^2+(y2 - y(jj))^2+(z2 - zs)^2) / c;
        t3 = sqrt((x3 - x(ii))^2+(y3 - y(jj))^2+(z3 - zs)^2) / c;
        m1 = M1(x1, x(ii), y1, y(jj), z1, zs);
        m2 = M2(x2, x(ii), y2, y(jj), z2, zs);
        m3 = M3(x3, x(ii), y3, y(jj), z3, zs);
        m = M(x1, x2, x3, x(ii), y1, y2, y3, y(jj), z1, z2, z3, zs);
        mt = Mt(c);
%         mc = Mc(n1, n2, n3, t1, t2, t3, ts);
        mc = Mc(t1, t2, t3);

        error = (m' * m) \ m' * (m1 * diag([errornor(1), errornor(2), errornor(3)]) * m1' ...
            +m2 * diag([errornor(1), errornor(2), errornor(3)]) * m2' ...
            +m3 * diag([errornor(1), errornor(2), errornor(3)]) * m3' ...
            +mt * diag([errornor(4), errornor(4), errornor(4)]) * mt' ...
            +mc * errornor(5) * mc') * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y', abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('TDOA平面角解算误差');
clim([0, 50]); colormap jet
shading interp;view(0,90)

averageValue = nanmean(errorsum, 'all');
fprintf('%.2f\n', averageValue); % 显示2位小数
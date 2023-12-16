%% 5平台纯方位定位误差推导 带测量平台误差
%
clc
clear
close all

%% 求出解析解
syms x1 y1 x2 y2 x3 y3 x4 y4 x5 y5 xs ys theta1 theta2 theta3 theta4 theta5
f1 = xs * cos(theta1) - ys * sin(theta1) - x1 * cos(theta1) + y1 * sin(theta1);
f2 = xs * cos(theta2) - ys * sin(theta2) - x2 * cos(theta2) + y2 * sin(theta2);
f3 = xs * cos(theta3) - ys * sin(theta3) - x3 * cos(theta3) + y3 * sin(theta3);
f4 = xs * cos(theta4) - ys * sin(theta4) - x4 * cos(theta4) + y4 * sin(theta4);
f5 = xs * cos(theta5) - ys * sin(theta5) - x5 * cos(theta5) + y5 * sin(theta5);
% 求出解析解
A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1); diff(f3, x1), diff(f3, y1); diff(f4, x1), diff(f4, y1); diff(f5, x1), diff(f5, y1)];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2); diff(f3, x2), diff(f3, y2); diff(f4, x2), diff(f4, y2); diff(f5, x2), diff(f5, y2)];
A_M3 = [diff(f1, x3), diff(f1, y3); diff(f2, x3), diff(f2, y3); diff(f3, x3), diff(f3, y3); diff(f4, x3), diff(f4, y3); diff(f5, x3), diff(f5, y3)];
A_M4 = [diff(f1, x4), diff(f1, y4); diff(f2, x4), diff(f2, y4); diff(f3, x4), diff(f3, y4); diff(f4, x4), diff(f4, y4); diff(f5, x4), diff(f5, y4)];
A_M5 = [diff(f1, x5), diff(f1, y5); diff(f2, x5), diff(f2, y5); diff(f3, x5), diff(f3, y5); diff(f4, x5), diff(f4, y5); diff(f5, x5), diff(f5, y5)];
A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys); diff(f3, xs), diff(f3, ys); diff(f4, xs), diff(f4, ys); diff(f5, xs), diff(f5, ys)];
A_Mtheta = [diff(f1, theta1), diff(f1, theta2), diff(f1, theta3), diff(f1, theta4), diff(f1, theta5); ...
    diff(f2, theta1), diff(f2, theta2), diff(f2, theta3), diff(f2, theta4), diff(f2, theta5); ...
    diff(f3, theta1), diff(f3, theta2), diff(f3, theta3), diff(f3, theta4), diff(f3, theta5); ...
    diff(f4, theta1), diff(f4, theta2), diff(f4, theta3), diff(f4, theta4), diff(f4, theta5); ...
    diff(f5, theta1), diff(f5, theta2), diff(f5, theta3), diff(f5, theta4), diff(f5, theta5)];

M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
M3 = matlabFunction(A_M3);
M4 = matlabFunction(A_M4);
M5 = matlabFunction(A_M5);
M = matlabFunction(A_M);
Mtheta = matlabFunction(A_Mtheta);
clearvars -except M1 M2 M3 M4 M5 M Mtheta


x = 0:10:2000;
y = (0:10:2000)';

[x1, y1, x2, y2, x3, y3, x4, y4, x5, y5] = deal(0, 0, 2000, 0, 2000, 2000, 0, 2000, 1000, 1000);

[lenx, leny] = deal(length(x), length(y));

errornor = [0, 0, (1.5 / 180 * pi)^2]; % 横 纵 角均方根误差
theta1 = atan((x1 - x)./(y1 - y));
theta2 = atan((x2 - x)./(y2 - y));
theta3 = atan((x3 - x)./(y3 - y));
theta4 = atan((x4 - x)./(y4 - y));
theta5 = atan((x5 - x)./(y5 - y));
errorsum = zeros(lenx, leny);
for ii = 1:lenx
    for jj = 1:leny
        m1 = M1(theta1(jj, ii));
        m2 = M2(theta2(jj, ii));
        m3 = M3(theta3(jj, ii));
        m4 = M4(theta4(jj, ii));
        m5 = M5(theta5(jj, ii));
        m = M(theta1(jj, ii), theta2(jj, ii), theta3(jj, ii), theta4(jj, ii), theta5(jj, ii));
        mtheta = Mtheta(theta1(jj, ii), theta2(jj, ii), theta3(jj, ii), theta4(jj, ii), theta5(jj, ii), x1, x2, x3, x4, x5, x(ii), y1, y2, y3, y4, y5, y(jj));
        %         error = (m' * m) \ m' * ((m1 * diag([errornor(1), errornor(2)]) * m1' ...
        %             +m2 * diag([errornor(1), errornor(2)]) * m2' ...
        %             +m3 * diag([errornor(1), errornor(2)]) * m3' ...
        %             +mtheta * diag([errornor(3), errornor(3), errornor(3)]) * mtheta') * m / (m' * m));
        error = (m' * m) \ m' * ((m1 * diag([errornor(1), errornor(2)]) * m1' ...
            + m2 * diag([errornor(1), errornor(2)]) * m2' ...
            + m3 * diag([errornor(1), errornor(2)]) * m3' ...
            + m4 * diag([errornor(1), errornor(2)]) * m4' ...
            + m5 * diag([errornor(1), errornor(2)]) * m5' ...
            + mtheta * diag([errornor(3), errornor(3), errornor(3), errornor(3), errornor(3)]) * mtheta') * m / (m' * m));
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('平面角解算误差');
caxis([0, 50]); colormap jet
shading interp;view(0,90)
mean2(abs(errorsum(1:100, :)))

%

%% MJ硕士论文图2.6  ZLM修改完善
%
clc
clear
close all

%% 求出解析解
syms x1 y1 x2 y2  x y alpha1 alpha2
f1 = (y - y1) / (x - x1) - tan(alpha1);
f2 = (y - y2) / (x - x2) - tan(alpha2);
s = solve(f1, f2, x, y); %已知2方程 求解2未知数
x = s.x;
y = s.y;
% 求出解析解
syms dalpha1 dalpha2
dx = diff(x, alpha1) * dalpha1 + diff(x, alpha2) * dalpha2;
dy = diff(y, alpha1) * dalpha1 + diff(y, alpha2) * dalpha2;
DX = matlabFunction(dx);
DY = matlabFunction(dy);

clearvars -except DX DY

x = 0:10:2000;
y = (0:10:2000)';

[x1, y1, x2, y2] = deal(300, 1000, 1700, 1000);

[lenx, leny] = deal(length(x), length(y));

[dalpha1, dalpha2] = deal((0.2 / 180 * pi)^2);

alpha1 = atan((y1 - y)./(x1 - x));
alpha2 = atan((y2 - y)./(x2 - x));
errorsum = zeros(lenx, leny);
for ii = 1:lenx
    for jj = 1:leny
        errorx1 = DX(alpha1(jj, ii), alpha2(jj, ii), dalpha1, dalpha2, x1, x2, y1, y2) * sqrt((x(ii) - x1)^2+(y(jj) - y1)^2);
        errorx2 = DX(alpha1(jj, ii), alpha2(jj, ii), dalpha1, dalpha2, x1, x2, y1, y2) * sqrt((x(ii) - x2)^2+(y(jj) - y2)^2);
        errory1 = DY(alpha1(jj, ii), alpha2(jj, ii), dalpha1, dalpha2, x1, x2, y1, y2) * sqrt((x(ii) - x1)^2+(y(jj) - y1)^2);
        errory2 = DY(alpha1(jj, ii), alpha2(jj, ii), dalpha1, dalpha2, x1, x2, y1, y2) * sqrt((x(ii) - x2)^2+(y(jj) - y2)^2);
        errorsum(jj, ii) = sqrt(0.5*(errorx1^2 + errorx2^2 + errory1^2 + errory2^2));
    end
end

errorsum = fillmissing(errorsum, 'movmedian', 5);
figure
surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('平面角解算误差');
caxis([0, 100]); colormap jet
shading interp;view(0,90)

mean2(abs(errorsum(1:100, :)))

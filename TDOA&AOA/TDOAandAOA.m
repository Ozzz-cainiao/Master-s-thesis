%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA&AOA\TDOAandAOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-16
% 描述: 时延差/方位联合定位  误差分析函数
% 输入:
% 输出:
%**************************************************************************
%% 两个平台，两个角参与解算
clc
clear
close all
tic;
syms x1 y1 x2 y2 xs ys c t1 t2 ts theta1 theta2
% theta1 角目标-平台1-平台2
f1 = (x1 - xs)^2 + (y1 - ys)^2 - c^2 * (t1 - ts)^2;
f2 = (x2 - xs)^2 + (y2 - ys)^2 - c^2 * (t2 - ts)^2;
f3 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2 ...
    -2 * sqrt((x1 - xs)^2+(y1 - ys)^2) * sqrt((x1 - x2)^2+(y1 - y2)^2) * cos(theta1));
f4 = (x1 - xs)^2 + (y1 - ys)^2 - ((x2 - xs)^2 + (y2 - ys)^2 + (x2 - x1)^2 + (y2 - y1)^2 ...
    -2 * sqrt((x2 - xs)^2+(y2 - ys)^2) * sqrt((x2 - x1)^2+(y2 - y1)^2) * cos(theta2));

% 对上式求全微分
A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1); 
    diff(f3, x1), diff(f3, y1);diff(f4, x1), diff(f4, y1);];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2); 
    diff(f3, x2), diff(f3, y2);diff(f4, x2), diff(f4, y2); ];
A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys);
    diff(f3, xs), diff(f3, ys); diff(f4, xs), diff(f4, ys);];
A_Mt = [diff(f1, t1), diff(f1, t2); diff(f2, t1), diff(f2, t2);
    diff(f3, t1), diff(f3, t2); diff(f4, t1), diff(f4, t2);];
A_Mc = [diff(f1, c); diff(f2, c); diff(f3, c); diff(f4, c)];
A_Ma = [diff(f1, theta1), diff(f1, theta2); diff(f2, theta1), diff(f2, theta2);
    diff(f3, theta1), diff(f3, theta2); diff(f4, theta1), diff(f4, theta2);];


M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
Mc = matlabFunction(A_Mc);
M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Ma = matlabFunction(A_Ma);
clearvars -except M Mt M1 M2 Mc Ma

x = 0:2:2000;
y = (0:2:2000)';
[x1, y1, x2, y2] = deal(500, 0, 1500, 0); %% 平台位置
[lenx, leny] = deal(length(x), length(y));
% errornor = [2^2, 2^2, 0.001^2, 1.5^2, (0.5 / 180 * pi)^2, (0.5 / 180 * pi)^2]; % dx^2 dy^2 t^2 c^2 theta误差
% errornor = [2^2, 2^2, 0.001^2, 1.5^2, (1 / 180 * pi)^2, (1 / 180 * pi)^2]; % dx^2 dy^2 t^2 c^2 theta误差

errornor = [2^2, 2^2, 0.003^2, 1.5^2, (0.5 / 180 * pi)^2, (0.5 / 180 * pi)^2]; % dx^2 dy^2 t^2 c^2 theta误差

errorsum = zeros(lenx, leny);
c = 1500;
ts = 0;
theta1 = zeros(lenx, leny);
for ii = 1:lenx
    for jj = 1:leny
        % theta 是角目标-平台1-平台2
        % theta = 角-基线和水平夹角
        theta1(ii, jj) = atan2(y(jj) - y1 ,x(ii) - x1) - atan2((y2 - y1),(x2 - x1)); % 四象限反正切
        theta2(ii, jj) = atan2(y(jj) - y2 ,x(ii) - x2) - atan2((y1 - y2),(x1 - x2)); % 四象限反正切
    end
end
for ii = 1:lenx
    for jj = 1:leny
        t1 = sqrt((x(ii) - x1)^2+(y(jj) - y1)^2) / c;
        t2 = sqrt((x(ii) - x2)^2+(y(jj) - y2)^2) / c;
        m1 = M1(theta1(ii, jj), theta2(ii, jj),x1, x2, x(ii), y1, y2, y(jj));
        m2 = M2(theta1(ii, jj), theta2(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        mc = Mc(c, t1, t2, ts);
        m = M(theta1(ii, jj), theta2(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        mt = Mt(c, t1, t2, ts);
        ma = Ma(theta1(ii, jj), theta2(ii, jj), x1, x2, x(ii), y1, y2, y(jj));

        ttt = (m1 * diag([errornor(1), errornor(2)]) * m1' ...
            + m2 * diag([errornor(1), errornor(2)]) * m2' ...
            + mt * diag([errornor(3), errornor(3)]) * mt' ...
            + mc * diag([errornor(4)]) * mc' ...
            + ma * diag([errornor(5), errornor(6)]) * ma');
        error = (m' * m) \ m' * ttt * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

limi = 20;
% 绘制填充的等高线图
errorsum(errorsum > limi) = limi;
figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 6]); % 左下宽高
contourf(x, y, abs(errorsum), 5 ,'ShowText','on'); % 这里的20是等高线的数量
% figure
% surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
% colorbar;
xlabel('x/m');
ylabel('y/m');
% title('TOA-AOA平面角解算误差');
% clim([0, 15]); 
colormap jet
shading interp;
% view(0,90)
average_value = nanmean(errorsum(:));
fprintf('%.2f\n', average_value); % 显示2位小数
% figure
% surf(x, y, abs(errorsum))
% set(gca, 'YDir', 'normal');
% colorbar;
% xlabel('x/m');
% ylabel('y/m');
% title('TOA-AOA平面角解算误差');
% clim([0, 10]); colormap jet
% shading interp;view(0,90)
% average_value = nanmean(errorsum(:));
% fprintf('%.2f\n', average_value); % 显示2位小数

%% 两个平台一个角参与解算
clc
clear
close all

syms x1 y1 x2 y2 xs ys c t1 t2 ts theta1 theta2
% theta1 角目标-平台1-平台2
f1 = sqrt((x1 - xs)^2 + (y1 - ys)^2) - sqrt((x2 - xs)^2 + (y2 - ys)^2) - c * (t1 - t2);
f2 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2 ...
    -2 * sqrt((x1 - xs)^2+(y1 - ys)^2) * sqrt((x1 - x2)^2+(y1 - y2)^2) * cos(theta1));


% 对上式求全微分
A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1); ];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2); ];
A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys); ];
A_Mt = [diff(f1, t1), diff(f1, t2); diff(f2, t1), diff(f2, t2);];
A_Mc = [diff(f1, c); diff(f2, c); ];
A_Ma = [diff(f1, theta1); diff(f2, theta1);];

M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
Mc = matlabFunction(A_Mc);
M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Ma = matlabFunction(A_Ma);
clearvars -except M Mt M1 M2 Mc Ma

x = 0:5:2000;
y = (0:5:2000)';
[x1, y1, x2, y2] = deal(500, 1000, 1500, 1000); %% 平台位置
% [x1, y1, x2, y2] = deal(0, 1000, 2000, 1000); %% 平台位置

[lenx, leny] = deal(length(x), length(y));
% errornor = [2^2, 2^2, 0.002^2, 1.5^2, 1^2]; % dx^2 dy^2 t^2 c^2 theta误差
errornor = [0^2, 0^2, 0.001^2, 1.5^2, (1 / 180 * pi)^2]; % dx^2 dy^2 t^2 c^2 theta误差

errorsum = zeros(lenx, leny);
c = 1500;
theta1 = zeros(lenx, leny);
for ii = 1:lenx
    for jj = 1:leny
        % theta 是角目标-平台1-平台2
        % theta = 角-基线和水平夹角
%         theta1(ii, jj) = atan(y(jj)/x(ii)) - atan((y2 - y1)/(x2 - x1)); %
%         反正切 弧度
        theta1(ii, jj) = atan2(y(jj) - y1 ,x(ii) - x1) - atan2((y2 - y1),(x2 - x1)); % 四象限反正切

    end
end
for ii = 1:lenx
    for jj = 1:leny
        t1 = sqrt((x(ii) - x1)^2+(y(jj) - y1)^2) / c;
        t2 = sqrt((x(ii) - x2)^2+(y(jj) - y2)^2) / c;
        m1 = M1(theta1(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        m2 = M2(theta1(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        mc = Mc(t1, t2);
        m = M(theta1(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        mt = Mt(c);
        ma = Ma(theta1(ii, jj), x1, x2, x(ii), y1, y2, y(jj));
        %         error = (m' * m) \ m' * (m1 * diag([errornor(1), errornor(2)]) * m1' ...
        %             +m2 * diag([errornor(1), errornor(2)]) * m2' ...
        %             +mt * diag([errornor(3), errornor(3)]) * mt' ...
        %             +mc * diag([errornor(4)]) * mc' ...
        %             ) * m / (m' * m);

        ttt = (m1 * diag([errornor(1), errornor(2)]) * m1' ...
            + m2 * diag([errornor(1), errornor(2)]) * m2' ...
            + mt * diag([errornor(3), errornor(3)]) * mt' ...
            + mc * diag([errornor(4)]) * mc' ...
            + ma * diag([errornor(5)]) * ma');
        error = (m' * m) \ m' * ttt * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end
limi = 50;
% 绘制填充的等高线图
errorsum(errorsum > limi) = limi;
figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 6]); % 左下宽高
contourf(x, y, abs(errorsum), 5 ,'ShowText','on'); % 这里的20是等高线的数量
% figure
% surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
% colorbar;
xlabel('x/m');
ylabel('y/m');
% title('TOA-AOA平面角解算误差');
% clim([0, 15]); 
colormap jet
shading interp;
% view(0,90)
average_value = nanmean(errorsum(:));
fprintf('%.2f\n', average_value); % 显示2位小数


% %% 两个平台，1个角参与解算 只用theta1
% clc
% clear
% close all
% 
% syms x1 y1 x2 y2 xs ys c t1 t2 ts theta1 theta2
% % theta1 角目标-平台1-平台2
% f1 = (x1 - xs)^2 + (y1 - ys)^2 - c^2 * (t1 - ts)^2;
% f2 = (x2 - xs)^2 + (y2 - ys)^2 - c^2 * (t2 - ts)^2;
% f3 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2 ...
%     -2 * sqrt((x1 - xs)^2+(y1 - ys)^2) * sqrt((x1 - x2)^2+(y1 - y2)^2) * cos(theta1));
% % f4 = (x1 - xs)^2 + (y1 - ys)^2 - ((x2 - xs)^2 + (y2 - ys)^2 + (x2 - x1)^2 + (y2 - y1)^2 ...
% %     -2 * sqrt((x2 - xs)^2+(y2 - ys)^2) * sqrt((x2 - x1)^2+(y2 - y1)^2) * cos(theta2));
% 
% % 对上式求全微分
% A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1); 
%     diff(f3, x1), diff(f3, y1);];
% A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2); 
%     diff(f3, x2), diff(f3, y2);];
% A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys);
%     diff(f3, xs), diff(f3, ys);];
% A_Mt = [diff(f1, t1), diff(f1, t2); diff(f2, t1), diff(f2, t2);
%     diff(f3, t1), diff(f3, t2); ];
% A_Mc = [diff(f1, c); diff(f2, c); diff(f3, c); ];
% A_Ma = [diff(f1, theta1), diff(f1, theta2); diff(f2, theta1), diff(f2, theta2);
%     diff(f3, theta1), diff(f3, theta2); ];
% 
% 
% M1 = matlabFunction(A_M1);
% M2 = matlabFunction(A_M2);
% Mc = matlabFunction(A_Mc);
% M = matlabFunction(A_M);
% Mt = matlabFunction(A_Mt);
% Ma = matlabFunction(A_Ma);
% clearvars -except M Mt M1 M2 Mc Ma
% 
% x = 0:5:2000;
% y = (0:5:2000)';
% [x1, y1, x2, y2] = deal(500, 1000, 1500, 1000); %% 平台位置
% [lenx, leny] = deal(length(x), length(y));
% % errornor = [2^2, 2^2, 0.002^2, 1.5^2, 1^2]; % dx^2 dy^2 t^2 c^2 theta误差
% errornor = [0^2, 0^2, 0.002^2, 1.5^2, (0.1 / 180 * pi)^2, (0.1 / 180 * pi)^2]; % dx^2 dy^2 t^2 c^2 theta误差
% 
% errorsum = zeros(lenx, leny);
% c = 1500;
% ts = 0;
% theta1 = zeros(lenx, leny);
% for ii = 1:lenx
%     for jj = 1:leny
%         % theta 是角目标-平台1-平台2
%         % theta = 角-基线和水平夹角
%         theta1(ii, jj) = atan2(y(jj) - y1 ,x(ii) - x1) - atan2((y2 - y1),(x2 - x1)); % 四象限反正切
% %         theta2(ii, jj) = atan2(y(jj) - y2 ,x(ii) - x2) - atan2((y1 - y2),(x1 - x2)); % 四象限反正切
%     end
% end
% for ii = 1:lenx
%     for jj = 1:leny
%         t1 = sqrt((x(ii) - x1)^2+(y(jj) - y1)^2) / c;
%         t2 = sqrt((x(ii) - x2)^2+(y(jj) - y2)^2) / c;
%         m1 = M1(theta1(ii, jj),x1, x2, x(ii), y1, y2, y(jj));
%         m2 = M2(theta1(ii, jj),  x1, x2, x(ii), y1, y2, y(jj));
%         mc = Mc(c, t1, t2, ts);
%         m = M(theta1(ii, jj),  x1, x2, x(ii), y1, y2, y(jj));
%         mt = Mt(c, t1, t2, ts);
%         ma = Ma(theta1(ii, jj),  x1, x2, x(ii), y1, y2, y(jj));
% 
%         ttt = (m1 * diag([errornor(1), errornor(2)]) * m1' ...
%             + m2 * diag([errornor(1), errornor(2)]) * m2' ...
%             + mt * diag([errornor(3), errornor(3)]) * mt' ...
%             + mc * diag([errornor(4)]) * mc' ...
%             + ma * diag([errornor(5), errornor(6)]) * ma');
%         error = (m' * m) \ m' * ttt * m / (m' * m);
%         errorsum(jj, ii) = sqrt(trace(error));
%     end
% end
% 
% figure
% surf(x, y, abs(errorsum))
% set(gca, 'YDir', 'normal');
% colorbar;
% xlabel('x/m');
% ylabel('y/m');
% title('TOA-AOA平面角解算误差');
% clim([0, 5]); colormap jet
% shading interp;view(0,90)
% average_value = nanmean(errorsum(:));
% disp(['非NaN值的平均值为：', num2str(average_value)]);



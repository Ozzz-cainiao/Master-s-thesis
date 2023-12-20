clc;
clear;
close all;
k = 0;
while k <= 100
    t = 0:(50 + k) / 200:50 + k;
    i = 1;
    test1_1 = randn(1, 201);
    while i <= 201
        f(i) = 5000 - 5000 * 10 * (-1000 + 10 * t(i)) / 1500 / sqrt((-1000 + 10 * t(i))^2+200^2);
        f(i) = f(i) + sqrt(1) * test1_1(1, i);
        i = i + 1;
    end

    x0 = [500, 5, -500, 50];
    % x0 初始值
    xdata = t;
    ydata = f;
    % xdata,ydata 拟合数据

    fun = @(x, xdata)x(1) - x(1) * x(2) * (x(3) + x(2) * xdata) ./ sqrt((x(3) + x(2) * xdata).^2+x(4)^2) / 1500;
    % fun 拟合函数
    options = optimset('Display', 'iter', 'Tolx', 1e-10, 'TolFun', 1e-10);
    lb = [0, 0, -2000, 1];
    ub = [10000, 30, -50, 10000];
    [x, resnorm] = lsqcurvefit(fun, x0, xdata, ydata, lb, ub);

    %画出优化后的曲线
    RMSE_f(k+1) = abs(x(1)-5000);
    RMSE_s(k+1) = abs(x(2)-10);
    RMSE_x(k+1) = abs(x(3)+1000);
    RMSE_y(k+1) = abs(x(4)-200);
    frequence(k+1) = x(1) / 10;
    speed(k+1) = x(2);
    x1(k+1) = x(3);
    y1(k+1) = x(4);
    k = k + 1;
end
t1 = 50:1:150;
f_real = 500 * ones(1, 101);
v_real = 10 * ones(1, 101);
x0_real = -1000 * ones(1, 101);
y0_real = 200 * ones(1, 101);
figure
plot(t1, frequence, 'k-', t1, f_real, 'r--');
xlabel('t/s');
ylabel('f/Hz');
%title('估计频率与时间关系');
legend('估计频率', '真实频率');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12
figure
plot(t1, RMSE_f, 'k');
xlabel('t/s');
ylabel('RMSEf/Hz');
%title('估计频率的均方误差与时间关系');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12

figure
plot(t1, speed, 'k-', t1, v_real, 'r--');
xlabel('t/s');
ylabel('v/(m/s)');
%title('估计速度与时间关系');
legend('估计速度', '真实速度');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12
figure
plot(t1, RMSE_s, 'k');
xlabel('t/s');
ylabel('RMSEv/(m/s)');
%title('估计速度的均方误差与时间关系');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12

figure
plot(t1, x1, 'k-', t1, x0_real, 'r--');
xlabel('t/s');
ylabel('x_s/m');
%title('估计横坐标与时间关系');
legend('估计横坐标', '真实横坐标');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12
figure
plot(t1, RMSE_x, 'k');
xlabel('t/s');
ylabel('RMSEx_s/m');
%title('估计横坐标的均方误差与时间关系');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12

figure
plot(t1, y1, 'k-', t1, y0_real, 'r--');
xlabel('t/s');
ylabel('y_s/m');
%title('估计纵坐标与时间关系');
legend('估计纵坐标', '真实纵坐标');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12
figure
plot(t1, RMSE_y, 'k');
xlabel('t/s');
ylabel('RMSEy_s/m');
%title('估计纵坐标的均方误差与时间关系');
set(gcf, 'Color', [1, 1, 1])
set(gca, 'Fontsize', 14) %设置字体大小为12
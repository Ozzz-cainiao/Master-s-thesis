%% 验证目标设置的位置

% Author LI Shuo
% 2023年9月18日 10点54分
clc
clear
close all

%% ==========================布放参数===============================
rand('seed', 1)
c = 1500;
var2d = 1.5^2;
% var2d   = 0^2 ;
var2 = var2d * (pi / 180)^2;

v0 = [10, 10, 10, 10];
range 	= [ 6e3 10e3 5e3 10e3]; % 目标产生位置的距离
bear = [80, 30, 15, 60]; % 目标产生位置的角度
course = [90, 50, 60, 120]; % 运动方向

num = length(v0);
x0 = range .* cosd(bear);
y0 = range .* sind(bear);
vx = v0 .* cosd(course);
vy = v0 .* sind(course);
X0 = [x0; y0; vx; vy];
T = 1e-3; %观测周期
T_all = 5; %观测时间
T_num = T_all / T; %观测次数
dt = T;

%匀速运动矩阵
F1 = [1, 0, T, 0; ...
    0, 1, 0, T; ...
    0, 0, 1, 0; ...
    0, 0, 0, 1];
%加速度矩阵
F2 = [0.5 * T^2, 0; ...
    0, 0.5 * T^2; ...
    T, 0; ...
    0, T];

x = zeros(num, T_num); % 目标x坐标
y = zeros(num, T_num); % 目标y坐标
x_r = zeros(num, T_num); % 目标相对观测者x坐标
y_r = zeros(num, T_num); % 目标相对观测者y坐标
angle_r = zeros(num, T_num); % 目标相对观测者方位
a_max = 0 / 1e3; % 目标最大加速度
X = arrayfun(@(j) zeros(4, T_num), 1:num, 'un', 0);

%% ==========================目标真实状态===============================
for t = 1:T_num
    for j = 1:length(x0)
        ax = a_max * (2 * rand - 1);
        ay = a_max * (2 * rand - 1);
        a = [ax, ay]';
        if t == 1
            X{j}(:, t) = X0(:, j);
        else
            X{j}(:, t) = F1 * X{j}(:, t - 1) + F2 * a;
        end
    end
end
for i = 1:num
    x(i, :) = X{i}(1, :);
    y(i, :) = X{i}(2, :);
end
birthPlace = [x(:, 1), y(:, 1)];
placeX = x;
placeY = y;
node = [0, 0; 10e3, 0; 10e3, 10e3; 0, 10e3];
for ii = 1:size(node, 1)
    for i = 1:num
        x_r(i, :) = node(ii, 1) - x(i, :);
        y_r(i, :) = node(ii, 2) - y(i, :);
        r_r = sqrt(x_r(i, :).^2+y_r(i, :).^2);
        angle_r(i, :) = atan2d(y_r(i, :), x_r(i, :));
        angle_r(angle_r < 0) = angle_r(angle_r < 0) + 360;
    end
    angR{ii} = angle_r;
    angM{ii} = angle_r + sqrt(var2d) * randn(size(angle_r));
end

%% 观测站和初始点的位置
color = ['#059341'; '#EFA90D'; '#006BB0'; '#DC2F1F'; '#EFA90D'; '#7E2F8E'];
%这句代码是在 MATLAB 中创建一个新的图形窗口（figure）。具体而言，它设置了图形窗口的一些属性，包括单位、位置和大小。

% - `'Units', 'centimeters'`：指定图形的单位为厘米。
% - `'Position', [20, 5, 20, 11.24 / 15 * 15]`：指定图形的位置和大小。
%   具体来说，图形位于屏幕上的位置 `[20, 5]`，宽度为 `20` 厘米，
%   高度为 `11.24 / 15 * 15` 厘米。这里的数字可能是根据具体需求设置的。

figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15])
for i = 1:num
    if i < 8
        color_line = color(i, :);
    else
        color_line = [1.0000, 0.5721, 0.0349];
    end
    aim_f(i) = scatter(birthPlace(i, 1), birthPlace(i, 2), 100, '*', 'MarkerEdgeColor', color_line);
    hold on
    plot(placeX(i, :), placeY(i, :), 'Color', color_line, 'LineWidth', 2)
end
obs_f = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
grid on
legend([aim_f, obs_f], '目标1', '目标2', '目标3', '目标4', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% axis([-3,3,-2.2,3.4])
set(gca, 'Box', 'on')
xlabel('东向坐标/km', 'FontSize', 12)
ylabel('北向坐标/km', 'FontSize', 12)

%% 各观测站看到的目标角度随时间的变化
for ii = 1:size(node, 1)
    figure
    hold on
    for i = 1:num
        if i < 8
            color_line = color(i, :);
        else
            color_line = [1.0000, 0.5721, 0.0349];
        end
        hold on
        scatter(angR{ii}(i, :), (T:T:T_all)/60*5, 1, 'filled', 'MarkerEdgeColor', color_line, 'MarkerFaceColor', color_line, 'LineWidth', 2)
    end
    set(gca, 'Box', 'on')
    xlabel('角度/°', 'FontSize', 12)
    ylabel('时间/min', 'FontSize', 12)
    % legend('目标1','目标2','目标3','Location','best','FontSize',12)
    legend('目标1', '目标2', '目标3', '目标4', 'Location', 'best')
    txtStr = ['观测站', num2str(ii)];
    grid on
    Hf = gcf;
    Hf.Position = [100, 100, 560, 420];
    hold off
end

%%
S = size(node, 1);
% close all
Ylim    = [-2.5,2.5];
xgrid = -3:0.02:3;
Ylim    = [0,10e3];
xgrid = 0:10:10e3;
y = cell(S, 1);
fig = figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);
for s = 1:S
    theta = angM{s}(:, 1);
    xp = node(s, 1);
    yp = node(s, 2);
    y{s} = cell2mat(arrayfun(@(x) (repmat(x-xp, length(theta), 1)).*tand(theta)+repmat(yp, length(theta), 1), xgrid, 'un', 0));
    %     locsp1  = arrayfun(@(x) abs(atan2d((y{s}(x,:)-yp),(xgrid-xp))-theta(x))>1,1:length(theta),'un',0);  % 在反向线上的点
    %     y{s}(cell2mat(arrayfun(@(v) v,cell2mat(locsp1'),'un',0))) = nan;                                   	% 在反向线上的点舍去
    locsp2 = arrayfun(@(x) y{s}(x, :) <= Ylim(1) | y{s}(x, :) >= Ylim(2), 1:length(theta), 'un', 0); % 超过视距的点
    y{s}(cell2mat(arrayfun(@(v) v, cell2mat(locsp2'), 'un', 0))) = nan; % 超过视距的点舍去
    figure(fig)
    hold on
    % 绘制测向线
    h = arrayfun(@(x) plot(xgrid, y{s}(x, :), '--', 'Color', '#808080'), 1:length(theta));
    hold off
end
figure(fig)
hold on
% 观测站位置
s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% 绘制目标出生点的位置
s2 = scatter(birthPlace(:, 1), birthPlace(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100);
legend([h(end), s1, s2], '方位测量', '观测站', '目标', 'Location', 'eastoutside', 'FontSize', 12)
hold off
% axis([min(xgrid)-1,max(xgrid)+3,Ylim])
% axis([-3,3,-2.5,2.5])
set(gca, 'Box', 'on')
xlabel('东向坐标/km', 'FontSize', 12)
ylabel('北向坐标/km', 'FontSize', 12)
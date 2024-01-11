%% 多目标强互扰条件下的非合作定位方法研究

% 典型的分布式多目标定位场景中往往包含多种独立的和相干的声源，
% 比如发射连续宽带信号的声源、发射脉冲信号与宽带信号叠加的声源，
% 还有接收脉冲信号然后模拟回波的声源等。定位场景十分复杂。
% 因此非合作多目标定位往往遇到以下几种问题：
% 1）目标信号多样，有连续声源、脉冲声源和相干声源等，且声源强度不一，如何建立一个可以对多种信号进行统一定位的模型；
% 2）多个声源之间会相互干扰，或者节点出现故障等，会产生偏离期望的信息，如何克服这些异常影响。
% 3）各观测节点测得的种类众多，并且存在相干声源，如何对这些信息进行区分与关联。
clc
clear
close all

%% 对多种信号进行统一定位的模型

%% 首先生成方位数据 生成4个目标

%% ==========================布放参数===============================
rand('seed', 1)
node = [0, 0; 10e3, 0; 10e3, 10e3; 0, 10e3]; % 节点位置
c = 1500;
var2d = 1.5^2;
% var2d   = 0^2 ;
var2 = var2d * (pi / 180)^2;
v0 = [10, 10, 20, 15];
range 	= [2e3 4e3 6e3 8e3]; % 目标产生位置的距离,与原点的距离
bear = [60, 30, 90, 45]; % 目标产生位置的角度
course = [90, 50, 40, 120]; % 运动方向
num = length(v0);
x0 = range .* cosd(bear); % 初始位置
y0 = range .* sind(bear); % 初始速度
vy = v0 .* cosd(course);
vx = v0 .* sind(course);
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
color = ['#059341'; '#EFA90D'; '#059341'; '#DC2F1F'; '#EFA90D'; '#7E2F8E'];
figure('Units', 'centimeters', 'Position', [20, 5, 20, 11.24 / 15 * 15]); 
hold on
for i = 1:num
    aim_r = plot(x(i, :), y(i, :), 'Color', color(i,:), 'LineWidth', 2);
end
set(gca, 'Box', 'on')
grid on
obs_f = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100); % 画线并声明变量名称
legend([aim_r obs_f],'真实轨迹', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
title('目标真实轨迹')


%% ==========================观测===============================

t_obs = T:T:T_num * T + 20;
X = repmat(node, 1, 1, T_num);
[x_obs, y_obs] = deal(zeros(size(node, 1), T_num));
for ii = 1:size(node, 1)
    x_obs(ii, :) = X(ii, 1, :);
    y_obs(ii, :) = X(ii, 2, :);
    angR{ii} = nan(num, length(t_obs));
    for i = 1:num
        x_r(i, :) = x(i, :) - x_obs(ii, 1:T_num);
        y_r(i, :) = y(i, :) - y_obs(ii, 1:T_num);
        r_r = sqrt(x_r(i, :).^2+y_r(i, :).^2);
        angle_r(i, :) = atan2d(y_r(i, :), x_r(i, :));
        angle_r(angle_r < 0) = angle_r(angle_r < 0) + 360;
        t_r = r_r / c;
        t_delay{i, ii} = round(t_r, 1);
        for iii = 1:T_num
            tNum = round(t_delay{ii}(iii)/T) + iii;
            angR{ii}(i, tNum) = angle_r(i, iii) + sqrt(var2d) * randn;
        end
    end
    for iii = 1:size(angR{ii}, 2)
        angR{ii}(:, iii) = sort(angR{ii}(:, iii));
    end
end

% 画图

figure('Units', 'centimeters', 'Position', [20, 5, 20, 11.24 / 15 * 15]); 
% 这段 MATLAB 语法 figure('Units', 'centimeters', 'Position', [20, 5, 20, 11.24 / 15 * 15]) 是用于创建一个图形窗口，并设置其单位、位置和大小。
% figure()：创建一个新的图形窗口，并返回一个与该窗口关联的图形对象。括号内可以指定其他参数来自定义窗口的属性。
% 'Units', 'centimeters'：这是一个名称-值对，用于设置图形窗口的单位。在本例中，单位被设置为厘米。
% 'Position', [20, 5, 20, 11.24 / 15 * 15]：这是另一个名称-值对，用于设置图形窗口的位置和大小。[20, 5, 20, 11.24 / 15 * 15] 是一个包含四个元素的向量，
% 分别表示窗口的左下角 x 坐标、左下角 y 坐标、宽度和高度。在本例中，窗口的左下角位于 x = 20 厘米、y = 5 厘米的位置，宽度为 20 厘米，
% 高度为 11.24 / 15 * 15 厘米。
for i = 1:num
    if i < 8
        color_line = color(i, :);
    else
        color_line = [1.0000, 0.5721, 0.0349];
    end
    hold on
    %     outLoc  = plot(outLoctionSPCX(i,:),outLoctionSPCY(i,:),'.','Color',color_line,'LineWidth',2);
    aim_a = scatter(x0(i), y0(i), 10, '*', 'MarkerEdgeColor', color(4, :));
    aim_f = plot(x(i, :), y(i, :), 'Color', color(4, :), 'LineWidth', 2);
end
obs_f = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
grid on
legend([aim_a, obs_f], '真实轨迹', '观测站', 'Location', 'eastoutside', 'FontSize', 12)

% legend([outLoc,aim_a obs_f],'定位结果','真实轨迹','观测站','Location','eastoutside','FontSize',12)
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
title('粗关联+时空关联定位')

%% 特征信息
% 目标1 lfm 连续声源
% 目标2 cw + lfm 脉冲信号与宽带信号叠加
% 目标3 cw + lfm 脉冲信号与宽带信号叠加
% 目标4 

%% ==========================新观测+添加特征信息===============================

t_obs = T:T:T_num * T + 20;
X = repmat(node, 1, 1, T_num);
[x_obs, y_obs] = deal(zeros(size(node, 1), T_num));
dataR = cell(1, size(node, 1));



for ii = 1:size(node, 1)
    x_obs(ii, :) = X(ii, 1, :);
    y_obs(ii, :) = X(ii, 2, :);
    dataR{ii} = cell(num, length(t_obs)); % 作为存储真实数据的cell  4*25000cell
    for i = 1:num
        x_r(i, :) = x(i, :) - x_obs(ii, 1:T_num);
        y_r(i, :) = y(i, :) - y_obs(ii, 1:T_num);
        r_r = sqrt(x_r(i, :).^2+y_r(i, :).^2);
        angle_r(i, :) = atan2d(y_r(i, :), x_r(i, :));
        angle_r(angle_r < 0) = angle_r(angle_r < 0) + 360;
        t_r = r_r / c;
        t_delay{i, ii} = round(t_r, 1);
        for iii = 1:T_num
            tNum = round(t_delay{ii}(iii)/T) + iii;
            angR{ii}{i, tNum} = angle_r(i, iii) + sqrt(var2d) * randn;
        end
    end
    for iii = 1:size(angR{ii}, 2)
        angR{ii}(:, iii) = sort(angR{ii}(:, iii));
    end
end

%% ==========================添加特征信息===============================




% 接收参数级的信息，根据接收数组的种类线谱特点进行分类判断

% 读入接收平台的信息
for i = 1:5
    data(i) = load(['platform', num2str(i), '.mat']);
    % 读进来是一个结构体 开始提取这个结构体的信息
    % 119行 对应于119帧


end

%进入不同的信号处理流程
% 怎么判断是不同的信号？
% cw 只有线谱  LFM只有连续谱  舰船辐射噪声既有连续谱又有线谱

% 设置标记数组,记录每种信号的下标号

%% 生成目标运动轨迹

%% 先进行粗关联再进行时空关联
function arrR = generate()

%% ==========================布放参数===============================
rand('seed', 1)
c = 1500;
var2d = 1.5^2;
% var2d   = 0^2 ;
var2 = var2d * (pi / 180)^2;
v0 = [10, 10];
range 	= [ 6000 8e3 ]; % 目标产生位置的距离
bear = [60, 30]; % 目标产生位置的角度
course = [90, 50]; % 运动方向
num = length(v0); % 目标数量
x0 = range .* cosd(bear);
y0 = range .* sind(bear);
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

%% ==========================观测===============================
node = [0, 0; 10e3, 0; 10e3, 10e3; 0, 10e3]; % 节点位置
t_obs = T:T:T_num * T + 20; % 观测时间
X = repmat(node, 1, 1, T_num); % 4*2*5000 double  其实就是这些观测时刻的各平台位置
[x_obs, y_obs] = deal(zeros(size(node, 1), T_num)); % 4*5000 double  4 * 5000 double
% 外层循环是不同观测平台  共有4个观测平台
for ii = 1:size(node, 1)
    x_obs(ii, :) = X(ii, 1, :);
    y_obs(ii, :) = X(ii, 2, :);
    angR{ii} = nan(num, length(t_obs));

    % 内层循环是目标数量
    for i = 1:num
        x_r(i, :) = x(i, :) - x_obs(ii, 1:T_num);
        y_r(i, :) = y(i, :) - y_obs(ii, 1:T_num);
        r_r = sqrt(x_r(i, :).^2+y_r(i, :).^2);
        angle_r(i, :) = atan2d(y_r(i, :), x_r(i, :));
        angle_r(angle_r < 0) = angle_r(angle_r < 0) + 360;
        t_r = r_r / c;
        t_delay{i, ii} = round(t_r, 1);

        % 这一层是观测次数  也就是时间上的观测结果序列
        for iii = 1:T_num
            tNum = round(t_delay{ii}(iii)/T) + iii;
            angR{ii}(i, tNum) = angle_r(i, iii) + sqrt(var2d) * randn; % 第一索引是平台序号，i是目标编号， tNum是时间序号
        end

    end

    for iii = 1:size(angR{ii}, 2) % ans = 25000
        angR{ii}(:, iii) = sort(angR{ii}(:, iii)); % 根据时间将角度排序
    end
end
aR = arrR;
xxx = 1000;
end

clc
close
clear

% %% 符号计算示例
% syms x y;
% eq1 = x^2 + y^2 - 1;  % 方程1
% eq2 = x + y - 1;      % 方程2
%
% % 解决方程组，表示y为一个关于x的函数
% sol = solve(eq1, eq2, x ,y);
% a = 11
% % 输出y作为关于x的函数
% % y_as_function_of_x = sol.y;
% % disp(y_as_function_of_x);

%% 进行符号计算
% syms x  x1 x2 x3 x4 y  y1 y2  y3  y4 c t12 t13 t14
% f1 = sqrt((x - x1).^2 + (y - y1).^2) - sqrt((x - x2).^2 + (y - y2).^2) - 1500*t12;
% f2 = sqrt((x - x1).^2 + (y - y1).^2) - sqrt((x - x3).^2 + (y - y3).^2) - 1500*t13;
% f3 = sqrt((x - x1).^2 + (y - y1).^2) - sqrt((x - x4).^2 + (y - y4).^2) - 1500*t14;
% % 解决方程组
% sol = solve(f1, f2, f3, x, y);
% disp(sol.x)

%% TDOA 推导，依据chatgpt---------这个可用
clc
clear
close all
syms xs  x1 x2 x3 x4 ys  y1 y2  y3  y4 c t12 t13 t14 d1 d2 d3 d4
d1 = sqrt((xs - x1).^2+(ys - y1).^2);
d2 = sqrt((xs - x2).^2+(ys - y2).^2);
d3 = sqrt((xs - x3).^2+(ys - y3).^2);
d4 = sqrt((xs - x4).^2+(ys - y4).^2);

t12 = (d1 - d2) / c;
t13 = (d1 - d3) / c;
t14 = (d1 - d4) / c;

a = t12 * t12;
b = t13 * t13;
c = t14 * t14;
sol = solve(a, b, xs, ys); % 方程个数等于未知数个数才可以
sol.xs
sol.ys

%%
clc
clear
close all
syms xs  x1 x2 x3 x4 ys  y1 y2  y3  y4 c t12 t13 t14 d1 d2 d3 d4
eq1 = sqrt((xs - x1).^2+(ys - y1).^2) == d1;
eq2 = sqrt((xs - x2).^2+(ys - y2).^2) == d2;
eq3 = sqrt((xs - x3).^2+(ys - y3).^2) == d3;
eq4 = sqrt((xs - x4).^2+(ys - y4).^2) == d4;

t12 = (d1 - d2) / c;
t13 = (d1 - d3) / c;
t14 = (d1 - d4) / c;

equations = [eq1, eq2, eq3, eq4];

% 使用最小二乘法求解
[A, b] = equationsToMatrix(equations, [xs, ys]);
x_sol = linsolve(A, b);

%% TDOA定位仿真
clc
clear
close all
% 目标初始位置
target1_initial = [3e3, 5e3];
target2_initial = [6e3, 6e3];

% 观测平台位置
platforms = [0, 0; 0, 1e4; 1e4, 1e4; 1e4, 0];

% 信号传播速度
v = 1500; % 光速

% 目标匀速运动速度
velocity = [20, 30]; % x和y方向的速度

% 模拟时间
time = 0:1:100;

% 初始化目标位置
target1_position = zeros(length(time), 2);
target2_position = zeros(length(time), 2);

target1_position(1, :) = target1_initial;
target2_position(1, :) = target2_initial;

% 模拟目标的匀速运动
for i = 2:length(time)
    target1_position(i, :) = target1_position(i-1, :) + velocity * (time(i) - time(i-1));
    target2_position(i, :) = target2_position(i-1, :) + velocity * (time(i) - time(i-1));
end

% 计算TDOA
tdoa_target1 = calculateTDOA(target1_position, platforms, v);
tdoa_target2 = calculateTDOA(target2_position, platforms, v);

% 添加噪声
noise_level = 1e-8;
tdoa_target1_noisy = tdoa_target1 + noise_level * randn(size(tdoa_target1));
tdoa_target2_noisy = tdoa_target2 + noise_level * randn(size(tdoa_target2));

% 使用TDOA进行定位
estimated_positions_target1 = localizeTarget(tdoa_target1_noisy, platforms, v);
estimated_positions_target2 = localizeTarget(tdoa_target2_noisy, platforms, v);

% 绘制结果
figure;
scatter(platforms(:, 1), platforms(:, 2), 'filled', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
hold on;
scatter(target1_position(:, 1), target1_position(:, 2), 'o', 'LineWidth', 2);
scatter(target2_position(:, 1), target2_position(:, 2), 's', 'LineWidth', 2);
plot(estimated_positions_target1(:, 1), estimated_positions_target1(:, 2), '-g', 'LineWidth', 2);
plot(estimated_positions_target2(:, 1), estimated_positions_target2(:, 2), '-b', 'LineWidth', 2);
legend('观测平台', '目标1真实位置', '目标2真实位置', '目标1估计轨迹', '目标2估计轨迹');
xlabel('X轴位置');
ylabel('Y轴位置');
title('TDOA目标定位');
grid on;
xlim([0, 1e4])
ylim([0, 1e4])


% 定义计算TDOA的函数
function tdoa = calculateTDOA(target_position, platforms, v)
num_platforms = size(platforms, 1);
num_combinations = nchoosek(num_platforms-1, 2); % 选择三个平台的组合数

tdoa = zeros(size(target_position, 1), num_combinations);

% 计算目标到每一对平台之间的时延差
k = 1;
reference_platform = platforms(1, :);

for i = 2:num_platforms
    distance_to_target_i = sqrt(sum((target_position - platforms(1, :)).^2, 2));
    distance_to_target_j = sqrt(sum((target_position - platforms(i, :)).^2, 2));

    tdoa(:, k) = (distance_to_target_i - distance_to_target_j) / v;
    k = k + 1;
end
end

% 定义定位目标的函数-------------这个函数依旧是有问题的
function estimated_positions = localizeTarget(tdoa, platforms, v)
    num_measurements = size(tdoa, 1);
    
    % 初始化估计位置矩阵
    estimated_positions = zeros(num_measurements, 2);
    
    for i = 1:num_measurements
        % 构建方程组
        A = zeros(i-1, 2);
        b = zeros(i-1, 1);
        
        for j = 2:i
            % 修正此处的索引
            j_platform = mod(j, size(platforms, 1));
            if j_platform == 0
                j_platform = size(platforms, 1);
            end
            
            A(j-1, :) = 2 * (platforms(j_platform, :) - platforms(1, :));
            b(j-1) = v^2 * (tdoa(j, 1) - tdoa(1, 1));
        end
        
        % 解方程组
        x = A \ b;
        
        % 估计目标位置
        estimated_positions(i, :) = platforms(1, :) + 0.5 * x';
    end
end


%
% % 定义定位目标的函数
% function estimated_position = localizeTarget(tdoa, platforms, v)
%     num_measurements = size(tdoa, 1);
%
%     % 构建方程组
%     A = zeros(num_measurements - 1, 2);
%     b = zeros(num_measurements - 1, 1);
%
%     for i = 2:num_measurements
%         % 修正此处的索引
%         i_platform = mod(i, size(platforms, 1));
%         if i_platform == 0
%             i_platform = size(platforms, 1);
%         end
%
%         A(i-1, :) = 2 * (platforms(i_platform, :) - platforms(1, :));
%         b(i-1) = v^2 * (tdoa(i, 1) - tdoa(1, 1));
%     end
%
%     % 解方程组
%     x = A \ b;
%
%     % 估计目标位置
%     estimated_position = platforms(1, :) + 0.5 * x';
% end

clc
clear
close all

% 参数设置
sound_speed = 1500; % 声速，单位：米/秒
platform_positions = [0, 0; 0, 1e4; 1e4, 1e4; 1e4, 0]; % 观测平台的位置
target_positions = [3e3, 5e3; 6e3, 6e3]; % 运动目标的初始位置
target_speed = 100; % 运动目标的速度，单位：米/秒
sampling_rate = 1; % 信号发射频率，单位：Hz

% 模拟运动目标的位置变化
time_steps = 100; % 模拟的时间步数
target_positions_history = zeros(time_steps, size(target_positions, 1), 2);

for t = 1:time_steps
    target_positions = target_positions + target_speed * [cos(pi/4); sin(pi/4)] / sampling_rate;
    target_positions_history(t, :, :) = target_positions;
end

% 计算到达时间差
num_platforms = size(platform_positions, 1);
num_targets = size(target_positions, 1);
TDOA_matrix = zeros(num_platforms, num_platforms, time_steps);

for t = 1:time_steps
    for i = 1:num_platforms
        for j = i+1:num_platforms
            distance_i = norm(squeeze(target_positions_history(t, :, 1)) - platform_positions(i, 1));
            distance_j = norm(squeeze(target_positions_history(t, :, 1)) - platform_positions(j, 1));
            TDOA_matrix(i, j, t) = (distance_i - distance_j) / sound_speed;
        end
    end
end


% TDOA定位
estimated_positions = zeros(time_steps, num_targets, 2);

for t = 1:time_steps
    for k = 1:num_targets
        A = zeros(num_platforms-1, 2);
        b = zeros(num_platforms-1, 1);

        idx = 1;
        for i = 1:num_platforms
            if i ~= 1
                A(idx, :) = 2 * (platform_positions(i, :) - platform_positions(1, :));
                b(idx) = TDOA_matrix(1, i, t) * sound_speed;
                idx = idx + 1;
            end
        end

        % 使用线性最小二乘法求解定位
        x = pinv(A) * b;
        estimated_positions(t, k, :) = platform_positions(1, :) + x';
    end
end

% 可视化结果
figure;
for k = 1:num_targets
    plot(squeeze(target_positions_history(:, k, 1)), squeeze(target_positions_history(:, k, 2)), '--', 'LineWidth', 2, 'DisplayName', ['True Target ' num2str(k)]);
    hold on;
    plot(squeeze(estimated_positions(:, k, 1)), squeeze(estimated_positions(:, k, 2)), 'o-', 'LineWidth', 2, 'DisplayName', ['Estimated Target ' num2str(k)]);
end
legend('Location', 'Best');
xlabel('X Position');
ylabel('Y Position');
title('TDOA定位结果');
grid on;

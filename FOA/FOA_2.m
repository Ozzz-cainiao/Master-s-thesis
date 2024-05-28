%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\FOA\FOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-4-29
% 描述: 对单平台信赖域算法的精度分析。估计频率精度对性能的影响
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.5; % 观测周期
dt = T;
startF = 0;
endF = 1;
all_res = zeros(11, 4);
times = 0;
myfun = @(params, t) params(1) * (1 - params(2) * (params(3) + params(2) * t) ./ (1500 * sqrt((params(3) + params(2) * t).^2+params(4)^2)));
fun = @(x, xdata)x(1) - x(1) * x(2) * (x(3) + x(2) * xdata) ./ sqrt((x(3) + x(2) * xdata).^2+x(4)^2) / 1500;

tic;
for var2f = startF:0.05:endF % 观测总时间
    times = times + 1;

    %% 重复次数
    count = 300;
    res = zeros(count, 4);
    T_all = 200;
    parfor num = 1:count
        T_num = T_all / T;
        t = 1:T:T_num * T;
        %% 运动模型
        % 这是什么模型？
        % 运动方程 x = x_last + v * t + 0.5 * a * t^2;
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

        %% 创建运动的声源对象
        initial_position1 = [-800, 200]; % 初始位置目标1
        velocity1 = [8, 0]; % 运动速度（假设在 x 轴上匀速运动）
        acc1 = 0; % 加速度
        % source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
        source1 = SoundSource('CW', 1200, 100, initial_position1, velocity1, F1, F2, acc1);

        %% 创建平台
        platform1 = Platform([0, 0]);

        %% 创建一个多维矩阵来存储目标信息
        % 维度1：平台，维度2：时刻，维度3：目标
        platFormAll = platform1;
        sourceAll = source1;
        numOfPlatForm = size(platFormAll, 2);
        numOfSource = size(sourceAll, 2);
        target_info_matrix = cell(numOfPlatForm, numOfSource, T_all+1); % cell矩阵
        angR = cell(numOfPlatForm, 1);
        Fre = cell(numOfPlatForm, 1);

        %% 观测
        for i = 1:T_num
            % 获取每个平台的每个目标信息
            for j = 1:numOfPlatForm % 遍历平台
                for k = 1:numOfSource % 遍历声源
                    if i == 1
                        [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfoFre(sourceAll(k), 0, 0, initial_position1(2));
                        t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻

                        Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延
                    else
                        sourceAll(k) = sourceAll(k).updatePosition();
                        [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfoFre(sourceAll(k), dt, 0, initial_position1(2));
                        t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                        Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延
                    end
                end % for k = 1:numOfSource % 遍历声源
            end % for j = 1:numOfPlatForm % 遍历平台
        end % for i = 1:T_num

        %% 计算
        FFre = cell2mat(Fre)';
        % sumOfFre = sumOfFre + FFre;
        % 找到非零值对应的索引
        nonZeroIndices = find(FFre(1:length(t)) ~= 0);

        % 根据索引从数组 B 中提取相应的值
        t = t(nonZeroIndices);
        FFre = FFre(nonZeroIndices);
        % initialGuess = [f0_guess, v_guess, x0_guess, y0_guess];  % Replace these with your initial guesses
        initialGuess = [FFre(end, 1), 10, -500, 200]; % Replace these with your initial guesses
        lb = [FFre(end, 1) - 10, 0, -4000, 1]; % 下界
        ub = [FFre(end, 1) + 10, 50, -10, 10000]; % 上界

        % 创建一个optimoptions对象
        options = optimoptions('lsqcurvefit');

        % 设置信赖域算法的迭代参数
        options.MaxIterations = 400;
        [xx, resnorm, residual, exitflag, output] = lsqcurvefit(myfun, initialGuess, t, FFre, lb, ub, options);
        % disp(xx)
        res(num, :) = xx;
    end

    %% 计算所有估计次数的平均值
    averageValue = mean(res);
    % fprintf('估计结果 %.2f\n', averageValue); % 显示2位小数
    all_res(times, :) = averageValue;
end
toc;
figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 7.5 / 4 * 3]); % 左下宽高
plot( startF:0.05:endF, all_res(:, 1), 'b-')
title('估计目标频率与测频精度曲线')
ylim([1198,1202]);xlim([startF,endF])
xlabel("测频精度(Hz)"); ylabel("频率(Hz)")

figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 7.5 / 4 * 3]); % 左下宽高
plot( startF:0.05:endF, all_res(:, 2), 'b-')
title('估计目标速度与测频精度曲线')
ylim([5,10]);xlim([startF,endF])
xlabel("测频精度(Hz)");ylabel("速度(m/s)")

figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 7.5 / 4 * 3]); % 左下宽高
plot( startF:0.05:endF, all_res(:, 3), 'b-')
title('估计目标x坐标与测频精度曲线')
ylim([-1000,-500]);xlim([startF,endF])
xlabel("测频精度(Hz)"); ylabel("坐标(m)")

figure('Units', 'centimeters', 'Position', [10, 10, 7.5, 7.5 / 4 * 3]); % 左下宽高
plot( startF:0.05:endF, all_res(:, 4), 'b-')
title('估计目标致近点距离与测频精度曲线')
ylim([0,300]);xlim([startF,endF])
xlabel("测频精度(Hz)");ylabel("坐标(m)")
%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\FOA\FOAPlus.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-19
% 描述: SZY 2.3.4仿真
%       参考论文邹男，于金正，桑志远，李娜，苏薪元，杨嘉轩，惠云梦．
%       非合作线谱声源 分布式定位方法[J/OL]．应用声学.
% 输入:
% 输出:
%**************************************************************************
clc
clear
close all

%% 观测数据
T = 0.2; % 观测周期
dt = T;
T_all = 300; % 观测总时间
T_num = T_all / T;
t = 0:T:T_num * T;
var2d = 1.5^2; % 角度制  角度误差
var2f = 0.2; % 观测的频率误差

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

%% 创建观测平台对象
platform1 = Platform([0, 0]);
platform2 = Platform([1.2e3, 0]);
platform3 = Platform([1.2e3, 1.2e3]);
platform4 = Platform([0, 1.2e3]);

%% 创建运动的声源对象
initial_position1 = [-400, 800]; % 初始位置目标1
v = 8;
slope = -0.5;
% slope = 0;
velocity1 = [v * cos(atan(slope)), v * sin(atan(slope))]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
% source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
source1 = SoundSource('CW', 1e3, 100, initial_position1, velocity1, F1, F2, acc1);

%% 求截距
% 已知直线的起始位置和斜率
x0 = initial_position1(1); % 起始位置的x坐标
y0 = initial_position1(2); % 起始位置的y坐标

% 计算一般方程的系数
% A = -slope;
B = 1;
C = slope * x0 - y0;

% 从一般方程中获取y轴上的截距
realb = -C / B;

% 显示结果
disp(['直线在y轴上的截距为: ', num2str(realb)]);

%% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
platFormAll = [platform1, platform2, platform3, platform4];
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
                %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);
                angR{j} = nan(T_all, numOfSource); % 现在只用来存放方位信息
                %                 [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfo(sourceAll(k), 0);

                [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfoFre(sourceAll(k), 0, slope, realb);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn;
                % 观测目标是频率
                Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延

            else
                sourceAll(k) = sourceAll(k).updatePosition();
                %                 [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                [angle, ~, t_delay, type, fre, platFormAll(j)] = platFormAll(j).getTargetInfoFre(sourceAll(k), dt, slope, realb);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(t_Num, k) = angle + sqrt(var2d) * randn; % 这个结果是度
                % 观测目标是频率
                Fre{j}(t_Num, k) = fre + sqrt(var2f) * randn; % 加时延
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm % 遍历平台
end % for i = 1:T_num

%% 对不同平台分别算
% myfun =  @(FFre, t) f0 - f0 * (v * (x0 + v * t) / (1500 * sqrt((x0 + v * t)^2 + y0^2)));
myfun = @(params, t) params(1) * (1 - params(2) * (params(3) + params(2) * t) ./ (1500 * sqrt((params(3) + params(2) * t).^2+params(4)^2)));
indexT = cell(numOfPlatForm, 1);
xx = cell(numOfPlatForm, 1);
for i = 1:numOfPlatForm

    FFre = cell2mat(Fre(i))';
    FFre = FFre(1:T_num);
    % 找到非零值对应的索引
    nonZeroIndices = find((~isnan(FFre)) & (FFre ~= 0));

    % 根据索引从数组 B 中提取相应的值
    indexT{i} = t(nonZeroIndices);
    tt = cell2mat(indexT(i));
    FFre = FFre(nonZeroIndices);

    initialGuess = [FFre(end, 1), 20, -1000, 2000]; % Replace these with your initial guesses
    lb = [FFre(end, 1) - 10, 0, -4000, 1]; % 下界
    ub = [FFre(end, 1) + 10, 50, -10, 10000]; % 上界

    % 创建一个optimoptions对象
    options = optimoptions('lsqcurvefit');

    % 设置信赖域算法的迭代参数
    options.MaxIterations = 400;
    [xx{i}, resnorm, residual, exitflag, output] = lsqcurvefit(myfun, initialGuess, tt, FFre, lb, ub, options);

    %     disp(output);

    %     figure
    %     plot(FFre);
    times = linspace(tt(1), tt(end));
    figure
    %     plot(tt, FFre, 'ko', times, myfun(cell2mat(xx(i)), times), 'b-')
    %     legend('Data', 'Fitted exponential')
    %     title('Data and Fitted Curve')
    %     plot(t, FFre, 'r-', times, myfun(averageValue, times), 'b-')
    plot(tt, FFre, 'r-', times, myfun(cell2mat(xx(i)), times), 'b-')

    legend('观测频率', '估计频率')
    title(['观测平台', num2str(i), '观测频率与估计频曲线'])
end

%% 开始获取运动参数
node = [0, 0; ...
    1200, 0; ...
    1200, 1e3; ...
    0, 1e3];
% 为什么V不对
d = zeros(1, numOfPlatForm);
vv = zeros(1, numOfPlatForm);
ff = zeros(1, numOfPlatForm);
for i = 1:numOfPlatForm
    fd = cell2mat(xx(i));
    d(i) = fd(4);
    vv(i) = fd(2);
    ff(i) = fd(1);
end
% vv

%% 开始计算目标运动参数
A = [node(3, 1), -node(3, 2); ...
    node(4, 1), -node(4, 2)];
B = cell(numOfPlatForm, 1);
X = cell(numOfPlatForm, 1);
B{1} = [d(3) / d(1) - 1, d(4) / d(1) - 1]';
B{2} = [-d(3) / d(1) - 1, d(4) / d(1) - 1]';
B{3} = [d(3) / d(1) - 1, -d(4) / d(1) - 1]';
B{4} = [-d(3) / d(1) - 1, -d(4) / d(1) - 1]';
res = zeros(numOfPlatForm, 2); % k, b
selectIndex = [1, 3, 4]; % 参与解算的观测平台
% 使用最小二乘法计算
Epsilon = zeros(numOfPlatForm, 1);
for i = 1:numOfPlatForm
    X{i} = inv(A'*A) * A' * cell2mat(B(i));
    k = X{i}(1) / X{i}(2);
    b = 1 / X{i}(2);
    res(i, 1) = X{i}(1) / X{i}(2); % k
    res(i, 2) = 1 / X{i}(2); % b
    dis = zeros(numOfPlatForm, 1);

    % 求1，3，4点到直线的距离
    for j = 1:numOfPlatForm
        dis(j) = pointToLineDistance(k, b, node(j, 1), node(j, 2));
        realDis(j) = pointToLineDistance(slope, realb, node(j, 1), node(j, 2));
    end

    % 解算点与距离之差的和
    for j = 1:length(selectIndex)
        Epsilon(i) = abs(dis(selectIndex(j))-d(selectIndex(j)));
    end
end

% 找到Epsilon最小的索引，这一对k,b作为解算结果
% 找到数组中的最小值
minValue = min(Epsilon);

% 找到最小值的索引
minIndex = find(Epsilon == minValue);
fprintf('估计频率为 %.2f，%.2f，%.2f，%.2f\n', ff(1), ff(2), ff(3), ff(4));
fprintf('估计速度为 %.2f，%.2f，%.2f，%.2f\n', vv(1), vv(2), vv(3), vv(4));
fprintf('估计距离为 %.2f，%.2f，%.2f，%.2f\n', d(1), d(2), d(3), d(4));
fprintf('估计的k的值是 %.2f，b 的值是 %.2f\n', res(minIndex, 1), res(minIndex, 2));
realDis = zeros(numOfPlatForm, 1);
% 求1，3，4点到直线的距离
for j = 1:numOfPlatForm
    realDis(j) = pointToLineDistance(slope, realb, node(j, 1), node(j, 2));
end
fprintf('真实距离为 %.2f，%.2f，%.2f，%.2f\n', realDis(1), realDis(2), realDis(3), realDis(4));

% 计算点到直线的距离
function distance = pointToLineDistance(k, b, x0, y0)
numerator = abs(k*x0-y0+b);
denominator = sqrt(k^2+1);
distance = numerator / denominator;
end

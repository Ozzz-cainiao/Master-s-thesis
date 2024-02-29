%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA_Calc.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-02-29
% 描述: TOA 两种解算方法解算结果，均不考虑深度，参考杨卓硕士论文
% 输入:  
% 输出:  
%**************************************************************************

%% 
clc
close all
clear
% 目标位置（单位：米）
target_position = [200, 300];

% 信号传播速度（假设单位为 m/s）
v = 1500; % 声速

% 接收器的位置（假设单位为米）
node = [
    0, 0; 
    1e3, 0; 
    0, 1e3;
    1e3, 1e3 
];

% 计算目标到各接收器的距离
d = zeros(size(node, 1), 1);
t = zeros(size(node, 1), 1);
for i = 1 : size(node, 1)
    d(i) = norm(target_position - node(i,:));
    t(i) = d(i) / v;
end


%% 线性解算方法 多平台选3，然后计算出目标位置
% 开始4选3
if size(node, 1) > 3
    combinationsT = nchoosek(t, 3);
    % 使用nchoosek函数生成所有可能的组合的索引
    combinationIndices = nchoosek(1:size(node, 1), 3);
    for index = 1:size(combinationsT, 1)
        timeDelay = combinationsT(index, :);
        nodeT = node(combinationIndices(index, :), :);
        [res(index, :)]= TOA2(timeDelay, nodeT);       
    end
    % 结果取平均
    res = res(find(~isnan(res(:, 1))), :);
    res = mean(res);
end


%% 线性解算 三圆交汇
function X = TOA1(timeDelay, node)
c = 1500;
len = size(node, 1);
r = zeros(len, 1);
d = zeros(len, 1);
for i = 1 : len 
    r(i) = sqrt(node(i, 1).^2 + node(i, 2).^2);
    d(i) = c * timeDelay(i);
end

C = 2*[node(2, 1) - node(1, 1), node(2, 2) - node(1, 2);
        node(3, 1) - node(1, 1), node(3, 2) - node(1, 2)];
D = [r(2).^2 - r(1).^2 + d(1).^2 - d(2).^2;
     r(3).^2 - r(1).^2 + d(1).^2 - d(3).^2;];
det_C = det(C);
if det_C ~= 0
    X = inv(C) * D;
else
    X = nan(2, 1);
end
end
%% 非线性解算 三平台 两个方程求双解 第三个方程判解
function X = TOA2(timeDelay, node)
X = nan(2, 2);
c = 1500;
len = size(node, 1);
r = zeros(len, 1);
d = zeros(len, 1);
for i = 1 : len 
    r(i) = sqrt(node(i, 1).^2 + node(i, 2).^2);
    d(i) = c * timeDelay(i);
end
if node(2, 2) - node(1, 2) < 1e-6
    X(1, 1) = (r(2).^2 - r(1).^2 + c.^2 * (timeDelay(1).^2 - timeDelay(2).^2))...
                /(2*(node(2, 1) - node(1, 1)));
else
    A = -(node(2, 1) - node(1, 1)) / (node(2, 2) - node(1, 2));
    B = (r(2).^2 - r(1).^2 + d(1).^ 2 - d(2).^2) - 2 * (node(2, 2) - node(1, 2));
    C = 1 + A.^2;
    D = -2 * (node(1, 1) + A * node(1, 2) - A * B);
    E = d(1).^2 - r(1).^2 + B.^2 - 2 * B * node(1, 2);
    if D * D - 4 * C * E > 0
        X(1, 1) = (-D + sqrt(D * D - 4 * C * E)) / (2 * C);
        X(2, 1) = (-D - sqrt(D * D - 4 * C * E)) / (2 * C);
        % 如何判解
        X(1, 2) = node(1, 2) - sqrt(c.^2 * timeDelay(1).^2 - (node(1, 1) - X(1, 1)).^2);
        X(2, 2) = node(1, 2) - sqrt(c.^2 * timeDelay(1).^2 - (node(1, 1) - X(2, 1)).^2);
    else
        disp("无解")
    end
end
X(1, 2) = sqrt(c.^2 * timeDelay(1).^2 - (node(1, 1) - X(1, 1)).^2) - node(1, 2);
X = X(1, :);
end

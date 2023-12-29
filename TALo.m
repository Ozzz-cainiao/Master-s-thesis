%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TALo.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-26
% 描述: TDOA/AOA定位方法的实现函数，参考公式 王静飞论文
% 输入:
% 输出:
%**************************************************************************

function [] = TALo(target_info_matrix, node)

%% 输入参数，当前测量的角度和时延 每一列是一个目标
numOfSource = size(target_info_matrix, 2);
numOfPlatForm = size(target_info_matrix, 1);
T_num = size(target_info_matrix{1, 1}, 1);
tMatrix = cell(numOfSource, 1);
aMatrix = cell(numOfSource, 1);
res = cell(numOfSource, 1);
for i = 1:numOfSource
    % 现在这是一个目标

    %% 针对不同平台

    % 提取有用信息
    tMatrix{i} = zeros(T_num, numOfPlatForm);
    aMatrix{i} = zeros(T_num, numOfPlatForm);
    for j = 1:numOfPlatForm
        % 获取当前结构体数组
        oneTarget = target_info_matrix{j, i};

        % 提取结构体中的信息
        t_delayInfo = [oneTarget.t_delay]; %
        angInfo = [oneTarget.angle];
        % 将提取到的信息存储到矩阵中的相应列
        tMatrix{i}(:, j) = t_delayInfo;
        aMatrix{i}(:, j) = angInfo;
    end

    %% 开始解算？

    res{i} = TA(tMatrix{i}, aMatrix{i}, node);
end

figure('Units', 'centimeters','Position', [15 5 20 11.24/15*15]);
hold on
for i = 1:numOfSource
    plot(res{i}(:, 1), res{i}(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
hold off
legend('目标1', '目标2', 'Location', 'eastoutside')
title("kmeans&TDOA/AOA定位结果")
set(gca, 'Box', 'on')
disp("结束");

end

%% 实现TDOA/AOA算法  参考公式 王静飞论文
% 输入参数 1个目标的时延和方位矩阵
% 输出参数 当前目标的时间定位结果
function [Res] = TA(tMatrix, aMatrix, node)
c = 1500;
time = size(tMatrix, 1);
% 使用 isnan 函数生成逻辑矩阵，其中非NaN元素为true，NaN元素为false
nanLogicalMatrix = ~isnan(tMatrix);

% 使用 sum 函数按行求和，得到每行非NaN元素的数量
nonNaNCountPerRow = sum(nanLogicalMatrix, 2);
Res = nan(time, 2);
for t = 1:time
    if nonNaNCountPerRow(t) >= 2
        % 可以进行计算了，寻找对应平台的索引
        % 获取该行中非NaN元素的索引
        nonNaNIndices = find(~isnan(tMatrix(t, :)));
        % 提取出对应的平台时延和角度
        currentT = tMatrix(t, nonNaNIndices);
        currentA = aMatrix(t, nonNaNIndices);
        nodeT = node(nonNaNIndices, :);
        % 两两组合 开始计算
        m = 0;
        for i = 1:size(currentA, 2)
            for j = i + 1:size(currentA, 2)
                % 在计算前加上一下判角条件
                % 当前平台的位置
                x1 = nodeT(i, 1);
                y1 = nodeT(i, 2);
                x2 = nodeT(j, 1);
                y2 = nodeT(j, 2);
                % 当前平台的时延
                t1 = currentT(i);
                t2 = currentT(j);
                % 当前平台的角度
                a1 = currentA(i);
                a2 = currentA(j);

                % 先随机开始计算
                ts = t1 - ((x2 - x1)^2 + (y2 - y1)^2 - c^2 * (t2 - t1)^2) ...
                    / (2 * c^2 * (t2 - t1) + 2 * c * cosd(a1) * sqrt((x2 - x1)^2+(y2 - y1)^2));
                d = t1 - ts;
                xs = x1 - c * d * ((y2 - y1) * sind(a1) - (x2 - x1) * cosd(a1)) ...
                    / sqrt((x2 - x1)^2+(y2 - y1)^2);
                ys = y1 - c * d * (-(y2 - y1) * cosd(a1) - (x2 - x1) * sind(a1)) ...
                    / sqrt((x2 - x1)^2+(y2 - y1)^2);

                % 求目标到两平台的距离
                targetPosition = [xs, ys]; % 目标位置坐标
                platformPositions = [x1, y1; x2, y2]; % 平台位置坐标矩阵

                % 使用 pdist 计算目标到两个平台的距离
                distances = pdist([targetPosition; platformPositions]);

                % 然后判断这个解是否距离i平台更近
                if distances(1) > distances(2)
                    % 重新计算结果
                    x1 = nodeT(j, 1);
                    y1 = nodeT(j, 2);
                    x2 = nodeT(i, 1);
                    y2 = nodeT(i, 2);
                    % 当前平台的时延
                    t1 = currentT(j);
                    t2 = currentT(i);
                    % 当前平台的角度
                    a1 = currentA(j);
%                     a2 = currentA(j);

                    % 先随机开始计算
                    ts = t1 - ((x2 - x1)^2 + (y2 - y1)^2 - c^2 * (t2 - t1)^2) ...
                        / (2 * c^2 * (t2 - t1) + 2 * c * cosd(a1) * sqrt((x2 - x1)^2+(y2 - y1)^2));
                    d = t1 - ts;
                    xs = x1 - c * d * ((y2 - y1) * sind(a1) - (x2 - x1) * cosd(a1)) ...
                        / sqrt((x2 - x1)^2+(y2 - y1)^2);
                    ys = y1 - c * d * (-(y2 - y1) * cosd(a1) - (x2 - x1) * sind(a1)) ...
                        / sqrt((x2 - x1)^2+(y2 - y1)^2);

                end
                m = m + 1;
                res(m, :) = [xs, ys];
                loc(m, :) = [i, j];
            end
        end
        [Res(t, 1), Res(t, 2)] = jujidu(res, loc, 800);
    end

end


end

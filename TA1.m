%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TA1.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-02
% 描述: 重新验证TA,同时加上多平台选2的功能，结果取平均，node是2维的
% 输入:  
% 输出:  
%**************************************************************************
function [res, loc] = TA1(currentT, currentA, nodeT)
% 两两组合 开始计算
% 开始多选2
if size(nodeT, 1) > 2
    combinationsT = nchoosek(currentT, 2);
    % 使用nchoosek函数生成所有可能的组合的索引
    combinationIndices = nchoosek(1:size(nodeT, 1), 2);
   
    for index = 1:size(combinationsT, 1)
        timeDelay = combinationsT(index, :);
        node = nodeT(combinationIndices(index, :), :);
        angle = currentA(combinationIndices(index, :));

        % 时延差/方位联合解算
        res(index, :) = calc(timeDelay, angle, node);

    end
    % 结果取平均
    res = res(find(~isnan(res(:, 1))), :);
    if size(res, 1) > 1
        res = mean(res, 1);
    elseif size(res, 1) < 1
        res = nan(1, 2);
    end
elseif size(nodeT, 1) == 2
    res = calc(currentT, currentA, nodeT);
end
loc = nan;
end

function res = calc(t, a, node)
c = 1500;
x1 = node(1, 1);
y1 = node(1, 2);
x2 = node(2, 1);
y2 = node(2, 2);
% 当前平台的时延
t1 = t(1);
t2 = t(2);
% 当前平台的角度 是目标-平台1-平台2的夹角
% 基线夹角
betai = atan2d((y2 - y1), (x2 - x1));
a1 = a(1) - betai;

% 先随机开始计算
ts = t1 - ((x2 - x1)^2 + (y2 - y1)^2 - c^2 * (t2 - t1)^2) ...
    / (2 * c^2 * (t2 - t1) + 2 * c * cosd(a1) * sqrt((x2 - x1)^2+(y2 - y1)^2));
d = t1 - ts;
xs = x1 - c * d * ((y2 - y1) * sind(a1) - (x2 - x1) * cosd(a1)) ...
    / sqrt((x2 - x1)^2+(y2 - y1)^2);
ys = y1 - c * d * (-(y2 - y1) * cosd(a1) - (x2 - x1) * sind(a1)) ...
    / sqrt((x2 - x1)^2+(y2 - y1)^2);

% 求目标到两平台的距离
res = [xs, ys]; % 目标位置坐标
platformPositions = [x1, y1; x2, y2]; % 平台位置坐标矩阵

% 使用 pdist 计算目标到两个平台的距离
distances = pdist([res; platformPositions]);

% 然后判断这个解是否距离i平台更近
if distances(1) > distances(2)
    % 重新计算结果
    x1 = node(2, 1);
    y1 = node(2, 2);
    x2 = node(1, 1);
    y2 = node(1, 2);
    % 当前平台的时延
    t1 = t(2);
    t2 = t(1);
    % 当前平台的角度
    betaj = atan2d((y2 - y1), (x2 - x1));
    a1 = a(2) - betaj;

    % 先随机开始计算
    ts = t1 - ((x2 - x1)^2 + (y2 - y1)^2 - c^2 * (t2 - t1)^2) ...
        / (2 * c^2 * (t2 - t1) + 2 * c * cosd(a1) * sqrt((x2 - x1)^2+(y2 - y1)^2));
    d = t1 - ts;
    xs = x1 - c * d * ((y2 - y1) * sind(a1) - (x2 - x1) * cosd(a1)) ...
        / sqrt((x2 - x1)^2+(y2 - y1)^2);
    ys = y1 - c * d * (-(y2 - y1) * cosd(a1) - (x2 - x1) * sind(a1)) ...
        / sqrt((x2 - x1)^2+(y2 - y1)^2);

end
res = [xs, ys];
% if abs(ts) < 1
%     res = [xs, ys];
%     %     loc{m, :}= ["t"+ num2str(i), "t"+ num2str(j), "theta"+ num2str(x)];
% else
%     
% end
end

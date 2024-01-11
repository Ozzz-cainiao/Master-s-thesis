%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: 这是自己编写的TDOA/AOA函数，计算每个时刻时延差/方位联合定位结果
% 输入: 绝对时延，角度，平台位置（二维的）
% 输出: 定位结果，分组结果
%**************************************************************************
function [res, loc] = TA(currentT, currentA, nodeT)
c = 1500;
% 两两组合 开始计算
m = 0;
for i = 1:size(currentA, 2)
    for j = i + 1:size(currentA, 2)
        x = i; % 假设用的角先默认为ai
        % 在计算前加上一下判角条件
        % 当前平台的位置
        x1 = nodeT(i, 1);
        y1 = nodeT(i, 2);
        x2 = nodeT(j, 1);
        y2 = nodeT(j, 2);
        % 当前平台的时延
        t1 = currentT(i);
        t2 = currentT(j);
        % 当前平台的角度 是目标-平台1-平台2的夹角
        % 基线夹角
        betai = atan2d((y2 - y1), (x2 - x1));
        a1 = currentA(i) - betai;

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
            betaj = atan2d((y2 - y1), (x2 - x1));
            a1 = currentA(j) - betaj;
            x = j;
            % 先随机开始计算
            ts = t1 - ((x2 - x1)^2 + (y2 - y1)^2 - c^2 * (t2 - t1)^2) ...
                / (2 * c^2 * (t2 - t1) + 2 * c * cosd(a1) * sqrt((x2 - x1)^2+(y2 - y1)^2));
            d = t1 - ts;
            xs = x1 - c * d * ((y2 - y1) * sind(a1) - (x2 - x1) * cosd(a1)) ...
                / sqrt((x2 - x1)^2+(y2 - y1)^2);
            ys = y1 - c * d * (-(y2 - y1) * cosd(a1) - (x2 - x1) * sind(a1)) ...
                / sqrt((x2 - x1)^2+(y2 - y1)^2);

        end
        if abs(ts) < 1
            m = m + 1;
            res(m, :) = [xs, ys];
            loc{m, :}= ["t"+ num2str(i), "t"+ num2str(j), "theta"+ num2str(x)];
        else
        end
    end
end
end
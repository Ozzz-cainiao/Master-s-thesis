%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TDOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: 这是自己编写的TDOA函数，计算每个时刻的结果
%       原理：双曲面交汇定位
%       参考文件 SolTrack.cpp
% 输入: 绝对时延，平台位置（三维的）
% 输出: 定位结果
%**************************************************************************

function [res, loc] = TDOA(timeDelay, node)
c = 1500;
zs = 0;
len = size(timeDelay, 2);
x_differ = node(1, 1) - node(2, 1);
y_differ = node(1, 2) - node(2, 2);

if len == 2 || len == 3
    d = zeros(len, 1);
    r = zeros(len, 1);
    for i = 1:len
        d(i) = node(i, 1)^2 + node(i, 2)^2 + node(i, 3)^2; % d是平台到原点的距离的平方
        r(i) = timeDelay(i)^2 * c^2; % r是平台到目标距离的平方
    end
    if x_differ < y_differ
        A = -1 * (node(2, 1) - node(1, 1)) / (node(2, 2) - node(1, 2));
        B = (2 * (node(1, 3) - node(2, 3)) * zs + d(2) - r(2) - d(1) + r(1)) / (2 * (node(2, 2) - node(1, 2)));
        C = 1 + A * A;
        D = -2 * (node(1, 1) + A * node(1, 2) - A * B);
        E = d(1) - r(1) + B * B - 2 * B * node(1, 1) - 2 * node(1, 3) * zs + zs^2;
        rotate = false;
    else
        A = -1 * (node(2, 2) - node(1, 2)) / (node(2, 1) - node(1, 1));
        B = (2 * (node(1, 3) - node(2, 3)) * zs + d(2) - r(2) - d(1) + r(1)) / (2 * (node(2, 1) - node(1, 1)));
        C = 1 + A * A;
        D = -2 * (node(1, 2) + A * node(1, 1) - A * B);
        E = d(1) - r(1) + B * B - 2 * B * node(1, 1) - 2 * node(1, 3) * zs + zs^2;
        rotate = true;
    end
    v = D * D - 4 * E * C;
    if v >= 0
        if ~rotate
            EstX(1) = (-1 * D + sqrt(v)) / (2 * C);
            EstY(1) = A * EstX(1) + B;
            EstX(2) = (-1 * D - sqrt(v)) / (2 * C);
            EstY(2) = A * EstX(2) + B;
        else
            EstY(1) = (-1 * D + sqrt(v)) / (2 * C);
            EstX(1) = A * EstY(1) + B;
            EstY(2) = (-1 * D - sqrt(v)) / (2 * C);
            EstX(2) = A * EstY(2) + B;
        end

        if len == 3
            % 如果寻找两个点到第三个平台更短的那个
            if pdist([node(3, :), [EstX(1), EstY(1), 0]]) > pdist([node(3, :), [EstX(2), EstY(2), 0]])
                EstX = EstX(2);
                EstY = EstY(2);
            else
                EstX = EstX(1);
                EstY = EstY(1);
            end
        else
            EstX = nan;
            EstY = nan;
        end
    elseif v < 0 || isnan(v)
        EstX = nan;
        EstY = nan;
    end
elseif len == 4
    d = zeros(len, 1);
    r = zeros(len, 1);
    for i = 1:len
        r(i) = sqrt(node(i, 1)^2+node(i, 2)^2+node(i, 3)^2); % d是平台到原点的距离的平方
        d(i) = timeDelay(i) * c; % r是平台到目标距离的平方
    end
    d21 = d(2) - d(1);
    d31 = d(3) - d(1);
    g11 = 2 * (node(2, 1) - node(1, 1));
    g21 = 2 * (node(3, 1) - node(1, 1));
    g12 = 2 * (node(2, 2) - node(1, 2));
    g22 = 2 * (node(3, 2) - node(1, 2));
    j1 = r(2)^2 - r(1)^2 - d21^2 - 2 * (node(2, 3) - node(1, 3)) * zs;
    j2 = r(3)^2 - r(1)^2 - d31^2 - 2 * (node(3, 3) - node(1, 3)) * zs;
    h1 = 2 * d21;
    h2 = 2 * d31;
    p = g11 * g22 - g21 * g12;
    px = (h2 * g12 - h1 * g22) / p;
    py = (h1 * g21 - h2 * g11) / p;
    qx = (j1 * g22 - j2 * g12) / p;
    qy = (j2 * g11 - j1 * g21) / p;
    aa = px * px + py * py - 1;
    bb = 2 * (px * qx - px * node(1, 1) + py * qy - py * node(1, 2));
    cc = (node(1, 1) - qx)^2 + (node(1, 2) - qy)^2 + (node(1, 3) - zs)^2;
    v = bb^2 - 4 * aa * cc;
    if v >= 0
        d1 = (-bb - sqrt(v)) / (2 * aa);
        d2 = (-bb + sqrt(v)) / (2 * aa);
        if d1 > 0 && d2 < 0
            EstX = px * d1 + qx;
            EstY = py * d1 + qy;
        elseif d1 < 0 && d2 > 0
            EstX = px * d2 + qx;
            EstY = py * d2 + qy;
        elseif d1 > 0 && d2 > 0
            EstX(1) = px * d1 + qx;
            EstY(1) = py * d1 + qy;
            EstX(2) = px * d2 + qx;
            EstY(2) = py * d2 + qy;
            % 这里需要冗余信息判解
            % 利用第4个平台的时延信息
            dis = pdist([node(4, :), [EstX(:), EstY(:), zeros(2, 0)]]);
            if dis(1) / c - timeDelay(4) < dis(2) / c - timeDelay(4)
                EstX = EstX(1);
                EstY = EstY(1);
            else
                EstX = EstX(2);
                EstY = EstY(2);
            end
        end
        % 这里可以获得目标点到各观测平台的距离 未加程序
    elseif v < 0 || isnan(v)
        EstX = nan;
        EstY = nan;
    end
end
res(:, 1) = EstX;
res(:, 2) = EstY;
loc = strings(1, len);
for i = 1 : len
    loc(i) = "p" + num2str(i);
end
% loc{1,:} = loc;
end

%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\AOARaw.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-11
% 描述:  目前未使用
% 纯方位交汇定位的经典解法，仅包含交汇定位部分，可由时空关联模块调用
% 输入: 各平台位置，各平台角度
% 输出:
%**************************************************************************


function res = AOARaw(angle, posPlatForm)
len = length(posPlatForm);
jiao = atan2(160, 500); %据双基元设置误差角度 在角度之外的才参与解算
% 开始排列组合
num_res = 1;
index = [];
for i = 1 : len
    for j = 1 : len
        if angle(j) > angle(i) && (abs(angle(j)-angle(i)) > jiao)
            tempx = AngleCross(posPlatForm(i, :), posPlatForm(j, :), ...
                                angle(i), angle(j));
            % 使用 isnan 函数检查矩阵中是否包含 NaN
            containsNaN = any(isnan(tempx));
            if containsNaN
                continue;
            else
                temp(num_res, :) = tempx;
                % 在这里去除nan的结果
                index = [index; temp(num_res, :)];
                num_res = num_res + 1;
            end
        end
    end
end
% 我先自己写一个加权的
% 1. 计算所有点的平均坐标
% 2. 计算距离的
% 结果中可能有nan，必须去除
% 计算所有点的平均 x 坐标和平均 y 坐标
averagePoint = mean(index, 1);
% 计算每个结果到平均值的距离,倒数作为权值
distances = sqrt((averagePoint(1) - index(:, 1)).^2 + ...
                 (averagePoint(2) - index(:, 2)).^2);
weights = 1 ./ distances;
% 对权值进行归一化
weights = weights / sum(weights);
% 打印权值
fprintf('权值：');
disp(weights);
% 计算加权平均的坐标
weighted_x = sum(index(:, 1) .* weights);
weighted_y = sum(index(:, 2) .* weights);
% 打印加权平均结果
fprintf('加权平均结果：(%.2f, %.2f)\n', weighted_x, weighted_y);
res = [weighted_x, weighted_y];
% % 这里的加权总和还得从头写，可以参考的王燕老师的。
% sum_x = 0;
% sum_y = 0;
% for i = 1:length(index)
%     lisan(i, 1) = index(i, 1);
%     lisan(i, 2) = index(i, 2);
%     lisan(i, 3) = sqrt((index(i, 1) - x(p, q))^2
%                       +(index(i, 2) - y(p, q))^2);
% end
% paixu = sortrows(lisan, 3); %升序排列
% for i = 1:3
%     sum_x = sum_x + paixu(i, 1);
%     sum_y = sum_y + paixu(i, 2);
% end
% xx = sum_x / 3;
% yy = sum_y / 3;


end

%% 两平台交汇定位 传入角度参数是弧度制
function res = AngleCross(pos1, pos2, angle1, angle2)
pos_x1 = pos1(1);
pos_y1 = pos1(2);
pos_x2 = pos2(1);
pos_y2 = pos2(2); %GPS测得两个平台位置

L = sqrt((pos_x1 - pos_x2)^2+(pos_y1 - pos_y2)^2); %基线长度
%基线与正东方向夹角 绝对值 以pi为单位
beta = abs(atan((pos_y2 - pos_y1)/(pos_x2 - pos_x1))); 

R1 = L * abs(cos(angle2+beta)/sin(angle2-angle1));
R2 = L * abs(cos(angle1+beta)/sin(angle2-angle1)); %计算得到的平台到目标的距离

xx1 = pos_x1 + R1 * sin(angle1);
xx2 = pos_x2 + R2 * sin(angle2);
yy1 = pos_y1 + R1 * cos(angle1);
yy2 = pos_y2 + R2 * cos(angle2);

res(1) = (xx1 + xx2) / 2;
res(2) = (yy1 + yy2) / 2;
end

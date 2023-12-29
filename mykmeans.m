%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mykmeans.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-28
% 描述: 抗异常参量，kmeans均值聚类，参考 李晴和桑志远论文
% 输入: 综合支持度数组, 质心门限
% 输出: idx 对综合支持度的分类结果
%**************************************************************************


%% 编写自己的kmeans函数

function [idx, centers] = mykmeans(w, gamma)
len = length(w);
gamma = 0.005;
iii = 1;
% 1. 选取综合支持度中最大和最小的两个作为聚类初始值
cen1 = max(w(:));
cen2 = min(w(:));
while iii == 1 || delta > gamma
    iii = iii + 1;
    % 2. 计算任意量对象间的综合支持度差值的绝对值
    %     f = zeros(len); % 记录当前点到综合支持度的差值
    idx = zeros(len, 1); % 分组结果
    for i = 1:len
        if abs(w(i)-cen1) <= abs(w(i)-cen2)
            idx(i) = 1;
        else
            idx(i) = 2;
        end
        %     f(i) = min(abs(w(i) - cen1), abs(w(i) - cen2));
    end

    % 3. 根据划分的结果，重新计算两个聚类的均值，并将其作为两个聚类新的中心值
    % 找到元素为1的索引
    indicesOfOnes = find(idx == 1);
    c1 = w(indicesOfOnes);
    % 获取元素为1的总数
    countOfOnes = length(indicesOfOnes);
    newcen1 = sum(c1) / countOfOnes;

    % 找到元素为2的索引
    indicesOfTwos = find(idx == 2);
    c2 = w(indicesOfTwos);
    % 获取元素为1的总数
    countOfTwos = length(indicesOfTwos);
    newcen2 = sum(c2) / countOfTwos;

    delta1 = abs(newcen1-cen1);
    delta2 = abs(newcen2-cen2);

    delta = max(delta1, delta2);
    cen1 = newcen1;
    cen2 = newcen2;
end
% 返回分组结果和聚类中心
centers = [cen1; cen2];
end
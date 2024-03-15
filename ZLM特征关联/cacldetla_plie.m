%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\cacldetla_pile.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-12
% 描述: 计算关联系数与灰色关联度函数
% 输入:
%	Xi:去量纲化后的目标特征序列矩阵
%	R：报文个数
%	canshu： 特征个数
%	k：当前计算的哪条报文
% 输出:
%	Rabrec：灰色关联度
%**************************************************************************

function [Rabrec] = cacldetla_plie(Xi, R, canshu, k)
%计算任意两个特征序列和之间的第i个特征差异值
for i = 1:R
    % delta(i,j)第i个向量和当前指定报文特征的差异值
    delta(i, :) = abs(Xi(i, :)-Xi(k, :));
    a = min(delta(i, :));
    b = max(delta(i, :));
    if (b == 0)
        b = 0.00000001;
    end
    ro = 0.5;
    yipusilo(i, :) = (a + ro * b) ./ (delta(i, :) + ro * b);
end
% 计算Xi(j)和Xk(j)的灰色关联系数
yipuxilo(find(isnan(yipusilo) == 1)) = 0;
yipusilo; % 9*6矩阵

%% 计算灰色关联度
% 定义灰色关联度为所有关联系数的平均值
Rabrec = (sum(yipusilo, 2) / canshu)'; % 是个1*9的矩阵
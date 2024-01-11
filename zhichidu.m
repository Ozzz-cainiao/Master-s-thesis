%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\zhichidu.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: 这是用来画kmeans算法的支持度的图的函数
% 输入:
% 输出:
%**************************************************************************

function zhichidu(res, idx)
% 找到包含nan或inf的行号
rowsToRemove = any(isnan(res), 2) | any(isinf(res), 2);
% 去除包含nan或inf的行
respie = res(~rowsToRemove, :);
mpie = size(respie, 1);
Apie = nan(mpie, mpie);

% 去除无解情况的矩阵A'
for i = 1:mpie
    for j = 1:mpie
        Apie(i, j) = pdist([respie(i, :); respie(j, :)]);
    end
end
% 重新计算Amax'
Amaxpie = max(Apie(:));
% 需要更新C1和C2
% 计算支持度基阵B
B = ones(mpie, mpie) - Apie ./ Amaxpie;
% 计算各初测坐标对于qm'的中和支持度
% 计算特征值和特征向量
[V, D] = eig(B);

% 获取特征值的对角线元素，D是一个对角矩阵
eigenvalues = diag(D);

% 找到最大模特征值的索引
[~, index] = max(abs(eigenvalues));

% 获取最大模特征值和对应的特征向量
%     maxEigenvalue = eigenvalues(index);
maxEigenvector = V(:, index);
% 支持度向量
w_mpie = zeros(1, mpie);
for i = 1:mpie
    w_mpie(i) = maxEigenvector(i) / sum(maxEigenvector);
end

% figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);
figure
h = plot(w_mpie, 'bo-'); % 返回图的句柄
% 在指定索引上更改数据点形状
for i = 1:length(idx)
    index = idx(i);
    h.Marker = 'o';  % 将所有数据点的形状先设置为默认的圆形
    hold on;
    plot(index, w_mpie(index), 'r*'); 
    hold off;
end
% 显示图例
legend('原始数据', 'C1');
title("综合支持度（筛选前后）")
end
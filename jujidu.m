%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\jujidu.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-28
% 描述: 对初测目标聚集度进行分析，然后调用kmeans抗异常参量，参考 李晴和桑志远论文
% 输入: 计算的初测坐标, loc是定位组合对， rho是聚集度参量
% 输出: 进行均值聚类抗异常参量后的坐标结果和异常参量
%**************************************************************************
function [respie] = jujidu(respie, locpie, rho, gamma)

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

% 判断一下Amaxpie和rho的关系
while Amaxpie > rho
    %% 计算支持度矩阵B
    B = ones(mpie, mpie) - Apie ./ Amaxpie;
    [V, D] = eig(B); % 计算特征值和特征向量
    eigenvalues = diag(D); % 获取特征值的对角线元素，D是一个对角矩阵
    [~, index] = max(abs(eigenvalues)); % 找到最大模特征值的索引

    %% 获取最大模特征值和对应的特征向量
    %     maxEigenvalue = eigenvalues(index);
    maxEigenvector = V(:, index);

    %% 计算支持度向量
    w_mpie = zeros(1, mpie);
    for i = 1:mpie
        w_mpie(i) = maxEigenvector(i) / sum(maxEigenvector);
    end

    %% 使用自己的kmeans函数
    [idx, ~] = mykmeans(w_mpie, gamma);

    %% 重新计算筛选后类标为C1的聚类中各综合支持度wpie所定位结果respie中间的最大距离Amaxpie
    respie = respie(idx == 1, :);
    mpie = size(respie, 1);
    Apie = zeros(mpie, mpie);

    %% 去除无解情况的矩阵A'
    for i = 1:mpie
        for j = 1:mpie
            Apie(i, j) = pdist([respie(i, :); respie(j, :)]);
        end
    end
    
    %% 更新Amaxpie
    Amaxpie = max(Apie(:));
end
end
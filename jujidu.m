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
% function [EstX, EstY] = jujidu(res, loc, rho)
function [respie] = jujidu(res, loc, rho, gamma)

m = size(res, 1);
%% 初测目标聚集度分析
C1 = ones(m, 1);
C2 = zeros(m, 1);
% 找到包含nan或inf的行号
rowsToRemove = any(isnan(res), 2) | any(isinf(res), 2);
C1(rowsToRemove, :) = 0;
C2(rowsToRemove, :) = 1;
% 去除包含nan或inf的行
respie = res(~rowsToRemove, :);
locpie = loc(~rowsToRemove, :);

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

%     figure 
%     plot(w_mpie, 'ro');
    %% 使用自己的kmeans函数
%     gamma = 0.005;
    [idx, ~] = mykmeans(w_mpie, gamma);

    %% 重新计算筛选后类标为C1的聚类中各综合支持度wpie所定位结果respie中间的最大距离Amaxpie
    respie = respie(idx == 1, :);
    mpie = size(respie, 1);
    Apie = zeros(mpie, mpie);
    % 去除无解情况的矩阵A'
    for i = 1:mpie
        for j = 1:mpie
            Apie(i, j) = pdist([respie(i, :); respie(j, :)]);
        end
    end
    Amaxpie = max(Apie(:));
end

EstX = mean(respie(:, 1));
EstY = mean(respie(:, 2));
%% 筛选出异常参量
% 当这个参量参与定位的所有解都被划分为C2时，这个量为异常参量

end
% 假设你有一个包含两个类别的数据集 X，其中每一行是一个样本，每一列是一个特征
X = [1 2; 2 3; 3 3; 4 5; 5 5];

% 对应的类别标签
Y = [1; 1; 1; 2; 2];

% 计算每个类别的均值向量
mu1 = mean(X(Y == 1, :));
mu2 = mean(X(Y == 2, :));

% 计算类内散布矩阵
S1 = cov(X(Y == 1, :));
S2 = cov(X(Y == 2, :));

% 计算类间散布矩阵
Sb = (mu1 - mu2)' * (mu1 - mu2);

% 计算投影方向
[V, D] = eig(inv(S1 + S2) * Sb);
projection_direction = V(:, end);

% 投影数据
X_projected = X * projection_direction;

% 绘制投影后的数据
figure;
scatter(X_projected(Y == 1), zeros(sum(Y == 1), 1), 'r', 'filled');
hold on;
scatter(X_projected(Y == 2), zeros(sum(Y == 2), 1), 'b', 'filled');
legend('Class 1', 'Class 2');
xlabel('Projected Dimension');
ylabel(' ');
title('LDA Projection');
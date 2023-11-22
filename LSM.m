%% 最小二乘法定位
%
% 这段MATLAB代码是一个最小二乘法定位算法的实现，用于通过方位角和观测节点的坐标来求解目标位置。
%
% 以下是对代码的解释：
%
% 1. 函数签名：`function [EstX, EstY] = LSM(Zt, node)`，这个函数接受两个输入参数：
%   `Zt`是方位角向量，`node`是观测节点的坐标矩阵。函数返回两个输出参数：`EstX`和`EstY`是求解得到的目标位置的估计值。
%
% 2. 数据预处理：`theta = Zt;`将输入的方位角向量赋值给变量`theta`。
%    然后通过`theta = theta(~isnan(Zt))';`去除其中的NaN值，得到一个非NaN的方位角向量。
%
% 3. 提取节点坐标：`x1 = node(~isnan(Zt), 1);`和`y1 = node(~isnan(Zt), 2);`
%    从观测节点的坐标矩阵中提取非NaN值对应的x和y坐标，分别保存在`x1`和`y1`中。
%
% 4. 构建最小二乘问题：通过观测数据构建最小二乘问题的矩阵形式。
%    `A = [-tand(theta), ones(length(x1), 1)];`构建了一个矩阵`A`，
%    其中第一列是`-tand(theta)`，第二列是全1的列向量，用于表示最小二乘问题的系数矩阵。
%    `B = y1 - x1 .* tand(theta);`构建了一个向量`B`，用于表示最小二乘问题的观测值。
%
% 5. 求解最小二乘问题：`X = (A' * A) \ A' * B;`使用最小二乘法求解问题，
%    通过计算 `(A' * A) \ A' * B` 得到目标位置的估计值。其中 `(A' * A) \ A'` 是最小二乘问题的正规方程的解。
%
% 6. 处理求解结果：通过判断求解结果是否为空，决定输出的目标位置估计值。
%    如果结果为空，即没有找到解，则将`EstX`和`EstY`设置为`inf`（无穷大）。
%    否则，将求解结果中的第一个元素赋值给`EstX`，第二个元素赋值给`EstY`，即得到目标位置的估计值。
%
% 总体而言，该代码实现了最小二乘法定位算法，通过使用方位角和观测节点的坐标来估计目标位置。

function [EstX, EstY] = LSM(Zt, node)
theta = Zt;
theta = theta(~isnan(Zt))';
x1 = node(~isnan(Zt), 1);
y1 = node(~isnan(Zt), 2);
A = [-tand(theta), ones(length(x1), 1)];
B = y1 - x1 .* tand(theta);
X = (A' * A) \ A' * B; % 目标位置X=[x;y]
if isempty(X)
    EstX = inf;
    EstY = inf;
else
    EstX = X(1);
    EstY = X(2);
end
end

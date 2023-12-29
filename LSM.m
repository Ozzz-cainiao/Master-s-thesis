%% LSM
% 输入 目标数*平台数
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
%% 参考链接 https://wenku.csdn.net/answer/cae8d6f669da494c81efd8f4e5dfda5a
function [x, fx, k] = trustregion(fun, x0, delta0, eta, maxiter)
% fun - 目标函数
% X0 - 初始点
% delta0 - 信赖域半径的初始值
% eta - 控制函数下降速度的参数
% maxiter - 最大迭代次数

x = x0; %初始点
fx = feval(fun, x); % 计目标函在部始点的医数值
delta = delta0; % 信赖域半径

k = 0; %送什次敬
while k < maxiter
    k = k + 1;

    % 计算梯度和海森矩阵
    [g, H] = getgradhess(fun, x);

    % 解决子问题，得到步长p和预测下降量pred
    [p, pred] = subproblem(g, H, delta);

    % 计算实际下降量act
    fxnew = feval(fun, x+p);
    act = fx - fxnew;

    %计算rho
    rho = act / pred;

    % 根据rho来调整信赖域半径delta
    if rho < 0.25
        delta = 0.25 * delta;
    elseif rho > 0.75 && abs(norm(p)-delta) < 1e-6
        delta = min(2*delta, delta0);
    end

    % 根据rho来更新x和x
    if rho > eta
        x = x + p;
        fx = fxnew;
    end

    % 判断是否收敛
    if norm(g) < 1e-6 || delta < 1e-6
        break;
    end
end
end

% 计算梯度和海森矩阵
function [g, H] = getgradhess(fun, x)
g = gradient(fun, x); %都度
H = hessian(fun, x); % 海森矩阵
end

%解决子问题
function [p, pred] = subproblem(g, H, delta)
% 将信赖域问题售化为二次规划
%min 1/2*p'*H*p +g'*p
% s.t. norm(p) <= delta

n = length(g); % 变量个数

%定义二次规划的Hessian和线性项
Hqp = H;
fqp = g;

% 定义不等式约束矩阵和右端向量
A = eye(n);
b = delta;
%源用MATLAB自意的二次规划求解器
options = optimoptions('quadprog', 'Display', 'off');
[p, ~, ~, ~, ~, ~, pred] = quadprog(Hqp, fqp, A, b, [], [], [], [], [], options);
end
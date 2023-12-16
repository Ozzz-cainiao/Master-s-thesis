syms x y dx dy
f = x^2 + y^2;
df = diff(f, x) * dx + diff(f, y) * dy;

% 假设 Δx 和 Δy 是已知的
Delta_x = 0.1; % 输入 x 的误差
Delta_y = 0.1; % 输入 y 的误差

% 代入误差
Delta_f = subs(df, [dx, dy], [Delta_x, Delta_y]);

% 计算可能的最大误差
max_error = abs(Delta_f)

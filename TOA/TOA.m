%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA\TOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-12
% 描述: 基于时延信息的定位技术，参考桑志远论文
% 输入:
% 输出:
%**************************************************************************

%% 桑志远 同步定位 不可用
clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 x4 y4 z1 z2 z3 z4 zs xs ys t1 t2 t3 t4 ts c xi yi zi ti
syms dx dy dz dt dc
f1 = (x1 - xs)^2 + (y1 - ys)^2 + (z1 - zs)^2 - (c * t1)^2;
f2 = (x2 - xs)^2 + (y2 - ys)^2 + (z2 - zs)^2 - (c * t2)^2;
f3 = (x3 - xs)^2 + (y3 - ys)^2 + (z3 - zs)^2 - (c * t3)^2;
f4 = (x4 - xs)^2 + (y4 - ys)^2 + (z4 - zs)^2 - (c * t4)^2;
% [sol_xs, sol_ys, sol_zs] = solve([f1 == 0, f2 == 0, f3 == 0, f4 == 0], [xs, ys, zs]);

fi = (xi - xs)^2 + (yi - ys)^2 + (zi - zs)^2 - (c * ti)^2;

f21 = simplify(f2-f1);
f31 = simplify(f3-f1);
f41 = simplify(f4-f1);
% 表示了结果
[sol_xs, sol_ys, sol_zs] = solve([f21 == 0, f31 == 0, f41 == 0], [xs, ys, zs]);


% 计算雅可比矩阵  等式——变量
J = jacobian([sol_xs, sol_ys, sol_zs], [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c]);
% 评估雅可比矩阵于特定点
% J_eval = subs(J, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c], [1000, 1000, 0]);
% J_eval = subs(J, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c], [-1e4, 0, 0, 1e4, 0, 0, 1e4, 1e4, 0, 0, 1e4, 0, 4.7, 4.7, 4.7, 4.7, 1500]);

% 误差向量 (Δx1, Δy1, Δz1, ...)
% error_vector = [Δx1, Δy1, Δz1, Δx2, Δy2, Δz2, Δx3, Δy3, Δz3, Δx4, Δy4, Δz4, Δt1, Δt2, Δt3, Δt4, Δc];
% error_vector = [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0];
% 误差向量 (Δx1, Δy1, Δz1, ...)
% error_vector = [dx, dy, dz, dt, dc];
error_vector = [dx, dy, dz, dx, dy, dz, dx, dy, dz, dx, dy, dz, dt, dt, dt, dt, dc];

% 误差传播
% errors = J_eval * error_vector';
errors = J * error_vector';
% 每个未知量的误差
error_xs = errors(1);
error_ys = errors(2);
error_zs = errors(3);
% 特定点的值
specific_point = [0, 0, 0, 1e4, 0, 0, 1e4, 1e4, 0, 0, 1e4, 0, 4.714045207910317, 4.714045207910317, 4.714045207910317, 4.714045207910317, 1500];

% 将符号变量替换为特定点的值
% error_xs_at_point = subs(error_xs, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c, dx, dy, dz, dt, dc], [specific_point, 1, 1, 0, 0, 0]);
% error_xs_at_point = subs(error_xs, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c, dx, dy, dz, dt, dc], [specific_point, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0]);
error_xs_at_point = subs(error_xs, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c], [specific_point]);

error_ys_at_point = subs(error_ys, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c, dx, dy, dz, dt, dc], [specific_point, 1, 1, 0, 0, 0]);

error_zs_at_point = subs(error_zs, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c, dx, dy, dz, dt, dc], [specific_point, 1, 1, 0, 0, 0]);

% 输出在特定点的误差
disp('Error at specific point:');
disp(['Error in xs: ', char(error_xs_at_point)]);
disp(['Error in ys: ', char(error_ys_at_point)]);
disp(['Error in zs: ', char(error_zs_at_point)]);

% 求全微分
df = diff(fi, xi) * dx + diff(fi, yi) * dy + diff(fi, zi) * dz + diff(fi, ti) * dt + diff(fi, c) * dc;

F = matlabFunction(df);
clearvars -except F


% 精度分析仿真
[x1, y1, x2, y2, x3, y3, x4, y4] = deal(0, 0, 1e4, 0, 1e4, 1e4, 0, 1e4); %% 平台位置
x = 0:10:1e4;
y = 0:10:1e4;
[lenx, leny] = deal(length(x), length(y));
c = 1500;
% 均方根误差
% 时延误差 2ms 阵位误差 2m 声速误差 1m/s
% errornor = [2,2,0.002^2,1.5^2];      % dx^2 dy^2 t1^2 c^2 误差
for ii = 1:lenx
    for jj = 1:leny
        m1 = F(c, err);

    end
end

%% 不可用
clc

% 符号变量
syms xs ys zs c t1 t2 t3 t4 x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4

% 方程组
f1 = (x1 - xs)^2 + (y1 - ys)^2 + (z1 - zs)^2 - (c * t1)^2;
f2 = (x2 - xs)^2 + (y2 - ys)^2 + (z2 - zs)^2 - (c * t2)^2;
f3 = (x3 - xs)^2 + (y3 - ys)^2 + (z3 - zs)^2 - (c * t3)^2;
f4 = (x4 - xs)^2 + (y4 - ys)^2 + (z4 - zs)^2 - (c * t4)^2;

% 雅可比矩阵
Jacobian = jacobian([f1, f2, f3, f4], [xs, ys, zs]);

% 设置已知量的值
[x1_val, y1_val, x2_val, y2_val, x3_val, y3_val, x4_val, y4_val, z1_val, z2_val, z3_val, z4_val] = deal(0, 0, 1e4, 0, 1e4, 1e4, 0, 1e4, 0, 0, 0, 0); %% 平台位置
c_val = 1500;

x = 0:10:1e4;
y = 0:10:1e4;
[lenx, leny] = deal(length(x), length(y));
for ii = 1:lenx
    for jj = 1:leny
        % 计算目标点到各平台的时延
        t1_val = sqrt((x1_val - x(ii))^2+(y1_val - y(jj))^2) / c;
        t2_val = sqrt((x2_val - x(ii))^2+(y2_val - y(jj))^2) / c;
        t3_val = sqrt((x3_val - x(ii))^2+(y3_val - y(jj))^2) / c;
        t4_val = sqrt((x4_val - x(ii))^2+(y4_val - y(jj))^2) / c;

        % 设置已知量的误差
        Delta_x = 1;
        Delta_y = 1;
        Delta_z = 0;
        Delta_t = 0;
        % 使用已知值替换方程组中的已知量
        f1_val = subs(f1, [x1, y1, z1, t1], [x1_val, y1_val, z1_val, t1_val]);
        f2_val = subs(f2, [x2, y2, z2, t2], [x2_val, y2_val, z2_val, t2_val]);
        f3_val = subs(f3, [x3, y3, z3, t3], [x3_val, y3_val, z3_val, t3_val]);
        f4_val = subs(f4, [x4, y4, z4, t4], [x4_val, y4_val, z4_val, t4_val]);
        % 计算雅可比矩阵
        Jacobian_val = jacobian([f1_val, f2_val, f3_val, f4_val], [xs, ys, zs]);
        % 构建输入误差向量
        Delta = [Delta_x; Delta_y; Delta_z; Delta_t];

        %         Delta = [Delta_x; Delta_y; Delta_z;Delta_t;Delta_x; Delta_y; Delta_z;Delta_t;Delta_x; Delta_y; Delta_z;Delta_t;Delta_x; Delta_y; Delta_z;Delta_t;];
        % 误差传播计算
        Delta_f = Jacobian_val * Delta;

    end
end

%% 不可用
clc
clear
% 符号变量
syms xs ys zs c t1 t2 t3 t4 x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4

% 方程组
f1 = (x1 - xs)^2 + (y1 - ys)^2 + (z1 - zs)^2 - (c * t1)^2;
f2 = (x2 - xs)^2 + (y2 - ys)^2 + (z2 - zs)^2 - (c * t2)^2;
f3 = (x3 - xs)^2 + (y3 - ys)^2 + (z3 - zs)^2 - (c * t3)^2;
f4 = (x4 - xs)^2 + (y4 - ys)^2 + (z4 - zs)^2 - (c * t4)^2;

% 构造方程组
equations = [f1; f2; f3; f4];

% 符号变量
variables = [xs; ys; zs];

% 计算雅可比矩阵
J = jacobian(equations, variables);

% 已知量
knowns = [c, t1, t2, t3, t4, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4];

% 将已知量代入方程组
equations_substituted = subs(equations, knowns);

% 求解雅可比矩阵的伪逆
J_pseudo_inv = pinv(subs(J, knowns));

% 计算未知量的变化
delta_variables = J_pseudo_inv * equations_substituted;

% 显示未知量的变化
disp('变化量：');
disp(delta_variables);

% 计算未知量的误差
variables_error = J_pseudo_inv * equations_substituted;

% 显示未知量的误差
disp('误差：');
disp(variables_error);

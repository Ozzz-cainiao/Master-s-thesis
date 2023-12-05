%% 
clc
clear

syms x1 y1 x2 y2 c xs ys theta t1 t2 ts a b d m n p

eq1 = (x1 - xs)^2 + (y1 - ys)^2 - c^2 * (t1 - ts)^2;
eq2 = (x2 - xs)^2 + (y2 - ys)^2 - c^2 * (t2 - ts)^2;
eq3 = (x2 - xs)^2 + (y2 - ys)^2 - ((x1 - xs)^2 + (y1 - ys)^2 + (x1 - x2)^2 + (y1 - y2)^2) + 2 * sqrt((x1 - xs)^2 + (y1 - ys)^2) * sqrt((x1 - x2)^2 + (y1 - y2)^2) * cos(theta);

% a = x1 - xs;
% b = y1 - ys;
% d = t1 - ts;
% m = x2 - x1;
% n = y2 - y1;
% p = t2 - t1;
eq1_revised = a^2 + b^2 - c^2 * d^2;
eq3_revised = (x2 - xs)^2 + (y2 - ys)^2 - (a^2 + b^2 + m^2 + n^2) + 2 * sqrt(a^2 + b^2) * sqrt(m^2 + n^2) * cos(theta);
% eq = [eq1, eq2, eq3];
eq12 = eq1 - eq2;
eq13 = eq1 - eq3;
eq23 = eq2 - eq3;
eq = [eq1, eq2, eq3];
solutions = solve(eq, [xs, ys, ts]);


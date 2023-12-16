%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA\TDOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-09
% 描述: 臧传斌 论文  TDOA
% 输入:
% 输出:
%**************************************************************************

%% 三平台
clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c
f1 = c * (t2 - t1 - (n2 - n1) * ts) - sqrt((x2 - xs)^2+(y2 - ys)^2) + sqrt((x1 - xs)^2+(y1 - ys)^2);
f2 = c * (t3 - t2 - (n3 - n2) * ts) - sqrt((x3 - xs)^2+(y3 - ys)^2) + sqrt((x2 - xs)^2+(y2 - ys)^2);

A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys)];
A_Mt = [diff(f1, t1), diff(f1, t2), diff(f1, t3); ...
    diff(f2, t1), diff(f2, t2), diff(f2, t3)];
A_Mc = [diff(f1, c); diff(f2, c)];

A_M1 = [diff(f1, x1), diff(f1, y1); diff(f2, x1), diff(f2, y1)];
A_M2 = [diff(f1, x2), diff(f1, y2); diff(f2, x2), diff(f2, y2)];
A_M3 = [diff(f1, x3), diff(f1, y3); diff(f2, x3), diff(f2, y3)];

M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Mc = matlabFunction(A_Mc);
clearvars -except M Mt Mc M1 M2 M3

% x = 0:10:2000;
% y = (0:10:2000)';
x = -1000:10:1000;
y = (-1000:10:1000)';
% [x1, y1, x2, y2, x3, y3] = deal(300, 300, 1700, 300, 1000, 1700); %% 平台位置
[x1, y1, x2, y2, x3, y3] = deal(-500, 500, 0, -500, 500, 500); %% 平台位置

[lenx, leny] = deal(length(x), length(y));

errornor = [0, 0, 0.001^2]; % x y ts^2误差  假设只有测时误差  已经是平方误差了
errorsum = zeros(lenx, leny);
c = 1500;
for ii = 1:lenx
    for jj = 1:leny
        m = M(x1, x2, x3, x(ii), y1, y2, y3, y(jj));
        mt = Mt(c);
        error = (m' * m) \ m' * (mt * diag([errornor(3), errornor(3), errornor(3)]) * mt') * m / (m' * m);
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y, abs(errorsum))
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('TDOA平面角解算误差');
clim([0, 30]); colormap jet
shading interp;view(0,90)

mean2(abs(errorsum(1:100, :)))


%% 臧传斌 论文  TDOA 暂不可用
% % 四平台
% % clc
% % clear
% % close all
% % % 求出解析解
% syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 t1 t2 t3 ts c
% f1 = (c * t1)^2 - sqrt((x1 - xs)^2+(y1 - ys)^2);
% f2 = (c * t2)^2 - sqrt((x2 - xs)^2+(y2 - ys)^2);
% f3 = (c * t3)^2 - sqrt((x3 - xs)^2+(y3 - ys)^2);
% % A_M = [diff(f1, xs), diff(f1, ys); diff(f2, xs), diff(f2, ys); diff(f3, xs), diff(f3, ys)];
% % A_Mt = [diff(f1, t1), diff(f1, t2), diff(f1, t3); ...
% %     diff(f2, t1), diff(f2, t2), diff(f2, t3);...
% %     diff(f3, t1), diff(f3, t2), diff(f3, t3)];
% %
% % M = matlabFunction(A_M);
% % Mt = matlabFunction(A_Mt);
% % clearvars -except M Mt
% %
% % x = 0:10:2000;
% % y = (0:10:2000)';
% %
% % % [x1, y1, x2, y2] = deal(300, 1000, 1700, 1000); %% 平台位置
% % [x1,y1,x2,y2,x3,y3] = deal(300,300,1700,300,1000,1700);%% 平台位置
% %
% % [lenx, leny] = deal(length(x), length(y));
% % errornor = [0, 0, 0.0001^2]; % x y ts^2误差  假设只有测时误差  已经是平方误差了
% %
% % % errornor = [0, 0, 0.001^2]; % x y ts^2误差  假设只有测时误差  已经是平方误差了
% % errorsum = zeros(lenx, leny);
% % c = 1500;
% % for ii = 1:lenx
% %     for jj = 1:leny
% %         t1 = sqrt((x1 - x(ii))^2+(y1 - y(jj))^2) / c;
% %         t2 = sqrt((x2 - x(ii))^2+(y2 - y(jj))^2) / c;
% %         t3 = sqrt((x3 - x(ii))^2+(y3 - y(jj))^2) / c;
% %         m = M(x1, x2, x3, x(ii), y1, y2,y3, y(jj));
% %         mt = Mt(c, t1, t2,t3);
% %         error = (m' * m) \ m' * (mt * diag([errornor(3), errornor(3), errornor(3)]) * mt') * m / (m' * m);
% %         errorsum(jj, ii) = sqrt(trace(error));
% %     end
% % end
% %
% % figure
% % surf(x, y, abs(errorsum))
% % set(gca, 'YDir', 'normal');
% % colorbar;
% % xlabel('x/m');
% % ylabel('y/m');
% % title('平面角解算误差');
% % clim([0, 1000]); colormap jet
% % shading interp;view(0,90)
% %
% % mean2(abs(errorsum(1:100, :)))
% 
% %% 杨卓 TDOA
% clc
% clear
% syms x1 y1 x2 y2 x3 y3 x4 y4 z1 z2 z3 z4 zs xs ys t1 t2 t3 t4 ts c xi yi zi ti
% syms dx dy dz dt dc
% f1 = (x1 - xs)^2 + (y1 - ys)^2 + (z1 - zs)^2 - (c * t1 - c * ts)^2;
% f2 = (x2 - xs)^2 + (y2 - ys)^2 + (z2 - zs)^2 - (c * t2 - c * ts)^2;
% f3 = (x3 - xs)^2 + (y3 - ys)^2 + (z3 - zs)^2 - (c * t3 - c * ts)^2;
% f4 = (x4 - xs)^2 + (y4 - ys)^2 + (z4 - zs)^2 - (c * t4 - c * ts)^2;
% 
% 
% f21 = simplify(f2-f1);
% f31 = simplify(f3-f1);
% f41 = simplify(f4-f1);
% 
% % 如果深度已知，则可以只采用f21 f31
% [sol_xs, sol_ys, sol_zs] = solve([f21 == 0, f31 == 0, f41 == 0], [xs, ys, zs]);
% d21 = ((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)^(1 / 2);
% d31 = ((x3 - x1)^2 + (y3 - y1)^2 + (z3 - z1)^2)^(1 / 2);
% 
% 
% res_x = subs(sol_xs, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, t1, t2, t3, t4, c], ...
%     [0, 0, 0, 1e4, 0, 0, 1e4, 1e4, 0, 0, 1e4, 0, 4.714045207910317, ...
%     4.714045207910317, 4.714045207910317, 4.714045207910317, 1500]);

%%  https://zhuanlan.zhihu.com/p/667934109
clc;
close all;
clear;
% 测向站数量
M = 6;
% 目标数量
N = 1;
% 6个测向站的位置坐标,单位米
% s1=[300 -100 150];
% s2=[-400 150 200];
% s3=[300 500 -300];
% s4=[350 200 100];
% s5=[-100 -100 -100];
% s6=[200 -300 -200];
s1 = [1200, 1800, 200];
s2 = [-1500, -800, 150];
s3 = [1400, -600, -200];
s4 = [-800, 1200, 120];
s5 = [1300, -800, -250];
s6 = [-1000, 1600, -150];
% 目标位置,单位米
s_all = [s1; s2; s3; s4; s5; s6];

% u = [2200, 1800, 2000].';
u = [8000, 6800, 3000].';
%
ri0 = zeros(M, 1);
for i = 1:1:M
    ri0(i, 1) = osjl(u, s_all(i, :).');
end
rij0 = ri0 - ri0(1);
%
Gt0 = zeros(M-1, 4);
ht0 = zeros(M-1, 1);
for i = 2:1:M
    Gt0(i-1, :) = -1 * [(s_all(i, :) - s_all(1, :)), rij0(i, 1)];
    ht0(i-1, 1) = 0.5 * ((rij0(i, 1))^2 - s_all(i, :) * s_all(i, :).' + s_all(1, :) * s_all(1, :).');
end
result11 = ht0 - Gt0 * [u; ri0(1)];
%
% 距离差对目标位置的导数
r_diff_u = zeros(M-1, 3);
for i = 2:1:M
    % 距离差对目标位置的导数
    ui_s1 = u.' - s_all(1, :);
    ui_si = u.' - s_all(i, :);
    r_diff_u(i-1, :) = ui_si / osjl(u.', s_all(i, :)) - ui_s1 / osjl(u.', s_all(1, :));
end
%
r_noise_power = 0.2:0.2:2;
big_loop_number = length(r_noise_power);
small_loop_number = 10000;
iter_number = zeros(big_loop_number, small_loop_number);
%
start_time = clock;
for big_loop = 1:1:big_loop_number
    cov_r = r_noise_power(big_loop)^2 * (eye(M-1) + ones(M-1, M-1)) / 2;

    %%
    F = r_diff_u.' * inv(cov_r) * r_diff_u;
    CRLB_u(big_loop) = sqrt(trace(inv(F)));
    for small_loop = 1:1:small_loop_number
        ri = ri0 + r_noise_power(big_loop) * randn(M, 1) / sqrt(2);
        rij = ri - ri(1);

        %%
        Gt1 = zeros(M-1, 4);
        ht1 = zeros(M-1, 1);
        for i = 2:1:M
            Gt1(i-1, :) = -1 * [(s_all(i, :) - s_all(1, :)), rij(i, 1)];
            ht1(i-1, 1) = 0.5 * ((rij(i, 1))^2 - s_all(i, :) * s_all(i, :).' + s_all(1, :) * s_all(1, :).');
        end
        yita_uls = pinv(Gt1) * ht1;
        u_uls = yita_uls(1:3);

        %%
        B1 = zeros(M-1, M-1);
        for i = 2:1:M
            B1(i-1, i-1) = osjl(u_uls, s_all(i, :).');
        end
        W1 = pinv(B1*cov_r*B1);
        yita_stage1 = pinv(Gt1.'*W1*Gt1) * Gt1.' * W1 * ht1;
        u_stage1 = yita_stage1(1:3);
        cov_stage1 = pinv(Gt1.'*W1*Gt1);
        MSE_u1(big_loop, small_loop) = (u_stage1 - u).' * (u_stage1 - u);

        %%
        G2 = [1, 0, 0; 0, 1, 0; 0, 0, 1; 1, 1, 1];
        h2 = [(u_stage1 - s_all(1, :).').^2; yita_stage1(4)^2];
        B2 = diag([u_stage1 - s_all(1, :).'; yita_stage1(4)]);
        W2 = pinv(4*B2*cov_stage1*B2);
        yita_stage2 = pinv(G2.'*W2*G2) * G2.' * W2 * h2;
        cov_stage2 = pinv(G2.'*W2*G2);
        u_stage2 = sqrt(yita_stage2(1:3)) + s_all(1, :).';
        MSE_u2(big_loop, small_loop) = (u_stage2 - u).' * (u_stage2 - u);

        %% 理论协方差矩阵
        B3 = diag(u_stage2-s_all(1, :).');
        cov_u = 0.25 * inv(B3) * cov_stage2 * inv(B3);
        MSE_cov_u(big_loop, small_loop) = trace(cov_u);

        %%
        [big_loop, small_loop];
    end
end
end_time = clock;
average_running_time = abs(etime(start_time, end_time)/small_loop_number/big_loop_number);

RMSE_u1 = sqrt(sum(MSE_u1, 2)/small_loop);
RMSE_u2 = sqrt(sum(MSE_u2, 2)/small_loop);
RMSE_cov_u = sqrt(sum(MSE_cov_u, 2)/small_loop);

% save('RMSE_TWLS_u.mat','RMSE_u2')
% save('RMSE_TWLS_cov_u.mat','RMSE_cov_u')
% save('TWLS_average_running_time.mat','average_running_time')


figure(1)
plot(r_noise_power, CRLB_u, 'r.-')
% hold on;
% plot(r_noise_power,RMSE_u1,'bo-')
hold on;
plot(r_noise_power, RMSE_u2, 'ks-')
% hold on;
% plot(r_noise_power,RMSE_cov_u,'ks')
% ylim([0 5])
xlabel('TDOA噪声(m)')
ylabel('RMSE(m)')


function [distance] = osjl(object, source)
% 输入均为列向量
distance = sqrt(sum((object - source).^2));
end

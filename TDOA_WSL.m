%% https://zhuanlan.zhihu.com/p/667934109

clc;
close all;
clear

%%
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

% u=[2200 1800 2000].';
u = [8000, 6800, 3000].';

%%
ri0 = zeros(M, 1);
for i = 1:1:M
    ri0(i, 1) = osjl(u, s_all(i, :).');
end
rij0 = ri0 - ri0(1);

%%
Gt0 = zeros(M-1, 4);
ht0 = zeros(M-1, 1);
for i = 2:1:M
    Gt0(i-1, :) = -1 * [(s_all(i, :) - s_all(1, :)), rij0(i, 1)];
    ht0(i-1, 1) = 0.5 * ((rij0(i, 1))^2 - s_all(i, :) * s_all(i, :).' + s_all(1, :) * s_all(1, :).');
end
result11 = ht0 - Gt0 * [u; ri0(1)];

%%
% 距离差对目标位置的导数
r_diff_u = zeros(M-1, 3);
for i = 2:1:M
    % 距离差对目标位置的导数
    ui_s1 = u.' - s_all(1, :);
    ui_si = u.' - s_all(i, :);
    r_diff_u(i-1, :) = ui_si / osjl(u.', s_all(i, :)) - ui_s1 / osjl(u.', s_all(1, :));
end

%%
r_noise_power = 0.2:0.2:2;
big_loop_number = length(r_noise_power);
small_loop_number = 10000;
iter_number = zeros(big_loop_number, small_loop_number);

%%
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
        [big_loop, small_loop]
    end
end
end_time = clock;
average_running_time = abs(etime(start_time, end_time)/small_loop_number/big_loop_number);

RMSE_u1 = sqrt(sum(MSE_u1, 2)/small_loop);
RMSE_u2 = sqrt(sum(MSE_u2, 2)/small_loop);
RMSE_cov_u = sqrt(sum(MSE_cov_u, 2)/small_loop);
figure
plot(MSE_cov_u)

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
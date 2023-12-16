clc;
close all;
clear

%% https://zhuanlan.zhihu.com/p/665460301
% 测向站数量
M = 6;
% 目标数量
N = 1;
% 6个测向站的位置坐标,单位米
s1 = [1200, 1800, 200].';
s2 = [-1500, -800, 150].';
s3 = [1400, -600, -200].';
s4 = [-800, 1200, 120].';
s5 = [1300, -800, -250].';
s6 = [-1000, 1600, -150].';
% 目标位置,单位米
% u1 = [8000, 6800, 3000].';
u1 = [8000, 6800, 0].';

%%
for i = 1:M
    %     组合字符串，得到接收站的变量名，以便循环
    str_si = ['s', mat2str(i)];
    %    返回变量名对应的值
    si_value = eval(str_si);
    %    测向站到目标的距离
    distance(i) = sqrt(sum((u1 - si_value).^2));
    %    测向站到目标的平均距离
    distance_average = sum(distance) / M;
    %     方位角真实值
    theta0(i) = atan((u1(1) - si_value(1))/(u1(2) - si_value(2)));
    %     仰角真实值
    beta0(i) = atan((u1(3) - si_value(3))/(sqrt((u1(1) - si_value(1))^2+(u1(2) - si_value(2))^2)));
end

%%
Ga0 = zeros(2*M, 3);
ha0 = zeros(2*M, 1);
for i = 1:1:M
    Ga10(i, :) = [cos(theta0(i)), -sin(theta0(i)), 0];
    Ga20(i, :) = [sin(theta0(i)) * sin(beta0(i)), cos(theta0(i)) * sin(beta0(i)), -cos(beta0(i))];

    %     组合字符串，得到接收站的变量名，以便循环
    str_si = ['s', mat2str(i)];
    %    返回变量名对应的值
    si_value = eval(str_si);
    ha10(i, :) = si_value(1) * cos(theta0(i)) - si_value(2) * sin(theta0(i));
    ha20(i, :) = si_value(1) * sin(theta0(i)) * sin(beta0(i)) + si_value(2) * cos(theta0(i)) * sin(beta0(i)) - si_value(3) * cos(beta0(i));
end
Ga0 = [Ga10; Ga20];
ha0 = [ha10; ha20];
result = Ga0 * u1 - ha0;

%%
% 误差的标准差，弧度
deta_theta_more = (0.1:0.2:2) .* pi / 180;
big_loop_numer = length(deta_theta_more);
% 独立实验次数
small_loop_numebr = 5000;

%%
% 误差向量，每一行为一个测向标准差下多次独立实验的误差
MSE_u = zeros(length(deta_theta_more), small_loop_numebr);
% 每一个测向标准差下的CRLB
CRLB = zeros(length(deta_theta_more), small_loop_numebr);

%%     对目标位置的导数
% 方位角、仰角对目标位置的导数
diff_theta_ui = zeros(M, 3, N);
diff_beta_ui = zeros(M, 3, N);
for j = 1:1:N
    for i = 1:1:M
        str_si = ['s', mat2str(i)];
        si_value = eval(str_si);

        ui_si = u1 - si_value;
        ai2 = sqrt(ui_si(1)^2+ui_si(2)^2);
        %     方位角对目标位置的导数
        diff_theta_ui_1 = -ui_si(2) / ai2^2;
        diff_theta_ui_2 = ui_si(1) / ai2^2;
        diff_theta_ui(i, :, j) = [diff_theta_ui_1, diff_theta_ui_2, 0];
        %     仰角对目标位置的导数
        diff_beta_ui_1 = -ui_si(1) * ui_si(3) / ai2 / (osjl(u1, si_value))^2;
        diff_beta_ui_2 = -ui_si(2) * ui_si(3) / ai2 / (osjl(u1, si_value))^2;
        diff_beta_ui_3 = ai2 / (osjl(u1, si_value))^2;
        diff_beta_ui(i, :, j) = [diff_beta_ui_1, diff_beta_ui_2, diff_beta_ui_3];
    end
end
F_u1 = [diff_theta_ui(:, :, 1); diff_beta_ui(:, :, 1)];
F_u = blkdiag(F_u1);
for big_loop = 1:length(deta_theta_more)
    deta_theta = deta_theta_more(big_loop);
    cov_z = deta_theta^2 * eye(2*M);
    % CRB界
    CRLB_DOA(big_loop) = sqrt(trace((F_u' * inv((cov_z)) * F_u)^-1));

    %%
    for small_loop = 1:1:small_loop_numebr
        %测向角,方位角叠加误差
        theta = theta0 + deta_theta * randn(1, M);
        beta = beta0 + deta_theta * randn(1, M);

        %%
        Ga = zeros(2*M, 3);
        ha = zeros(2*M, 1);
        for i = 1:1:M
            Ga1(i, :) = [cos(theta(i)), -sin(theta(i)), 0];
            Ga2(i, :) = [sin(theta(i)) * sin(beta(i)), cos(theta(i)) * sin(beta(i)), -cos(beta(i))];

            %     组合字符串，得到接收站的变量名，以便循环
            str_si = ['s', mat2str(i)];
            %    返回变量名对应的值
            si_value = eval(str_si);
            ha1(i, :) = si_value(1) * cos(theta(i)) - si_value(2) * sin(theta(i));
            ha2(i, :) = si_value(1) * sin(theta(i)) * sin(beta(i)) + si_value(2) * cos(theta(i)) * sin(beta(i)) - si_value(3) * cos(beta(i));
        end
        Ga = [Ga1; Ga2];
        ha = [ha1; ha2];

        %% 初始阶段
        W1 = inv(cov_z);
        u_uls = pinv(Ga) * ha;

        %%  ha对测量向量的导数
        A1 = zeros(M, M);
        A21 = zeros(M, M);
        A22 = zeros(M, M);
        for i = 1:1:M
            %     组合字符串，得到接收站的变量名，以便循环
            str_si = ['s', mat2str(i)];
            %    返回变量名对应的值
            si_value = eval(str_si);
            A1(i, i) = -si_value(1) * sin(theta(i)) - si_value(2) * cos(theta(i));
            A21(i, i) = si_value(1) * cos(theta(i)) * sin(beta(i)) - si_value(2) * sin(theta(i)) * sin(beta(i));
            A22(i, i) = si_value(1) * sin(theta(i)) * cos(beta(i)) + si_value(2) * cos(theta(i)) * cos(beta(i)) + si_value(3) * sin(beta(i));
        end
        A = [A1, zeros(M, M); A21, A22];

        %%  Ga对测量值的导数
        B = zeros(2*M, 2*M);
        for i = 1:1:M
            B1 = zeros(M, 3);
            B21 = zeros(M, 3);
            B1(i, :) = [-sin(theta(i)), -cos(theta(i)), 0;];
            B21(i, :) = [cos(theta(i)) * sin(beta(i)), -sin(theta(i)) * sin(beta(i)), 0];
            B(:, i) = [B1; B21] * u_uls;
        end
        for i = 1:1:M
            B22 = zeros(M, 3);
            B22(i, :) = [sin(theta(i)) * cos(beta(i)), cos(theta(i)) * cos(beta(i)), sin(beta(i))];
            B(:, i+6) = [zeros(M, 3); B22] * u_uls;
        end
        C = A - B;
        W2 = inv(C.'*cov_z*C);
        u_estimate = inv(Ga.'*W2*Ga) * Ga.' * W2 * ha;
        MSE_u(big_loop, small_loop) = sum((u_estimate - u1).^2);
        cov_u(big_loop, small_loop) = trace(inv(Ga.'*W2*Ga));

        %%
        [big_loop, small_loop]
    end
end
RMSE_u = sqrt(sum(MSE_u, 2)/small_loop_numebr);
RMSE_cov_u = sqrt(sum(cov_u, 2)/small_loop_numebr);

figure
plot(deta_theta_more./(pi / 180), CRLB_DOA, 'r.-');
hold on;
plot(deta_theta_more./(pi / 180), RMSE_u, 'ks-');
xlabel('测向误差标准差（°）');
ylabel('RMSE/m');
legend('CRLB', 'WLS算法')

%%
function [distance] = osjl(object, source)
% 输入均为列向量
distance = sqrt(sum((object - source).^2));
end
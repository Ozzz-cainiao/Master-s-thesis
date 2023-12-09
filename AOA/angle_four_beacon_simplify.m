close all
clear
clc

%% 参数设计
% positions = [500, 500, 0; 500, -500, 0; -500, -500, 0; -500, 500, 0];
positions = [500, 500; 500, -500; -500, -500; -500, 500];

z = 0;
[x, y] = meshgrid((-500:5:500)); %目标移动范围
diedai = 1000;

%% 误差设置
std_p = 1.5;
delta = normrnd(0, std_p, 8, diedai);

std_angle = 1 / 180 * pi; %单位为pi
delta_angle = normrnd(0, std_angle, 4, diedai);

jiao = atan2(160, 500); %据双基元设置误差角度 在角度之外的才参与解算

%% 纯方位交汇定位
n = size(x);
delta_R = zeros(n);

for p = 1:n(1)
    for q = 1:n(2)
        delta_r = zeros(1, diedai);
        for s = 1:diedai
            pos = positions + reshape(delta(:, s), 2, 4).';
%             pos(:, 3) = z;

            array_alpha = atan2(x(p, q) - pos(:, 1), y(p, q) - pos(:, 2)) + delta_angle(:, s);
            index = [];
            num_res = 1;

            for i_alpha = 1:4
                for j_alpha = (i_alpha+1):4
                    if abs(array_alpha(i_alpha) - array_alpha(j_alpha)) > jiao
                        res = AngleCross(pos(i_alpha, :), pos(j_alpha, :), array_alpha(i_alpha), array_alpha(j_alpha));
                        index(num_res, :) = res;
                        num_res = num_res + 1;
                    end
                end
            end

            sum_x = 0;
            sum_y = 0;
            lisan = zeros(length(index), 3);
            for i = 1:length(index)
                lisan(i, 1) = index(i, 1);
                lisan(i, 2) = index(i, 2);
                lisan(i, 3) = sqrt((index(i, 1) - x(p, q))^2+(index(i, 2) - y(p, q))^2);
            end
            paixu = sortrows(lisan, 3); %升序排列
            for i = 1:3
                sum_x = sum_x + paixu(i, 1);
                sum_y = sum_y + paixu(i, 2);
            end
            xx = sum_x / 3;
            yy = sum_y / 3;
            delta_r(s) = sqrt((xx - x(p, q))^2 + (yy - y(p, q))^2);
        end
        delta_R(p, q) = mean(delta_r);
    end
end

%% 画图
figure
h = pcolor(x, y, delta_R);
hold on;
plot(positions(:,1), positions(:,2), 'r*');
hold on;
set(h, 'edgecolor', 'none', 'facecolor', 'interp');
colorbar;
xlabel('x轴/m', 'FontSize', 14);
ylabel('y轴/m', 'FontSize', 14);

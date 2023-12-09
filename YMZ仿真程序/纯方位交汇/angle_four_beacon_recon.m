close all
clear
clc

%% 参数设计
[x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, z] = deal(500, 500, 0, ...
                                                            500, -500, 0, ...
                                                            -500, -500, 0, ...
                                                            -500, 500, 0, 0);


[x, y] = meshgrid((-500:5:500)); %目标移动范围
diedai = 1000;

%% 误差设置
std_p = 1.5;
delta = normrnd(0, std_p, 8, diedai);

std_angle = 1 / 180 * pi; % 单位为pi
delta_angles = normrnd(0, std_angle, 4, diedai);

jiao = atan2(160, 500); %据双基元设置误差角度 在角度之外的才参与解算

%% 纯方位交汇定位
n = size(x);
delta_R = zeros(n);
for p = 1:n(1)
    for q = 1:n(2)
        delta_r = zeros(1, diedai);
        for s = 1:diedai
            pos = [x1 + delta(1, s), y1 + delta(2, s), z1;
                x2 + delta(3, s), y2 + delta(4, s), z2;
                x3 + delta(5, s), y3 + delta(6, s), z3;
                x4 + delta(7, s), y4 + delta(8, s), z4];

            alpha = atan2(x(p, q) - pos(:, 1), y(p, q) - pos(:, 2)) + delta_angles(:, s);
%             array_alpha = [alpha1, alpha2, alpha3, alpha4]; %四个角 六种组合
%             array_pos = [pos1; pos2; pos3; pos4]; %四个浮标位置 六种组合
            num_res = 1;
            index = [];

            for i_alpha = 1:4
                for j_alpha = 1:4
                    if j_alpha > i_alpha
                        if abs(alpha(i_alpha)-alpha(j_alpha)) > jiao
                            res(num_res, :) = AngleCross(pos(i_alpha, :), pos(j_alpha, :), alpha(i_alpha), alpha(j_alpha));
                            index = [index; res(num_res, :)];
                            num_res = num_res + 1;
                        end
                    end
                end
            end
            %             res(1,:)=AngleCross(pos1,pos2,alpha1,alpha2);
            %             res(2,:)=AngleCross(pos1,pos3,alpha1,alpha3);
            %             res(3,:)=AngleCross(pos1,pos4,alpha1,alpha4);
            %             res(4,:)=AngleCross(pos2,pos3,alpha2,alpha3);
            %             res(5,:)=AngleCross(pos2,pos4,alpha2,alpha4);
            %             res(6,:)=AngleCross(pos3,pos4,alpha3,alpha4);

            sum_x = 0;
            sum_y = 0;
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

            delta_r(s) = sqrt((xx - x(p, q))^2+(yy - y(p, q))^2);
        end
        delta_R(p, q) = mean(delta_r);
    end
end

%% 画图
figure
h = pcolor(x, y, delta_R);
hold on;
plot(x1, y1, 'r*', x2, y2, 'r*', x3, y3, 'r*', x4, y4, 'r*');
hold on;
set(h, 'edgecolor', 'none', 'facecolor', 'interp');
colorbar;
% caxis([0 35]);
xlabel('x轴/m', 'FontSize', 14);
ylabel('y轴/m', 'FontSize', 14);
% title('纯方位交汇定位解算误差');
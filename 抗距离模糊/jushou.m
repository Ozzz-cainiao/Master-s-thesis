close all;clear all;clc;

%% 参数设置
x1 = 1000;
y1 = 1000;
z1 = 0; %第i个信标坐标，i=1,2,3,4
x2 = 1000;
y2 = -1000;
z2 = 0;
x3 = -1000;
y3 = -1000;
z3 = 0;
x4 = -1000;
y4 = 1000;
z4 = 0;
z = 0; %目标深度0

T = 0.1; %同步周期设为0.1s
v = 40; %目标航速40m/s
esv = 1500; %声速真实值

%% 目标轨迹预设
x_begin = -800;
y_begin = -500; %目标起始点（-800，800）

theta = pi / 6; %目标航行角度与正东呈30度

num = 400; %目标有400个轨迹点
for i = 1:num
    x(i) = x_begin + (i - 1) * v * T * cos(theta);
    y(i) = y_begin + (i - 1) * v * T * sin(theta);
end

%% 误差设置
std_t = 0.001;
std_p = 3;
std_c = 1;

length = 100;
delta_t1 = normrnd(0, std_t, 1, length); %随机生成一个均值为0，标准差为2 的1*1000随机数矩阵
delta_t2 = normrnd(0, std_t, 1, length);
delta_t3 = normrnd(0, std_t, 1, length);
delta_t4 = normrnd(0, std_t, 1, length);

delta_x1 = normrnd(0, std_p, 1, length);
delta_y1 = normrnd(0, std_p, 1, length);
dz1 = 0;
delta_x2 = normrnd(0, std_p, 1, length);
delta_y2 = normrnd(0, std_p, 1, length);
dz2 = 0;
delta_x3 = normrnd(0, std_p, 1, length);
delta_y3 = normrnd(0, std_p, 1, length);
dz3 = 0;
delta_x4 = normrnd(0, std_p, 1, length);
delta_y4 = normrnd(0, std_p, 1, length);
dz4 = 0;

delta_c = normrnd(0, std_c, 1, length);

%% 模拟声学参数测量
%浮标GPS上传位置
pos_x1 = x1 + delta_x1;
pos_y1 = y1 + delta_y1; %100次迭代
pos_x2 = x2 + delta_x2;
pos_y2 = y2 + delta_y2;
pos_x3 = x3 + delta_x3;
pos_y3 = y3 + delta_y3;
pos_x4 = x4 + delta_x4;
pos_y4 = y4 + delta_y4;
for i = 1:length
    pos1(i, :) = [pos_x1(i), pos_y1(i), z1]; %100次迭代×[x y z]坐标
    pos2(i, :) = [pos_x2(i), pos_y2(i), z2];
    pos3(i, :) = [pos_x3(i), pos_y3(i), z3];
    pos4(i, :) = [pos_x4(i), pos_y4(i), z4];
end
%加入误差后的声速
c = esv + delta_c; %100次迭代
%实际距离
dis1 = sqrt((x - x1).^2+(y - y1).^2); %400个目标航迹
dis2 = sqrt((x - x2).^2+(y - y2).^2);
dis3 = sqrt((x - x3).^2+(y - y3).^2);
dis4 = sqrt((x - x4).^2+(y - y4).^2);
%实际时延
time1 = dis1 ./ esv;
time2 = dis2 ./ esv;
time3 = dis3 ./ esv;
time4 = dis4 ./ esv;
%加入误差后的时延
for i = 1:num
    t1(i, :) = time1(i) + delta_t1; %400个点×100次迭代
    t2(i, :) = time2(i) + delta_t2;
    t3(i, :) = time3(i) + delta_t3;
    t4(i, :) = time4(i) + delta_t4;
end
%同步周期相对时延
tb_t1 = mod(t1, T); %400个点×100次迭代
tb_t2 = mod(t2, T);
tb_t3 = mod(t3, T);
tb_t4 = mod(t4, T);

%% 举手表决法
load mohu_res;
res_part = res([4201:12600], :);
res_in = flipud(res_part); % 翻转所有元素
nn = 1;
s = length;
res_jushou = [];
for p = 1:num
    kk = 1;
    sel_res = [];
    for mohu1 = 0:20
        tao4 = sqrt((res_in((p - 1)*21+mohu1+1, 1) - pos_x3(s))^2+(res_in((p - 1)*21+mohu1+1, 2) - pos_y3(s))^2) / c(s);
        tao = abs(mod(tao4, T)-tb_t3(p, s));
        if tao < 0.005 * T
            sel_res(kk, :) = res_in((p - 1)*21+mohu1+1, :);
            kk = kk + 1;
        end
    end
    n = size(sel_res);
    if n(1) == 1
        res_jushou(nn, 1) = sel_res(1, 1);
        res_jushou(nn, 2) = sel_res(1, 2);
        nn = nn + 1;
    else
        if nn < 3
            min_tao = T;
            for i = 1:n(1)
                tao4 = sqrt((sel_res(i, 1) - pos_x3(s))^2+(sel_res(i, 2) - pos_y3(s))^2) / c(s);
                tao = abs(mod(tao4, T)-tb_t3(p, s));
                if tao < min_tao
                    res_jushou(nn, 1) = sel_res(i, 1);
                    res_jushou(nn, 2) = sel_res(i, 2);
                    min_tao = tao;
                end
            end
            if min_tao < T
                nn = nn + 1;
            end
        else
            next_x = 2 * res_jushou(nn-1, 1) - res_jushou(nn-2, 1);
            next_y = 2 * res_jushou(nn-1, 2) - res_jushou(nn-2, 2);
            min_dis = 1000;
            for i = 1:n(1)
                dis = sqrt((sel_res(i, 1) - next_x)^2+(sel_res(i, 2) - next_y)^2);
                if dis < min_dis
                    res_jushou(nn, 1) = sel_res(i, 1);
                    res_jushou(nn, 2) = sel_res(i, 2);
                    min_dis = dis;
                end
            end
            if min_dis < 1000
                nn = nn + 1;
            end
        end
    end
end

%% 画图
figure
plot(x, y, 'b*', 'MarkerSize', 3); hold on %目标真实轨迹
plot(res_jushou(:, 1), res_jushou(:, 2), 'r.'); hold on %抗模糊轨迹
plot(x1, y1, 'ro', x2, y2, 'ro', x3, y3, 'ro', x4, y4, 'ro'); hold on
axis equal
axis([-1000, 1000, -1000, 1000]);
xlabel('x轴/m', 'FontSize', 14);
ylabel('y轴/m', 'FontSize', 14);
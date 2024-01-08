close all;clear all;clc;

%% ��������
x1 = 1000;
y1 = 1000;
z1 = 0; %��i���ű����꣬i=1,2,3,4
x2 = 1000;
y2 = -1000;
z2 = 0;
x3 = -1000;
y3 = -1000;
z3 = 0;
x4 = -1000;
y4 = 1000;
z4 = 0;
z = 0; %Ŀ�����0

T = 0.1; %ͬ��������Ϊ0.1s
v = 40; %Ŀ�꺽��40m/s
esv = 1500; %������ʵֵ

%% Ŀ��켣Ԥ��
x_begin = -800;
y_begin = -500; %Ŀ����ʼ�㣨-800��800��

theta = pi / 6; %Ŀ�꺽�нǶ���������30��

num = 400; %Ŀ����400���켣��
for i = 1:num
    x(i) = x_begin + (i - 1) * v * T * cos(theta);
    y(i) = y_begin + (i - 1) * v * T * sin(theta);
end

%% �������
std_t = 0.001;
std_p = 3;
std_c = 1;

length = 100;
delta_t1 = normrnd(0, std_t, 1, length); %�������һ����ֵΪ0����׼��Ϊ2 ��1*1000���������
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

%% ģ����ѧ��������
%����GPS�ϴ�λ��
pos_x1 = x1 + delta_x1;
pos_y1 = y1 + delta_y1; %100�ε���
pos_x2 = x2 + delta_x2;
pos_y2 = y2 + delta_y2;
pos_x3 = x3 + delta_x3;
pos_y3 = y3 + delta_y3;
pos_x4 = x4 + delta_x4;
pos_y4 = y4 + delta_y4;
for i = 1:length
    pos1(i, :) = [pos_x1(i), pos_y1(i), z1]; %100�ε�����[x y z]����
    pos2(i, :) = [pos_x2(i), pos_y2(i), z2];
    pos3(i, :) = [pos_x3(i), pos_y3(i), z3];
    pos4(i, :) = [pos_x4(i), pos_y4(i), z4];
end
%�������������
c = esv + delta_c; %100�ε���
%ʵ�ʾ���
dis1 = sqrt((x - x1).^2+(y - y1).^2); %400��Ŀ�꺽��
dis2 = sqrt((x - x2).^2+(y - y2).^2);
dis3 = sqrt((x - x3).^2+(y - y3).^2);
dis4 = sqrt((x - x4).^2+(y - y4).^2);
%ʵ��ʱ��
time1 = dis1 ./ esv;
time2 = dis2 ./ esv;
time3 = dis3 ./ esv;
time4 = dis4 ./ esv;
%���������ʱ��
for i = 1:num
    t1(i, :) = time1(i) + delta_t1; %400�����100�ε���
    t2(i, :) = time2(i) + delta_t2;
    t3(i, :) = time3(i) + delta_t3;
    t4(i, :) = time4(i) + delta_t4;
end
%ͬ���������ʱ��
tb_t1 = mod(t1, T); %400�����100�ε���
tb_t2 = mod(t2, T);
tb_t3 = mod(t3, T);
tb_t4 = mod(t4, T);

%% ���ֱ����
load mohu_res;
res_part = res([4201:12600], :);
res_in = flipud(res_part); % ��ת����Ԫ��
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

%% ��ͼ
figure
plot(x, y, 'b*', 'MarkerSize', 3); hold on %Ŀ����ʵ�켣
plot(res_jushou(:, 1), res_jushou(:, 2), 'r.'); hold on %��ģ���켣
plot(x1, y1, 'ro', x2, y2, 'ro', x3, y3, 'ro', x4, y4, 'ro'); hold on
axis equal
axis([-1000, 1000, -1000, 1000]);
xlabel('x��/m', 'FontSize', 14);
ylabel('y��/m', 'FontSize', 14);
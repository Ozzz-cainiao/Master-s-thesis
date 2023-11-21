%% 浮标上传的数据结构  参数级
% 平台号、周期、平台经纬度、有效目标数、目标1到目标4
% 目标1的结构体
% 方位、幅度、10个线谱频率和幅度、3个连续谱的幅度和频率、demon谱特征：桨叶数基频

%% 产生目标1的轨迹
clear
close all
clc

% 轨迹设置
[x0, y0] = deal(250, 500);
v = 10;
a1 = 0.1;
a2 = 0.5;
ta = 0;
tb = 120;
dt = 0.1;
t = ta : dt : tb;
XX1 = x0+ a1.* t.* t + v.* t;
YY1 = y0 + 0.5 * a2.* t.*t + v.*t;
figure
plot(XX1, YY1);
title("设定轨迹");grid on; hold on;

[x1, y1,x2,y2, x3,y3,x4,y4,x5,y5] = deal(0,0,3000, 0, 3000, 3000, 0, 5000, 1500, 4000);

arrX = [x1,x2,x3,x4,x5];
arrY = [y1, y2, y3, y4, y5];
plot(arrX, arrY, 'r*');
hold on;


%% 产生目标2 的轨迹

% 轨迹设置
[x0, y0] = deal(2000, 3200);
v = -10;
a1 = 0.1;
a2 = 0.5;
ta = 0;
tb = 120;
dt = 0.1;
t = ta : dt : tb;
XX2 = x0+ a1.* t.* t + v.* t;
YY2 = y0 + 0.5 * a2.* t.*t + v.*t;
figure
plot(XX2, YY2);
title("设定轨迹2");grid on; hold on;

[x1, y1,x2,y2, x3,y3,x4,y4,x5,y5] = deal(0,0,3000, 0, 3000, 3000, 0, 5000, 1500, 4000);

arrX = [x1,x2,x3,x4,x5];
arrY = [y1, y2, y3, y4, y5];
plot(arrX, arrY, 'r*');
hold on;

%% 求出目标1到各个平台的角度
theta1 = [atan2(XX1, YY1);
    atan2((XX1-x2),(YY1-y2));
    atan2((XX1-x3),(YY1-y3));
    atan2((XX1-x4),(YY1-y4));
    atan2((XX1-x5),(YY1-y5))];%和Y正的夹角 顺时针 弧度值
%% 求出目标2到各个平台的角度
theta2 = [atan2(XX2, YY2);
    atan2((XX2-x2),(YY2-y2));
    atan2((XX2-x3),(YY2-y3));
    atan2((XX2-x4),(YY2-y4));
    atan2((XX2-x5),(YY2-y5))];%和Y正的夹角 顺时针 弧度值

%% 导出计算的方位角


%% 加入目标1 的传播时延
c = 1500;
T = 1;%每1秒获得一次方位信息
% 节点1
for i = 1 : 5
    t1(i,:) = sqrt((XX1 - arrX(i)).^2+(YY1 - arrY(i)).^2)./c + t;
end

%% 加入目标2 的传播时延
c = 1500;
T = 1;%每1秒获得一次方位信息
% 节点1
for i = 1 : 5
    t2(i,:) = sqrt((XX2 - arrX(i)).^2+(YY2 - arrY(i)).^2)./c + t;
end

%% 两个目标一起考虑到达各个观测节点的时间
themin = [min(t1(1,:)),min(t1(2,:)),min(t1(3,:)),min(t1(4,:)),min(t1(5,:)), min(t2(1,:)), min(t2(2,:)), min(t2(3,:)), min(t2(4,:)), min(t2(5,:))];
themax = [max(t1(1,:)),max(t1(2,:)),max(t1(3,:)),max(t1(4,:)),max(t1(5,:)), max(t2(1,:)), max(t2(2,:)), max(t2(3,:)), max(t2(4,:)), max(t2(5,:))];
tt = max(themin) : T : min(themax);

for i = 1 : 5 
    [~, loc1(i,:)] = min(abs(t1(i,:)' - tt));
    [~, loc2(i,:)] = min(abs(t2(i,:)' - tt));
end


for ii = i:length(loc1)
    for jj = 1 : 5
        newtheta1(jj, ii) = theta1(i, loc1(jj,ii));
        newtheta2(jj, ii) = theta2(i, loc2(jj,ii));
    end
end

nnntheta1 = newtheta1(1, :) ./pi * 180;
figure;
plot(nnntheta1);
figure;
nnntheta2 = newtheta2(2,:) ./ pi *180;
plot(nnntheta2);


%% 各浮标的经纬度位置
lon = [120,120.031,120.032, 120.001,120.017];
lat = [30.0, 29.9993,30.0263,30.045,30.0357];


%% 设置5个UDP服务器模拟5个浮标
u1 = udp('127.0.0.1', 'RemotePort', 8080, 'LocalPort', 8833); %outputbuffersize = 1200;
u2 = udp('127.0.0.1', 'RemotePort', 8080, 'LocalPort', 8833); %outputbuffersize = 1200;
u3 = udp('127.0.0.1', 'RemotePort', 8080, 'LocalPort', 8833); %outputbuffersize = 1200;
u4 = udp('127.0.0.1', 'RemotePort', 8080, 'LocalPort', 8833); %outputbuffersize = 1200;
u5 = udp('127.0.0.1', 'RemotePort', 8080, 'LocalPort', 8833); %outputbuffersize = 1200;

uu = [u1, u2,u3, u4, u5];
% fopen(u1);

time = length(loc1(1));
for fram = 1 : time
    pause(1)
    for index = 1 : 5
        TIME = datetime("now");
        tempValue.NodeIndex = uint8(index);%节点编号
        tempValue.sendTime = typecast(single(clock()),'uint8');%发送时刻
        tempValue.NodeLongHead = typecast(single(lon(index)),'uint8');%经度
        tempValue.NodeLatHead = typecast(single(lat(index)),'uint8');%纬度
        tempValue.ValidNum = uint8(2);% 有效目标数


        %% 目标1 方位 幅度 10个线谱 3个连续谱 1个demon谱
        tempValue.node(1).orientation = typecast(single(newtheta1(index, fram)),'uint8');
        tempValue.node(1).amplitude_V = typecast(single(100),'uint8');

        % 10个线谱
        for a = 1 : 10
            tempValue.node(1).line(a).fre = typecast(uint16(10 * a), 'uint8');
            tempValue.node(1).line(a).ampdB = typecast(single(20 * a),'uint8');
        end

        % 3个连续谱
        for aa = 1 : 3
            tempValue.node(1).Conti(aa).fre = typecast(uint16(50 * aa), 'uint8');
            tempValue.node(1).Conti(aa).ampdB = typecast(single(15 * aa),'uint8');
        end

        % 1个demon谱
       
        tempValue.node(1).demon.BladeNum = uint8(5);
        tempValue.node(1).demon.baseFre = typecast(single(110),'uint8');
        
        %% 目标2 方位 幅度 10个线谱 3个连续谱 1个demon谱
        tempValue.node(2).orientation = typecast(single(newtheta2(index, fram)),'uint8');
        tempValue.node(2).amplitude_V = typecast(single(100),'uint8');

        % 10个线谱
        for a = 1 : 10
            tempValue.node(2).line(a).fre = typecast(uint16(10 * a), 'uint8');
            tempValue.node(2).line(a).ampdB = typecast(single(20 * a),'uint8');
        end

        % 3个连续谱
        for aa = 1 : 3
            tempValue.node(2).Conti(aa).fre = typecast(uint16(20 * aa), 'uint8');
            tempValue.node(2).Conti(aa).ampdB = typecast(single(15 * aa),'uint8');
        end

        % 1个demon谱
       
        tempValue.node(2).demon.BladeNum = uint8(5);
        tempValue.node(2).demon.baseFre = typecast(single(10),'uint8');

        %% 目标3 - 5 不存在，都设为0 方位 幅度 10个线谱 3个连续谱 1个demon谱
        for iii = 3 : 5
            tempValue.node(iii).orientation = typecast(single(0),'uint8');
            tempValue.node(iii).amplitude_V = typecast(single(0),'uint8');
    
            % 10个线谱
            for a = 1 : 10
                tempValue.node(iii).line(a).fre = typecast(uint16(0), 'uint8');
                tempValue.node(iii).line(a).ampdB = typecast(single(0),'uint8');
            end
    
            % 3个连续谱
            for aa = 1 : 3
                tempValue.node(iii).Conti(aa).fre = typecast(uint16(0), 'uint8');
                tempValue.node(iii).Conti(aa).ampdB = typecast(single(0),'uint8');
            end
    
            % 1个demon谱
           
            tempValue.node(iii).demon.BladeNum = uint8(0);
            tempValue.node(iii).demon.baseFre = typecast(single(0),'uint8');
        end

        %%  转成数组
        temp = [tempValue.NodeIndex,tempValue.sendTime,tempValue.NodeLongHead,...
            tempValue.NodeLatHead,tempValue.ValidNum,...
            tempValue.node(1).orientation,tempValue.node(1).amplitude_V,...
            tempValue.node(1).line(1).fre,tempValue.node(1).line(1).ampdB,...
            tempValue.node(1).line(2).fre,tempValue.node(1).line(2).ampdB,...
            tempValue.node(1).line(3).fre,tempValue.node(1).line(3).ampdB,...
            tempValue.node(1).line(4).fre,tempValue.node(1).line(4).ampdB,...
            tempValue.node(1).line(5).fre,tempValue.node(1).line(5).ampdB,...
            tempValue.node(1).line(6).fre,tempValue.node(1).line(6).ampdB,...
            tempValue.node(1).line(7).fre,tempValue.node(1).line(7).ampdB,...
            tempValue.node(1).line(8).fre,tempValue.node(1).line(8).ampdB,...
            tempValue.node(1).line(9).fre,tempValue.node(1).line(9).ampdB,...
            tempValue.node(1).line(10).fre,tempValue.node(1).line(10).ampdB,...
            tempValue.node(1).Conti(1).fre,tempValue.node(1).Conti(1).ampdB,...
            tempValue.node(1).Conti(2).fre,tempValue.node(1).Conti(2).ampdB,...
            tempValue.node(1).Conti(3).fre,tempValue.node(1).Conti(3).ampdB,...
            tempValue.node(1).demon.BladeNum,tempValue.node(1).demon.baseFre,...
            tempValue.node(2).line(1).fre,tempValue.node(2).line(1).ampdB,...
            tempValue.node(2).line(2).fre,tempValue.node(2).line(2).ampdB,...
            tempValue.node(2).line(3).fre,tempValue.node(2).line(3).ampdB,...
            tempValue.node(2).line(4).fre,tempValue.node(2).line(4).ampdB,...
            tempValue.node(2).line(5).fre,tempValue.node(2).line(5).ampdB,...
            tempValue.node(2).line(6).fre,tempValue.node(2).line(6).ampdB,...
            tempValue.node(2).line(7).fre,tempValue.node(2).line(7).ampdB,...
            tempValue.node(2).line(8).fre,tempValue.node(2).line(8).ampdB,...
            tempValue.node(2).line(9).fre,tempValue.node(2).line(9).ampdB,...
            tempValue.node(2).line(10).fre,tempValue.node(2).line(10).ampdB,...
            tempValue.node(2).Conti(1).fre,tempValue.node(2).Conti(1).ampdB,...
            tempValue.node(2).Conti(2).fre,tempValue.node(2).Conti(2).ampdB,...
            tempValue.node(2).Conti(3).fre,tempValue.node(2).Conti(3).ampdB,...
            tempValue.node(2).demon.BladeNum,tempValue.node(2).demon.baseFre,...
            tempValue.node(3).line(1).fre,tempValue.node(3).line(1).ampdB,...
            tempValue.node(3).line(2).fre,tempValue.node(3).line(2).ampdB,...
            tempValue.node(3).line(3).fre,tempValue.node(3).line(3).ampdB,...
            tempValue.node(3).line(4).fre,tempValue.node(3).line(4).ampdB,...
            tempValue.node(3).line(5).fre,tempValue.node(3).line(5).ampdB,...
            tempValue.node(3).line(6).fre,tempValue.node(3).line(6).ampdB,...
            tempValue.node(3).line(7).fre,tempValue.node(3).line(7).ampdB,...
            tempValue.node(3).line(8).fre,tempValue.node(3).line(8).ampdB,...
            tempValue.node(3).line(9).fre,tempValue.node(3).line(9).ampdB,...
            tempValue.node(3).line(10).fre,tempValue.node(3).line(10).ampdB,...
            tempValue.node(3).Conti(1).fre,tempValue.node(3).Conti(1).ampdB,...
            tempValue.node(3).Conti(2).fre,tempValue.node(3).Conti(2).ampdB,...
            tempValue.node(3).Conti(3).fre,tempValue.node(3).Conti(3).ampdB,...
            tempValue.node(3).demon.BladeNum,tempValue.node(3).demon.baseFre,...
            tempValue.node(4).line(1).fre,tempValue.node(4).line(1).ampdB,...
            tempValue.node(4).line(2).fre,tempValue.node(4).line(2).ampdB,...
            tempValue.node(4).line(3).fre,tempValue.node(4).line(3).ampdB,...
            tempValue.node(4).line(4).fre,tempValue.node(4).line(4).ampdB,...
            tempValue.node(4).line(5).fre,tempValue.node(4).line(5).ampdB,...
            tempValue.node(4).line(6).fre,tempValue.node(4).line(6).ampdB,...
            tempValue.node(4).line(7).fre,tempValue.node(4).line(7).ampdB,...
            tempValue.node(4).line(8).fre,tempValue.node(4).line(8).ampdB,...
            tempValue.node(4).line(9).fre,tempValue.node(4).line(9).ampdB,...
            tempValue.node(4).line(10).fre,tempValue.node(4).line(10).ampdB,...
            tempValue.node(4).Conti(1).fre,tempValue.node(4).Conti(1).ampdB,...
            tempValue.node(4).Conti(2).fre,tempValue.node(4).Conti(2).ampdB,...
            tempValue.node(4).Conti(3).fre,tempValue.node(4).Conti(3).ampdB,...
            tempValue.node(4).demon.BladeNum,tempValue.node(4).demon.baseFre,...
            tempValue.node(5).line(1).fre,tempValue.node(5).line(1).ampdB,...
            tempValue.node(5).line(2).fre,tempValue.node(5).line(2).ampdB,...
            tempValue.node(5).line(3).fre,tempValue.node(5).line(3).ampdB,...
            tempValue.node(5).line(4).fre,tempValue.node(5).line(4).ampdB,...
            tempValue.node(5).line(5).fre,tempValue.node(5).line(5).ampdB,...
            tempValue.node(5).line(6).fre,tempValue.node(5).line(6).ampdB,...
            tempValue.node(5).line(7).fre,tempValue.node(5).line(7).ampdB,...
            tempValue.node(5).line(8).fre,tempValue.node(5).line(8).ampdB,...
            tempValue.node(5).line(9).fre,tempValue.node(5).line(9).ampdB,...
            tempValue.node(5).line(10).fre,tempValue.node(5).line(10).ampdB,...
            tempValue.node(5).Conti(1).fre,tempValue.node(5).Conti(1).ampdB,...
            tempValue.node(5).Conti(2).fre,tempValue.node(5).Conti(2).ampdB,...
            tempValue.node(5).Conti(3).fre,tempValue.node(5).Conti(3).ampdB,...
            tempValue.node(5).demon.BladeNum,tempValue.node(5).demon.baseFre];
        
    end
end














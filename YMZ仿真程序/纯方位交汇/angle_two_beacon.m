close all;clear all;clc;
%% 参数设置
x1=-500;y1=0;z1=0;
x2=500;y2=0;z2=0;
[x,y]=meshgrid((-1000:10:1000));

diedai=1000;
%% 误差设置
std_p=1.5;
delta_x1=normrnd(0,std_p,1,diedai);delta_y1=normrnd(0,std_p,1,diedai);
delta_x2=normrnd(0,std_p,1,diedai);delta_y2=normrnd(0,std_p,1,diedai);

std_angle=1/180*pi;%单位为pi
delta_angle1=normrnd(0,std_angle,1,diedai);
delta_angle2=normrnd(0,std_angle,1,diedai);

pos_x1=x1+delta_x1;pos_y1=y1+delta_y1;%100次迭代
pos_x2=x2+delta_x2;pos_y2=y2+delta_y2;

%% 纯方位交汇定位
n=size(x);
for p=1:n(1)
    for q=1:n(2)
        for s=1:diedai
            pos_x1=x1+delta_x1(s);pos_y1=y1+delta_y1(s);
            pos_x2=x2+delta_x2(s);pos_y2=y2+delta_y2(s);%GPS测得两个平台位置
            
            L=sqrt((pos_x1-pos_x2)^2+(pos_y1-pos_y2)^2);%基线长度
            
            beta=abs(atan((pos_y2-pos_y1)/(pos_x2-pos_x1)));%基线与正东方向夹角 绝对值 以pi为单位
            
            alpha1=atan2(x(p,q)-pos_x1,y(p,q)-pos_y1)+delta_angle1(s);
            alpha2=atan2(x(p,q)-pos_x2,y(p,q)-pos_y2)+delta_angle2(s);%测得的目标方位角 单位为pi 正北为基准 顺时针为正 逆时针为负
            
            R1=L*abs(cos(alpha2+beta)/sin(alpha2-alpha1));
            R2=L*abs(cos(alpha1+beta)/sin(alpha2-alpha1));%计算得到的平台到目标的距离
            
            xx1=pos_x1+R1*sin(alpha1);
            xx2=pos_x2+R2*sin(alpha2);
            yy1=pos_y1+R1*cos(alpha1);
            yy2=pos_y2+R2*cos(alpha2);
            
            xx=(xx1+xx2)/2;
            yy=(yy1+yy2)/2;
            
            delta_r(s)=sqrt((xx-x(p,q))^2+(yy-y(p,q))^2);
        end
        delta_R(p,q)=mean(delta_r);
    end
end
%% 画图
figure
x_range = -1000:10:1000;
y_range    =  -1000:10:1000;
[X,Y] = meshgrid(x_range,y_range);
h=surf(X,Y,delta_R);hold on;
caxis([min(delta_R(:)) 100]);
view([0,90]);
shading interp
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);
% title('纯方位交汇定位解算误差');

% h=pcolor(x,y,delta_R);hold on;
plot(x1,y1,'r*',x2,y2,'r*');hold on;
% set(h,'edgecolor','none','facecolor','interp');
colorbar;

close all;clear all;clc;
%% 参数设计
line=1000;
x1=line;y1=line;z1=0;              %第i个信标坐标，i=1,2,3,4
x2=line;y2=0;z2=0;             
x3=0;y3=0;z3=0;              
x4=0;y4=line;z4=0; 
z=0;                           %目标深度0

%% 目标轨迹预设
T_source = 0.1; %发声周期
dt = T_source;%观测周期
T_guance = 10;       %观测时长
c = 1500;     %声速
mode = 1;     %模式1：运动  模式2：静止
%% 目标轨迹
X = zeros(4,T_guance/dt);             %目标轨迹
if mode ==1
    v = 0.5;                       %目标角速度
    X0 =[300,30,100,50]';           %目标初始运动状态[x,vx,y,vy]';
    file_name='s_move';
else
    v = 0;                       %目标角速度
    X0 =[500,0,-500,0]';           %目标初始运动状态[x,vx,y,vy]';
    file_name='s_stop';
end

%----------匀速直线运动-------------
F1 = [1,dt,0,0;
    0,1,0,0;
    0,0,1,dt;
    0,0,0,1,];
%----------逆时针转弯运动-------------
F2 = [1,sin(v*dt)/v,0,-(1-cos(v*dt))/v;
    0,cos(v*dt),0,-sin(v*dt);
    0,(1-cos(v*dt))/v,1,sin(v*dt)/v;
    0,sin(v*dt),0,cos(v*dt)];
%----------顺时针转弯运动-------------
F3 = [1,-sin(-v*dt)/v,0,(1-cos(-v*dt))/v;
    0,cos(-v*dt),0,-sin(-v*dt);
    0,-(1-cos(-v*dt))/v,1,-sin(-v*dt)/v;
    0,sin(-v*dt),0,cos(-v*dt);];
%-----------生成真实轨迹----------
k=1;
for t = dt:dt:T_guance
    if t==dt
        X(:,k)=X0;
    elseif t>dt&&t<5
        X(:,k)=F1*X(:,k-1);                          %匀速运动过程
    elseif t>=5&&t<=10
        if mode == 2
            F=F1;
        else
            F=F3;%顺时针转弯运动过程
        end
        X(:,k)=F*X(:,k-1);                          
    elseif t>10&&t<15
        if mode == 2
            F=F1;
        else
            F=F2;
        end
        X(:,k)=F*X(:,k-1);
    elseif t>=15&&t<=26
        X(:,k)=F1*X(:,k-1);                          %匀速运动过程

    
    end
    k=k+1;
end

x=X(1,:);
y=X(3,:);

T=0.1;%观测周期设为0.1s

num=length(X);%目标有400个轨迹点

%% 误差设置
diedai=100;%100次迭代

std_p=1.5;
delta_x1=normrnd(0,std_p,1,diedai);delta_y1=normrnd(0,std_p,1,diedai);
delta_x2=normrnd(0,std_p,1,diedai);delta_y2=normrnd(0,std_p,1,diedai);
delta_x3=normrnd(0,std_p,1,diedai);delta_y3=normrnd(0,std_p,1,diedai);
delta_x4=normrnd(0,std_p,1,diedai);delta_y4=normrnd(0,std_p,1,diedai);

std_angle=1/180*pi;%单位为pi
delta_angle1=normrnd(0,std_angle,1,diedai);
delta_angle2=normrnd(0,std_angle,1,diedai);
delta_angle3=normrnd(0,std_angle,1,diedai);
delta_angle4=normrnd(0,std_angle,1,diedai);
%% 模拟声学测量参数
%浮标GPS上传位置
pos_x1=x1+delta_x1;pos_y1=y1+delta_y1;%100次迭代
pos_x2=x2+delta_x2;pos_y2=y2+delta_y2;
pos_x3=x3+delta_x3;pos_y3=y3+delta_y3;
pos_x4=x4+delta_x4;pos_y4=y4+delta_y4;
for i=1:diedai
    pos1(i,:)=[pos_x1(i) pos_y1(i) z1];%100次迭代×[x y z]坐标
    pos2(i,:)=[pos_x2(i) pos_y2(i) z2];
    pos3(i,:)=[pos_x3(i) pos_y3(i) z3];
    pos4(i,:)=[pos_x4(i) pos_y4(i) z4];
end
%加入误差后的角度
for i=1:num
    alpha1(i,:)=atan2(x(i)-pos_x1,y(i)-pos_y1)+delta_angle1;%150个点×100次迭代
    alpha2(i,:)=atan2(x(i)-pos_x2,y(i)-pos_y2)+delta_angle2;
    alpha3(i,:)=atan2(x(i)-pos_x3,y(i)-pos_y3)+delta_angle3;
    alpha4(i,:)=atan2(x(i)-pos_x4,y(i)-pos_y4)+delta_angle4;
end
% figure
% plot(alpha1(:,1),'r');hold on;
% plot(alpha2(:,1),'b');hold on;
% plot(alpha3(:,1),'g');hold on;
% plot(alpha4(:,1),'m');
% xlim([1:150]);
% legend('基元1','基元2','基元3','基元4');
%原本角度
for i=1:length(alpha1)
    for j=1:diedai
        res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(i,j),alpha2(i,j));
        res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(i,j),alpha1(i,j));
        res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(i,j),alpha3(i,j));
        res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(i,j),alpha4(i,j));
%         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
%         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

        xx(j)=mean(res(:,1));
        yy(j)=mean(res(:,2));
    end
    result_x(i)=mean(xx);
    result_y(i)=mean(yy);
end

% figure
% plot(x,y,'go');hold on;
% axis equal
% axis([0-100 line+100 0-100 line+100]);
% xlabel('x轴/m','FontSize',14);
% ylabel('y轴/m','FontSize',14);hold on;


%计算各浮标测得的角度真实序列
n1=sqrt((pos_x1(1)-x(1))^2+(pos_y1(1)-y(1))^2)/1500/T;
mm=normrnd(0,std_angle,floor(n1),diedai);%0均值，std_angle标准差，floor(n1)×diedai形式的正态分布的随机数矩阵。
alpha1=[mm;alpha1];

n2=sqrt((pos_x2(1)-x(1))^2+(pos_y2(1)-y(1))^2)/1500/T;
mm=normrnd(0,std_angle,floor(n2),diedai);%补前面的0
alpha2=[mm;alpha2];

n3=sqrt((pos_x3(1)-x(1))^2+(pos_y3(1)-y(1))^2)/1500/T;
mm=normrnd(0,std_angle,floor(n3),diedai);%补前面的0
alpha3=[mm;alpha3];

n4=sqrt((pos_x4(1)-x(1))^2+(pos_y4(1)-y(1))^2)/1500/T;
mm=normrnd(0,std_angle,floor(n4),diedai);
alpha4=[mm;alpha4];


%不用时空关联解算的角度
for i=1:length(alpha3)
    for j=1:diedai
        res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(i,j),alpha2(i,j));
        res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(i,j),alpha1(i,j));
        res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(i,j),alpha3(i,j));
        res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(i,j),alpha4(i,j));
%         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
%         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

        xx(j)=mean(res(:,1));
        yy(j)=mean(res(:,2));
        
    end
    no_result_x(i)=mean(xx);
    no_result_y(i)=mean(yy);
end
% for i=1:length(alpha3)
%     if (no_result_y(i)>500 && no_result_y(i)<600)
%        no_result_x(i)=no_result_x(i-1);
%        no_result_y(i)=no_result_y(i-1); 
%     end
% end
% figure
% plot(no_result_x(8:length(no_result_x)),no_result_y(8:length(no_result_x)),'go');

%% 用时空关联
a  = [x1,y1];          %4个基元位置
b  = [x2,y2];
c  = [x3,y3];
d  = [x4, y4];
esv=1500;

for z=9:length(no_result_x)-7
    target_x   = no_result_x(z);
    target_y   = no_result_y(z);
    for nn=1:100
    distance_a = sqrt((target_x-a(1,1))^2+(target_y-a(1,2))^2);
    t_a        = distance_a/esv;
    distance_b = sqrt((target_x-b(1,1))^2+(target_y-b(1,2))^2);
    t_b        = distance_b/esv;
    distance_c = sqrt((target_x-c(1,1))^2+(target_y-c(1,2))^2);
    t_c        = distance_c/esv;
    distance_d = sqrt((target_x-d(1,1))^2+(target_y-d(1,2))^2);
    t_d        = distance_d/esv;
    dis = [distance_a distance_b distance_c distance_d];
    [mindis,minwei]= max(dis);
    if minwei==1
         tao_ab     = abs(t_a-t_b);                %计算两个阵元接收到目标声信号的时延差
         tao_ac     = abs(t_a-t_c);
         tao_ad     = abs(t_a-t_d);
         m_ab=floor(z-tao_ab/0.1);
         m_ac=floor(z-tao_ac/0.1);
         m_ad=floor(z-tao_ad/0.1);
            for j=1:diedai
            res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(z,j),alpha2(m_ab,j));
            res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(m_ad,j),alpha1(z,j));
            res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(m_ab,j),alpha3(m_ac,j));
            res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(m_ac,j),alpha4(m_ad,j));
    %         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
    %         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

            xx(j)=mean(res(:,1));
            yy(j)=mean(res(:,2));
            end
          target_x_change = mean(xx);%res(1);
          target_y_change = mean(yy);%res(2);
    end
    if minwei==2
         tao_ba     = abs(t_b-t_a);                %计算两个阵元接收到目标声信号的时延差
         tao_bc     = abs(t_b-t_c);
         tao_bd     = abs(t_b-t_d);
         m_ba=floor(z-tao_ba/0.1);
         m_bc=floor(z-tao_bc/0.1);
         m_bd=floor(z-tao_bd/0.1);
            for j=1:diedai
            res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(m_ba,j),alpha2(z,j));
            res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(m_bd,j),alpha1(m_ba,j));
            res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(z,j),alpha3(m_bc,j));
            res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(m_bc,j),alpha4(m_bd,j));
    %         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
    %         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

            xx(j)=mean(res(:,1));
            yy(j)=mean(res(:,2));
            end
          target_x_change = mean(xx);%res(1);
          target_y_change = mean(yy);%res(2);
    end
      if minwei==3
         tao_ca     = abs(t_c-t_a);                %计算两个阵元接收到目标声信号的时延差
         tao_cb     = abs(t_c-t_b);
         tao_cd     = abs(t_c-t_d);
         m_ca=floor(z-tao_ca/0.1);
         m_cb=floor(z-tao_cb/0.1);
         m_cd=floor(z-tao_cd/0.1);
            for j=1:diedai
            res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(m_ca,j),alpha2(m_cb,j));
            res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(m_cd,j),alpha1(m_ca,j));
            res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(m_cb,j),alpha3(z,j));
            res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(z,j),alpha4(m_cd,j));
    %         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
    %         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

            xx(j)=mean(res(:,1));
            yy(j)=mean(res(:,2));
            end
          target_x_change = mean(xx);%res(1);
          target_y_change = mean(yy);%res(2);
      end
      if minwei==4
         tao_da     = abs(t_d-t_a);                %计算两个阵元接收到目标声信号的时延差
         tao_db     = abs(t_d-t_b);
         tao_dc     = abs(t_d-t_c);
         m_da=floor(z-tao_da/0.1);
         m_db=floor(z-tao_db/0.1);
         m_dc=floor(z-tao_dc/0.1);
            for j=1:diedai
            res(1,:)=AngleCross(pos1(j,:),pos2(j,:),alpha1(m_da,j),alpha2(m_db,j));
            res(2,:)=AngleCross(pos4(j,:),pos1(j,:),alpha4(z,j),alpha1(m_da,j));
            res(3,:)=AngleCross(pos2(j,:),pos3(j,:),alpha2(m_db,j),alpha3(m_dc,j));
            res(4,:)=AngleCross(pos3(j,:),pos4(j,:),alpha3(m_dc,j),alpha4(z,j));
    %         res(5,:)=AngleCross(pos1(j,:),pos3(j,:),alpha1(i,j),alpha3(i,j));
    %         res(6,:)=AngleCross(pos2(j,:),pos4(j,:),alpha2(i,j),alpha4(i,j));

            xx(j)=mean(res(:,1));
            yy(j)=mean(res(:,2));
            end
          target_x_change = mean(xx);%res(1);
          target_y_change = mean(yy);%res(2);
      end

    if(nn == 100)
        target_pos_x   = target_x_change;
       target_pos_y   = target_y_change;
       break;
    end
    if ((abs(target_x-target_x_change))<1)&&(abs(target_y-target_y_change))<1
       target_pos_x   = target_x_change;
       target_pos_y   = target_y_change;
       break;
    else
       target_x   = target_x_change; 
       target_y   = target_y_change;  
    end
    end
    x_position(z)  = target_pos_x;
    y_position(z)  = target_pos_y;
end
x_position_MA = MovingAverage(x_position(9:end), 11);
y_position_MA = MovingAverage(y_position(9:end), 11);
%% 画图
figure
plot(x(1:87),y(1:87),'go');hold on;
plot(no_result_x(8:94),no_result_y(8:94),'b.');hold on;
plot(x1,y1,'r*');hold on;
plot(x2,y2,'r*');hold on;
plot(x3,y3,'r*');hold on;
plot(x4,y4,'r*');hold on;
axis equal
axis([0-100 line+100 0-100 line+100]);
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);hold on;
legend('目标真实轨迹','时间关联前解算目标轨迹');
figure
plot(x(1:87),y(1:87),'go');hold on;
plot(x_position_MA,y_position_MA,'r.');hold on;
plot(x1,y1,'r*');hold on;
plot(x2,y2,'r*');hold on;
plot(x3,y3,'r*');hold on;
plot(x4,y4,'r*');hold on;
axis equal
axis([0-100 line+100 0-100 line+100]);
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);hold on;
legend('目标真实轨迹','时间关联后解算目标轨迹');


for i=1:length(x_position_MA)
    delta_a(i)=sqrt((x_position_MA(i)-x(i))^2+(y_position_MA(i)-y(i))^2);
    delta_b(i)=sqrt((no_result_x(i)-x(i))^2+(no_result_y(i)-y(i))^2);
end
%%
figure
plot(delta_b,'b.');hold on;
plot(delta_a,'g.');
xlim([9 90])
ylim([0 40])
xlabel('周期号','FontSize',14);
ylabel('定位误差/m','FontSize',14);hold on;
legend('时间关联前定位误差','时间关联后定位误差');
%误差rmse





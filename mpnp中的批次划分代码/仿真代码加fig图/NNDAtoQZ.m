%量测值生成并进行关联滤波
clc; clear; close all; 
%************************************************
%          0.参数设置
%************************************************
I=eye(4); 
T = 1; %发声周期
dt = T;%观测周期
T_guance = 100;       %观测时长
simTime = T_guance/dt ;       
r=0.1; 
R=[r 0;
   0 r];   %量测噪声
H=[1 0 0 0;
   0 0 1 0];   %测量模型
Zk1 = zeros(2,1);
Zk2 = zeros(2,1);
%************************************************
%          1.量测生成
%************************************************
T = 1; %发声周期
dt = T;%观测周期
T_guance = 100;       %观测时长
%1.1 目标1：先直线运动，后顺时针转弯运动
X1 = zeros(4,T_guance/dt);
v = 0.01;                       %目标角速度
X01 =[200,10,200,10]';           %目标初始运动状态[x,vx,y,vy]';
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
        X1(:,k)=X01;    
    elseif t>dt&&t<(T_guance/2)
        X1(:,k)=F1*X1(:,k-1);                          %匀速运动过程
    elseif t>=(T_guance/2)&&t<=T_guance
        X1(:,k)=F2*X1(:,k-1);                          %顺时针转弯运动过程
    end
    k=k+1;
end
%1.2 目标2：匀速直线运动
X2 = zeros(4,T_guance/dt);
X02 =[500,10,0,10]';           %目标初始运动状态[x,vx,y,vy]';
%-----------生成真实轨迹----------
k=1;
for t = dt:dt:T_guance
    if t==dt
        X2(:,k)=X02;    
    elseif t>dt&&t<=T_guance
        X2(:,k)=F1*X2(:,k-1);                          %匀速运动过程
    end
    k=k+1;
end
%1.3 目标1和目标2量测值生成
for i=1:1:simTime 
    Vk=[sqrt(r)*randn;sqrt(r)*randn]; 
    Zk1(:,i)=H*X1(:,i)+Vk;      %生成量测值 
    Zk2(:,i)=H*X2(:,i)+Vk;
end 
figure
plot(0,0,'k*');hold on;
plot(Zk1(1,:),Zk1(2,:),'r*'); %实际测量值 
grid on; hold on
plot(Zk2(1,:),Zk2(2,:),'b*'); %实际测量值
legend('观测点','目标1','目标2');
xlim([-200,1600]);ylim([-200,1600]);
%1.4 加入杂波和漏报
%%%杂波，加入[-20,20]均匀分布的噪声
% q=20;
% Pf = 0.1;       %虚警概率
% n = size(Zk1,2);    %数据个数
% Pf_numb1 = randperm(n,n*Pf);%随机在n个方位中选择置0位置
% Pf_numb2 = randperm(n,n*Pf);%随机在n个方位中选择置0位置
% a1=X1(1,Pf_numb1)-q; a2=X2(1,Pf_numb2)-q;
% b1=X1(1,Pf_numb1)+q; b2=X2(1,Pf_numb2)+q;
% c1=X1(3,Pf_numb1)-q; c2=X2(3,Pf_numb2)-q;
% d1=X1(3,Pf_numb1)+q; d2=X2(3,Pf_numb2)+q; 
% xi1=a1+(b1-a1).*rand(1,n*Pf); 
% yi1=c1+(d1-c1).*rand(1,n*Pf);
% xi2=a2+(b2-a2).*rand(1,n*Pf);
% yi2=c2+(d2-c2).*rand(1,n*Pf);
% pianyi1 = sqrt((xi1-Zk1(1,Pf_numb1)).^2+(yi1-Zk1(2,Pf_numb1)).^2);%目标1杂波偏移量
% pianyi2 = sqrt((xi2-Zk2(1,Pf_numb2)).^2+(yi2-Zk2(2,Pf_numb2)).^2);%目标2杂波偏移量
% Zk1(1,Pf_numb1)=xi1;Zk1(2,Pf_numb1)=yi1;
% Zk2(1,Pf_numb2)=xi2;Zk2(2,Pf_numb2)=yi2;
% 
% %%%漏报，值为NaN
% Pm = 0.05;       %漏报概率
% n = size(Zk1,2);    %数据个数
% PM_numb1 = randperm(n,n*Pm);%随机在n个方位中选择置0位置
% PM_numb2 = randperm(n,n*Pm);%随机在n个方位中选择置0位置
% Zk1(:,PM_numb1)=nan;
% Zk2(:,PM_numb2)=nan;

%************************************************
%          2.量测值关联滤波
%************************************************
Zk(1:2,:) = Zk1;
Zk(3:4,:) = Zk2;
figure
plot(0,0,'k*');hold on;
plot(Zk1(1,:),Zk1(2,:),'r*'); %实际测量值 
grid on; hold on
plot(Zk2(1,:),Zk2(2,:),'b*'); %实际测量值
legend('观测点','目标1','目标2');
xlim([-200,1600]);ylim([-200,1600]);
%轨迹起始设置
for i=50:1:60
    Zk(1,i) = NaN;
    Zk(2,i) = NaN;
end
for i=61:1:70
    Zk(1,i) = Zk(1,i)-300;
    Zk(2,i) = Zk(2,i)+300;
end
for i=10:1:20
    Zk(3,i) = NaN;
    Zk(4,i) = NaN;
end
for i=21:1:40
    Zk(3,i) = Zk(3,i)-300;
    Zk(4,i) = Zk(4,i)+300;
end
figure
plot(0,0,'k*');hold on;
plot(Zk(1,:),Zk(2,:),'r*'); %实际测量值 
plot(Zk(1,61:70),Zk(2,61:70),'m*'); %实际测量值 
grid on; hold on
plot(Zk(3,:),Zk(4,:),'b*'); %实际测量值
plot(Zk(3,21:40),Zk(4,21:40),'g*'); %实际测量值
legend('观测点','目标1','新目标1','目标2','新目标2');
xlim([-200,1600]);ylim([-200,1600]);
    
targetnum = 2;%目标个数
L=8;%轨迹起始判断L个点
Lth=3;%判断是否收敛的阈值
disth=100;%目标相邻两点间距最大值，为了避免第一次量测值有NaN时第二个点关联错误
Z_NNDAforward = zeros(targetnum*2,1);%上一帧的关联结果
Z_kalmanforward = zeros(targetnum*2,1);%上一帧的滤波结果
v_last = zeros(targetnum,2);%上一帧数计算得到的目标速度，2表示vx和vy
bisconvergence = zeros(targetnum,2);%表示前L个点是否收敛，值为1或0或NaN
isconvergence = zeros(1,targetnum);%记录前L个点是否收敛,值为1或0
X_Preforward = zeros(4,targetnum);%上一帧滤波预测结果，用于当前帧NNDA
P_Preforward = cat(3);%上一帧滤波预测结果，用于当前帧NNDA
bisendforward=zeros(targetnum,L);%上一帧轨迹终止标志
endZkforward=zeros(targetnum*2,L);%上一帧轨迹终止对应存储量测值

figure
plot(0,0,'k*');hold on;
for i=1:simTime
    if i==1
        forwardNum=0;
        RealNumforward=0;
        for itar = 1:targetnum
            Z_NNDAforward(:,1) = NaN;
            Z_kalmanforward(:,1) = NaN;
            v_last(itar,1) = NaN;
            v_last(itar,2) = NaN;
            bisconvergence(itar,1) = NaN;
            bisconvergence(itar,2) = NaN;
            X_Preforward(:,itar) = [NaN;NaN;NaN;NaN];
            P_Preforward = cat(3,P_Preforward,NaN);
        end
    end
[Z_NNDA,Z_kalman,RealNum,v_update,X_Pre,P_Pre,bisconvergence,bisend,endZk] ...
    = myNNDAQZ(Zk(:,i),targetnum,L,Lth,disth,T,Z_NNDAforward, Z_kalmanforward, ...
    forwardNum,RealNumforward,v_last,X_Preforward,P_Preforward,bisendforward,endZkforward);
Z_NNDAforward=Z_NNDA;
Z_kalmanforward=Z_kalman;
forwardNum=i;
RealNumforward=RealNum;
v_last=v_update;
X_Preforward=X_Pre;
P_Preforward=P_Pre;
bisendforward=bisend;
endZkforward=endZk;

%绘图
ii = 1:size(Z_NNDA,2);
if size(Z_NNDA,2)<L
    plot(Zk(1,ii),Zk(2,ii),'b.'); %实际测量值 
    grid on; hold on
    plot(Zk(3,ii),Zk(4,ii),'b.'); %实际测量值 
else
    plot(Zk(1,i-L+ii),Zk(2,i-L+ii),'b.'); %实际测量值 
    grid on; hold on
    plot(Zk(3,i-L+ii),Zk(4,i-L+ii),'b.'); %实际测量值 
end
    plot(Z_NNDA(1,ii),Z_NNDA(2,ii),'go'); %目标1关联的测量值
    plot(Z_kalman(1,ii),Z_kalman(2,ii),'g-','LineWidth',2); %目标1滤波值
    plot(Z_NNDA(3,ii),Z_NNDA(4,ii),'ro'); %目标2关联上测量值
    plot(Z_kalman(3,ii),Z_kalman(4,ii),'r-','LineWidth',2); %目标2滤波值
end
legend('观测点','量测值','量测值','目标1关联量测','目标1滤波值','目标2关联量测','目标2滤波值');
% title('目标运动轨迹'); 
xlabel('x/m'); ylabel('y/m'); 
xlim([-200,1600]);ylim([-200,1600]);
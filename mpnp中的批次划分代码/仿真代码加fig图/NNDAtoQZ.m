%����ֵ���ɲ����й����˲�
clc; clear; close all; 
%************************************************
%          0.��������
%************************************************
I=eye(4); 
T = 1; %��������
dt = T;%�۲�����
T_guance = 100;       %�۲�ʱ��
simTime = T_guance/dt ;       
r=0.1; 
R=[r 0;
   0 r];   %��������
H=[1 0 0 0;
   0 0 1 0];   %����ģ��
Zk1 = zeros(2,1);
Zk2 = zeros(2,1);
%************************************************
%          1.��������
%************************************************
T = 1; %��������
dt = T;%�۲�����
T_guance = 100;       %�۲�ʱ��
%1.1 Ŀ��1����ֱ���˶�����˳ʱ��ת���˶�
X1 = zeros(4,T_guance/dt);
v = 0.01;                       %Ŀ����ٶ�
X01 =[200,10,200,10]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
%----------����ֱ���˶�-------------
F1 = [1,dt,0,0;
    0,1,0,0;
    0,0,1,dt;
    0,0,0,1,];
%----------��ʱ��ת���˶�-------------
F2 = [1,sin(v*dt)/v,0,-(1-cos(v*dt))/v;
    0,cos(v*dt),0,-sin(v*dt);
    0,(1-cos(v*dt))/v,1,sin(v*dt)/v;
    0,sin(v*dt),0,cos(v*dt)];
%----------˳ʱ��ת���˶�-------------
F3 = [1,-sin(-v*dt)/v,0,(1-cos(-v*dt))/v;
    0,cos(-v*dt),0,-sin(-v*dt);
    0,-(1-cos(-v*dt))/v,1,-sin(-v*dt)/v;
    0,sin(-v*dt),0,cos(-v*dt);];
%-----------������ʵ�켣----------
k=1;
for t = dt:dt:T_guance
    if t==dt
        X1(:,k)=X01;    
    elseif t>dt&&t<(T_guance/2)
        X1(:,k)=F1*X1(:,k-1);                          %�����˶�����
    elseif t>=(T_guance/2)&&t<=T_guance
        X1(:,k)=F2*X1(:,k-1);                          %˳ʱ��ת���˶�����
    end
    k=k+1;
end
%1.2 Ŀ��2������ֱ���˶�
X2 = zeros(4,T_guance/dt);
X02 =[500,10,0,10]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
%-----------������ʵ�켣----------
k=1;
for t = dt:dt:T_guance
    if t==dt
        X2(:,k)=X02;    
    elseif t>dt&&t<=T_guance
        X2(:,k)=F1*X2(:,k-1);                          %�����˶�����
    end
    k=k+1;
end
%1.3 Ŀ��1��Ŀ��2����ֵ����
for i=1:1:simTime 
    Vk=[sqrt(r)*randn;sqrt(r)*randn]; 
    Zk1(:,i)=H*X1(:,i)+Vk;      %��������ֵ 
    Zk2(:,i)=H*X2(:,i)+Vk;
end 
figure
plot(0,0,'k*');hold on;
plot(Zk1(1,:),Zk1(2,:),'r*'); %ʵ�ʲ���ֵ 
grid on; hold on
plot(Zk2(1,:),Zk2(2,:),'b*'); %ʵ�ʲ���ֵ
legend('�۲��','Ŀ��1','Ŀ��2');
xlim([-200,1600]);ylim([-200,1600]);
%1.4 �����Ӳ���©��
%%%�Ӳ�������[-20,20]���ȷֲ�������
% q=20;
% Pf = 0.1;       %�龯����
% n = size(Zk1,2);    %���ݸ���
% Pf_numb1 = randperm(n,n*Pf);%�����n����λ��ѡ����0λ��
% Pf_numb2 = randperm(n,n*Pf);%�����n����λ��ѡ����0λ��
% a1=X1(1,Pf_numb1)-q; a2=X2(1,Pf_numb2)-q;
% b1=X1(1,Pf_numb1)+q; b2=X2(1,Pf_numb2)+q;
% c1=X1(3,Pf_numb1)-q; c2=X2(3,Pf_numb2)-q;
% d1=X1(3,Pf_numb1)+q; d2=X2(3,Pf_numb2)+q; 
% xi1=a1+(b1-a1).*rand(1,n*Pf); 
% yi1=c1+(d1-c1).*rand(1,n*Pf);
% xi2=a2+(b2-a2).*rand(1,n*Pf);
% yi2=c2+(d2-c2).*rand(1,n*Pf);
% pianyi1 = sqrt((xi1-Zk1(1,Pf_numb1)).^2+(yi1-Zk1(2,Pf_numb1)).^2);%Ŀ��1�Ӳ�ƫ����
% pianyi2 = sqrt((xi2-Zk2(1,Pf_numb2)).^2+(yi2-Zk2(2,Pf_numb2)).^2);%Ŀ��2�Ӳ�ƫ����
% Zk1(1,Pf_numb1)=xi1;Zk1(2,Pf_numb1)=yi1;
% Zk2(1,Pf_numb2)=xi2;Zk2(2,Pf_numb2)=yi2;
% 
% %%%©����ֵΪNaN
% Pm = 0.05;       %©������
% n = size(Zk1,2);    %���ݸ���
% PM_numb1 = randperm(n,n*Pm);%�����n����λ��ѡ����0λ��
% PM_numb2 = randperm(n,n*Pm);%�����n����λ��ѡ����0λ��
% Zk1(:,PM_numb1)=nan;
% Zk2(:,PM_numb2)=nan;

%************************************************
%          2.����ֵ�����˲�
%************************************************
Zk(1:2,:) = Zk1;
Zk(3:4,:) = Zk2;
figure
plot(0,0,'k*');hold on;
plot(Zk1(1,:),Zk1(2,:),'r*'); %ʵ�ʲ���ֵ 
grid on; hold on
plot(Zk2(1,:),Zk2(2,:),'b*'); %ʵ�ʲ���ֵ
legend('�۲��','Ŀ��1','Ŀ��2');
xlim([-200,1600]);ylim([-200,1600]);
%�켣��ʼ����
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
plot(Zk(1,:),Zk(2,:),'r*'); %ʵ�ʲ���ֵ 
plot(Zk(1,61:70),Zk(2,61:70),'m*'); %ʵ�ʲ���ֵ 
grid on; hold on
plot(Zk(3,:),Zk(4,:),'b*'); %ʵ�ʲ���ֵ
plot(Zk(3,21:40),Zk(4,21:40),'g*'); %ʵ�ʲ���ֵ
legend('�۲��','Ŀ��1','��Ŀ��1','Ŀ��2','��Ŀ��2');
xlim([-200,1600]);ylim([-200,1600]);
    
targetnum = 2;%Ŀ�����
L=8;%�켣��ʼ�ж�L����
Lth=3;%�ж��Ƿ���������ֵ
disth=100;%Ŀ���������������ֵ��Ϊ�˱����һ������ֵ��NaNʱ�ڶ������������
Z_NNDAforward = zeros(targetnum*2,1);%��һ֡�Ĺ������
Z_kalmanforward = zeros(targetnum*2,1);%��һ֡���˲����
v_last = zeros(targetnum,2);%��һ֡������õ���Ŀ���ٶȣ�2��ʾvx��vy
bisconvergence = zeros(targetnum,2);%��ʾǰL�����Ƿ�������ֵΪ1��0��NaN
isconvergence = zeros(1,targetnum);%��¼ǰL�����Ƿ�����,ֵΪ1��0
X_Preforward = zeros(4,targetnum);%��һ֡�˲�Ԥ���������ڵ�ǰ֡NNDA
P_Preforward = cat(3);%��һ֡�˲�Ԥ���������ڵ�ǰ֡NNDA
bisendforward=zeros(targetnum,L);%��һ֡�켣��ֹ��־
endZkforward=zeros(targetnum*2,L);%��һ֡�켣��ֹ��Ӧ�洢����ֵ

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

%��ͼ
ii = 1:size(Z_NNDA,2);
if size(Z_NNDA,2)<L
    plot(Zk(1,ii),Zk(2,ii),'b.'); %ʵ�ʲ���ֵ 
    grid on; hold on
    plot(Zk(3,ii),Zk(4,ii),'b.'); %ʵ�ʲ���ֵ 
else
    plot(Zk(1,i-L+ii),Zk(2,i-L+ii),'b.'); %ʵ�ʲ���ֵ 
    grid on; hold on
    plot(Zk(3,i-L+ii),Zk(4,i-L+ii),'b.'); %ʵ�ʲ���ֵ 
end
    plot(Z_NNDA(1,ii),Z_NNDA(2,ii),'go'); %Ŀ��1�����Ĳ���ֵ
    plot(Z_kalman(1,ii),Z_kalman(2,ii),'g-','LineWidth',2); %Ŀ��1�˲�ֵ
    plot(Z_NNDA(3,ii),Z_NNDA(4,ii),'ro'); %Ŀ��2�����ϲ���ֵ
    plot(Z_kalman(3,ii),Z_kalman(4,ii),'r-','LineWidth',2); %Ŀ��2�˲�ֵ
end
legend('�۲��','����ֵ','����ֵ','Ŀ��1��������','Ŀ��1�˲�ֵ','Ŀ��2��������','Ŀ��2�˲�ֵ');
% title('Ŀ���˶��켣'); 
xlabel('x/m'); ylabel('y/m'); 
xlim([-200,1600]);ylim([-200,1600]);
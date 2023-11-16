% NNDA-FA 何友《雷达数据处理及应用》P116
% 二维空间匀速直线运动，状态向量为X=[x,vx,y,vy] x1=x0+vxT y1=y0+vyT
% 仿真： 1、改变虚假量测数量nc：公式求取、手动设置 2、改变量测噪声R=[r 0; 0 r]，即r 3、改变虚假量测位置q，偏离真实位置的程度
% 问题： 每次产生杂波时，Sk如何确定？
 
function nnda_test()
clc; 
clear; 
close all; 
%************************************************
%          参数设置
%************************************************
I=eye(4); 
T = 1;                            %采样间隔 
simTime = 100 ;             %仿真步数 
A=[1 T 0 0;
    0 1 0 0;
    0 0 1 T;
    0 0 0 1];   %实际模型：CV 
A_Model=[1 T 0 0;
    0 1 0 0;
    0 0 1 T;
    0 0 0 1];     %建立模型 
H=[1 0 0 0;
   0 0 1 0];   %测量模型 
Q=1;              %实际过程噪声 
Q_Model=1;   %建立模型的过程噪声 
G = [T^2/2 0;
    T 0;
    0 T^2/2;
    0 T];   %噪声加权矩阵 
r=20; 
R=[r 0;
   0 r];   %量测噪声 
X0=[200;0;10000;-15];  %初始状态 
X(:,1)=X0; 
Vk=[sqrt(r)*randn;sqrt(r)*randn]; 
Zk(:,1)=H*X(:,1)+Vk; 
gama=16;            %杂波设置参数 
lamda=0.0004;     %单位面积的虚假量测数 
%************************************************
%          量测生成
%************************************************
for i=2:1:simTime 
    X(:,i)=A*X(:,i-1);          % 真实状态 
    Vk=[sqrt(r)*randn;sqrt(r)*randn]; 
    Zk(:,i)=H*X(:,i)+Vk;      %生成量测值 
end 
%************************************************
%          NNSF初始化 %************************************************
Xk_NNSF=[210;0;10100;-16];  %初始状态、与实际值略有差别 
R11=r; R22=r; R12=0; R21=0; 
Pkk_NNSF=[R11 R11/T R12 R12/T; 
    R11/T 2*R11/T^2 R12/T 2*R12/T^2; 
    R21 R21/T R22 R22/T; 
    R21/T 2*R21/T^2 R22/T 2*R22/T^2];   %初始协方差 
Xkk =Xk_NNSF ; % X0 
Pkk = Pkk_NNSF ; 
X_Pre = Xk_NNSF ; 
P_Pre = Pkk_NNSF ; 
P=R; 
for i=1:1:simTime 
    %************************************************
    %          产生杂波
    %************************************************
    Sk=H*P_Pre*H'+P;              %新息协方差 
    Av=pi*gama*sqrt(det(Sk));   % 量测确认区域面积： **根号下SK 
    nc=floor(10*Av*lamda+1);    %设置杂波数量：10Av*+1 取整   nc = 20 
    % disp(nc);       %显示杂波数目
    %虚假量测
    q=sqrt(10*Av)/2;  %中间变量 
    q=q/10;  %人为减小，虚假量测不能分布太广，否则跟不上目标 
    a=X(1,i)-q; 
    b=X(1,i)+q; 
    c=X(3,i)-q; 
    d=X(3,i)+q; 
    xi=a+(b-a)*rand(1,nc); 
    yi=c+(d-c)*rand(1,nc); 
    clear Z_Matrix;     %从内存中清除 
    clear PZ_Matrix;   %从内存中清除 
    for j=1:nc 
        Z_Matrix(:,j) = [xi(j);yi(j)];  %杂波量测：Z_Matrix数组的前nc列 
    end 
    Z_Matrix(:,nc+1)=Zk(:,i);  %真实量测：Z_Matrix数组的第nc+1列 
     
    PZ_Matrix = cat(3);  %定义变量而已 
    for j=1:1:nc 
        PZ_Matrix = cat(3,PZ_Matrix,[q,0; 0,q]);    %PZ_Matrix维数：2*2*nc； 
        %  杂波量测对应的方差 PZ_Matrix(:,:,j)=[q,0; 0,q]  (j=1:nc)
    end 
    PZ_Matrix = cat(3,PZ_Matrix,R);  %则很难使量测对应的方差：PZ_Matrix(:,:,nc+1)=R 
    %************************************************
    %          NNDA关联
    %************************************************
    Z_Predict = H*X_Pre;               %量测预测 
    PZ_Predict = H*P_Pre*H'+R ;   %新息协方差 
    [Z,P] = myNNDA(Z_Matrix, PZ_Matrix, Z_Predict, PZ_Predict);   % NNDA，返回关联量测和对应方差     
    Z_NNDA(:,i) = Z;  %关联的量测存储 
    %************************************************
    %          卡尔曼滤波
    %************************************************
    [Xk,Pk,Kk]=Kalman(Xkk,Pkk,Z,A_Model,G,Q_Model,H,P); 
    Xkk=Xk; 
    Pkk=Pk; 
    % 预测
    X_Pre=A_Model*Xkk; 
    P_Pre=A_Model*Pkk*A_Model'+G*Q_Model*G'; 
    %取出各个状态值
    Ex_NNSF(i)=Xkk(1);     %x 
    Evx_NNSF(i)=Xkk(2);    %vx 
    Ey_NNSF(i)=Xkk(3);    %y 
    Evy_NNSF(i)=Xkk(4);    %vx 
    error1_NNSF(i)=Ex_NNSF(i)-X(1,i);%Pkk(1,1);    %error-x 
    error2_NNSF(i)=Ey_NNSF(i)-X(3,i);    %error-vx 
    error3_NNSF(i)=Evx_NNSF(i)-X(2,i);    %error-y 
    error4_NNSF(i)=Evy_NNSF(i)-X(4,i);    %error-vx 
end 
%************************************************
%          绘图
%************************************************
i=1:simTime; 
%轨迹
figure 
plot(X(1,i),X(3,i),'-','LineWidth',2); %真实值 
grid on; hold on 
plot(Ex_NNSF(1,i),Ey_NNSF(1,i),'r-','LineWidth',2); %滤波值 
plot(Zk(1,i),Zk(2,i),'*'); %实际测量值 
plot(Z_NNDA(1,i),Z_NNDA(2,i),'o'); %关联上的测量值 
legend('真实值','滤波值','实际量测','关联量测'); 
title('目标运动轨迹'); xlabel('x/m'); ylabel('y/m'); 
text(X(1,1)+1,X(3,1)+5,'t=1'); 
 
%位置误差
figure 
subplot(211) 
plot(abs(error1_NNSF(i)),'LineWidth',2); grid on 
title('位置误差'); xlabel('t/s'); ylabel('error-x/m'); 
subplot(212) 
plot(abs(error3_NNSF(i)),'LineWidth',2); grid on 
xlabel('t/s'); ylabel('error-y/m'); 
 
%速度误差
figure 
subplot(211) 
plot(abs(error2_NNSF(i)),'LineWidth',2); grid on 
title('速度误差'); xlabel('t/s'); ylabel('error-vx/m/s'); subplot(212) 
plot(abs(error4_NNSF(i)),'LineWidth',2); grid on 
xlabel('t/s'); ylabel('error-vy/m/s'); 
end
 
function [Z,P] = myNNDA(Z_Matrix, PZ_Matrix, Z_Predict, PZ_Predict) 
% 最邻近数据关联函数 输入： Z_Matrix：波门内的有效量测值（包括杂波和真实量测） PZ_Matrix：有效量测值的误差方差阵
% Z_Predict：量测预测值 PZ_Predict：量测预测值的误差方差阵 输出： Z：按照统计距离最近原则关联上的量测值
% P：关联上的量测值对应的协方差
nm=size(Z_Matrix); 
n=nm(2);    % 波门内有效量测的数量,即列数 
for i=1:1:n 
    e(:,i)=Z_Matrix(:,i)-Z_Predict;  %每个量测与预测值的距离 
    S(:,:,i)=PZ_Predict+PZ_Matrix(:,:,i);  %对应协方差（X、R、Q互不相关条件下） 
    D(:,i)=e(:,i)'*inv(S(:,:,i))*e(:,i); %统计距离 
end 
Z=Z_Matrix(:,1) ; 
P=PZ_Matrix(:,:,1); 
d=D(:,1); 
index=1; 
for i=2:1:n 
    if D(:,i)<d 
        d=D(:,i); 
        Z=Z_Matrix(:,i) ; 
        P=PZ_Matrix(:,:,i); 
        index=i; 
    end 
end 
end

function [X,P,K]=Kalman(X_Forward,P_Forward,Z,A,G,Q,H,R) 
%卡尔曼滤波 author:LW date:2011.06.08 参数说明
%       Z--观测数据矢量
 
%       A--系统模型状态矩阵 G--系统模型噪声系数矩阵 Q--系统模型噪声方差 H--量测系数矩阵 R--量测模型噪声协方差
%       X_Forward--前次估计状态矢量 %       P_Forward--前次估计状态协方差矩阵
 
%       X--输出估计状态矢量 P--输出估计状态协方差矩阵
 
% 预测
X_Pre=A*X_Forward; 
P_Pre=A*P_Forward*A'+G*Q*G'; 
 
% 增益矩阵
K=P_Pre*H'*inv(H*P_Pre*H'+R)'; 
 
% Pzz = H*P_Forward*H'+ R;                   %S(k+1/k+1) 新息协方差 Pxz =
% P_Forward*H' ;                       %状态与量测之间的协方差 K =
% P_Forward*H'*(inv(Pzz));               %K(k+1) 增益
 
% 修正滤波值和误差协方差阵
X=A*X_Forward+K*(Z-H*(A*X_Forward)); 
 
M=K*H; 
n=size(M); 
I=eye(n); 
P=(I-K*H)*P_Pre*(I-K*H)'+ K*R*K';   
end
  
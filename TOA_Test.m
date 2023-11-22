%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA_Test.m
% 版本: v1.0
% 作者: 网络代码
% 联系方式: https://blog.csdn.net/Maxwellkuai/article/details/106378087
% 日期: 2023-11-22
% 描述: 实现TOA算法
% 输入:  
% 输出:  
%**************************************************************************




clear  
close all  
clc 


delta_t = 2; % 目标的本地时钟和基站系统的钟差  
  
c=340;    %信号的飞行速度，声音信号  
  
BS1 = [-7000, 2000]; % 基站1的位置  
BS2 = [6800, 5400];  % 基站2的位置  
BS3 = [3800, -5600]; % 基站3的位置  
BS4 = [-3200, -3900];% 基站4的位置  
  
targ_p =  [-2000, 1000];%目标的实际位置  
  
pack_s = 0;% 目标发出包裹时的目标本地时间  
  
t1 = sqrt((BS1-targ_p)*(BS1-targ_p)')/c; % 信号到达基站1的信号飞行时间  
t2 = sqrt((BS2-targ_p)*(BS2-targ_p)')/c; % 信号到达基站2的信号飞行时间  
t3 = sqrt((BS3-targ_p)*(BS3-targ_p)')/c; % 信号到达基站3的信号飞行时间  
t4 = sqrt((BS4-targ_p)*(BS4-targ_p)')/c; % 信号到达基站4的信号飞行时间  
  
t_flight = [t1 t2 t3 t4];  
  
pack_r_truth = pack_s + delta_t + t_flight; %包裹到达各个基站时的基站本地实际的时间  
  
pack_r_measure = pack_r_truth + randn(1,4)*0.1; %包裹到达各个基站时的基站本地记录的时间，含有偏差  
  
%% 设置算法矩阵参数 用的是老师方法2  
  
%{  
推导步骤如下  
(x-x1)^2+(y-y1)^2=(c*t1)^2   
(x-x2)^2+(y-y2)^2=(c*t2)^2   
(x-x3)^2+(y-y3)^2=(c*t3)^2   
(x-x4)^2+(y-y4)^2=(c*t4)^2   
  
展开为  
x^2+y^2-2x*x1-2y*y1+〖x1〗^2+〖y1〗^2=(c*t1)^2 ①  
x^2+y^2-2x*x2-2y*y2+〖x2〗^2+〖y2〗^2=(c*t2)^2 ②  
x^2+y^2-2x*x3-2y*y3+〖x3〗^2+〖y3〗^2=(c*t3)^2 ③  
  
②减①，③减①  
2(x1-x2)x+2(y1-y2)y=(c*t2)^2-(c*t1)^2-(〖x2〗^2-〖x1〗^2+〖y2〗^2-〖y1〗^2)   
2(x1-x3)x+2(y1-y3)y=(c*t3)^2-(c*t1)^2-(〖x3〗^2-〖x1〗^2+〖y3〗^2-〖y1〗^2)  
  
  
X=(A^T A)^(-1) A^T B  
  
%}  
  
A(1,:)=2*[BS1(1)-BS2(1) BS1(2)-BS2(2) pack_r_measure(2)-pack_r_measure(1)] ;    %方程相减之后，包裹位置系数A设置  
A(2,:)=2*[BS1(1)-BS3(1) BS1(2)-BS3(2) pack_r_measure(3)-pack_r_measure(1)] ;  
A(3,:)=2*[BS1(1)-BS4(1) BS1(2)-BS4(2) pack_r_measure(4)-pack_r_measure(1)] ;  
  
B(1)=(pack_r_measure(2)^2-pack_r_measure(1)^2)*340^2-BS2(1)^2-BS2(2)^2+BS1(1)^2+BS1(2)^2;   %方程相减之后，常数矩阵B  
B(2)=(pack_r_measure(3)^2-pack_r_measure(1)^2)*340^2-BS3(1)^2-BS3(2)^2+BS1(1)^2+BS1(2)^2;  
B(3)=(pack_r_measure(4)^2-pack_r_measure(1)^2)*340^2-BS4(1)^2-BS4(2)^2+BS1(1)^2+BS1(2)^2;  
  
Res=(A'*A)^(-1)*A'*B';      %利用矩阵乘法把系数矩阵转到方程右边求解包裹位置  
%也可以简便写为 Res=A^(-1)*B  
  
x=Res(1);  
y=Res(2);  
del=Res(3)/340/340;  
  
targ_p_estimated_x = x  
targ_p_estimated_y = y  
delta_t_estiamted = del  
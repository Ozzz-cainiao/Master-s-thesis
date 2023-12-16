%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TOA\TOACalc.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-15
% 描述: 依照杨卓和https://blog.csdn.net/Maxwellkuai/article/details/106378087
%       实现的TOA算法
% 输入:  
% 输出:  
%**************************************************************************

% 假设目标在水平面上运动 深度为0 只用3个平台即可解算出唯一的目标位置
clc 
clear  
close all  

c=1500;    %信号的飞行速度，声音信号  

% 观测平台位置
BS_matrix = [-7000, 2000, 0;
             6800, 5400, 0;
             3800, -5600, 0;
             -3200, -3900, 0]; 
  
targ_p =  [-2000, 1000, 0];%目标的实际位置  
zs = 0;
len = length(BS_matrix);
t_flight = zeros(1, len);
r = zeros(1, len); % x^2 + y^2 + z^2
for i = 1 : len
    t_flight(i) = sqrt(sum((BS_matrix(i,:) - targ_p).^2)) / c;  % 信号到达基站i的信号飞行时间
    r(i) = BS_matrix(i,:) * BS_matrix(i,:)';
end
  
pack_r_truth = t_flight; %包裹到达各个基站时的基站本地实际的时间  
  
pack_r_measure = pack_r_truth + randn(1,4)*0.1; %包裹到达各个基站时的基站本地记录的时间，含有偏差  
  
%% 设置算法矩阵参数 用的是老师方法2  
  
%{  
推导步骤如下  
(x-x1)^2+(y-y1)^2=(c*t1)^2   
(x-x2)^2+(y-y2)^2=(c*t2)^2   
(x-x3)^2+(y-y3)^2=(c*t3)^2   

  
展开为  
x^2+y^2-2x*x1-2y*y1+〖x1〗^2+〖y1〗^2=(c*t1)^2 ①  
x^2+y^2-2x*x2-2y*y2+〖x2〗^2+〖y2〗^2=(c*t2)^2 ②  
x^2+y^2-2x*x3-2y*y3+〖x3〗^2+〖y3〗^2=(c*t3)^2 ③  
  
②减①，③减①  
2(x1-x2)x+2(y1-y2)y=(c*t2)^2-(c*t1)^2-(〖x2〗^2-〖x1〗^2+〖y2〗^2-〖y1〗^2)   
2(x1-x3)x+2(y1-y3)y=(c*t3)^2-(c*t1)^2-(〖x3〗^2-〖x1〗^2+〖y3〗^2-〖y1〗^2)  
  
  
X=(A^T A)^(-1) A^T B  
  
%}  
%% 设置算法矩阵参数 用的是老师方法2  
  
%{  
推导步骤如下  
(x-x1)^2+(y-y1)^2=(c*t1)^2   
(x-x2)^2+(y-y2)^2=(c*t2)^2   
(x-x3)^2+(y-y3)^2=(c*t3)^2   

  
展开为  
x^2+y^2-2x*x1-2y*y1+〖x1〗^2+〖y1〗^2=(c*t1)^2 ①  
x^2+y^2-2x*x2-2y*y2+〖x2〗^2+〖y2〗^2=(c*t2)^2 ②  
x^2+y^2-2x*x3-2y*y3+〖x3〗^2+〖y3〗^2=(c*t3)^2 ③  
  
①-②，①-③  
2(x1-x2)x+2(y1-y2)y=(c*t1)^2-(c*t2)^2+(〖x2〗^2-〖x1〗^2+〖y2〗^2-〖y1〗^2)   
2(x1-x3)x+2(y1-y3)y=(c*t1)^2-(c*t3)^2+(〖x3〗^2-〖x1〗^2+〖y3〗^2-〖y1〗^2)  
  
C = 2[x1 - x2, y1 - y2;
      x1 - x3, y1 - y3] 
D = []
X = [x;y]
X=(A^T A)^(-1) A^T B  
  
%} 
% 采用费线性解算方法，先利用前2个方程求出一组双解，再利用另一个方程进行判解
% 先利用平台1、2作差
% 从所有平台中选2个
cha = BS_matrix(2, 2) - BS_matrix(1, 2);
if cha == 0
    xs = (r(2)^2 - r(1)^2 + c^2 * (t(1)^2 - t(2)^2)) / (2*(x(2) - x(1)));
else
    A = (x(2) - x(1)) / (y(2) - y(1));
    B = (-2 * (z(2) - z(1)) * zs + r(2)^2 - r(1)^2 + d(1)^2 - d(2)^2) / (2 *(y(2) - y(1)));
    C = 1 + A * A';  % 存疑
    D = -2 * (x(1) + A * y(1) - A * B);
    E = d(1)^2 - r(1)^2 + B * B' - 2 * B * y(1) - 2 * z(1) * zs + zs^2;
    xs1 = (-D + sqrt(D * D' - 4 * C * E)) / (2 * C);
    x12 = (-D - sqrt(D * D' - 4 * C * E)) / (2 * C);
    % 将xs带入方程组的前2式，求出来双解
    xs
end









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

%% 计算两点之间的距离
function [distance] = osjl(object, source)
% 输入均为列向量
distance = sqrt(sum((object - source).^2));
end
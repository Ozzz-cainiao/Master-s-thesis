close all;clear all;clc;

%% 误差设置
std_t=0.001;
std_p=1.5;
std_c=1;

length=10000;
delta_t1=normrnd(0,std_t,1,length);  
delta_t2=normrnd(0,std_t,1,length);
delta_t3=normrnd(0,std_t,1,length);
delta_t4=normrnd(0,std_t,1,length);%随机生成一个均值为0，标准差为2 的1*1000随机数矩阵

delta_xi1=normrnd(0,std_p,1,length);delta_yi1=normrnd(0,std_p,1,length);delta_zi1=normrnd(0,std_p,1,length);    
delta_xj1=normrnd(0,std_p,1,length);delta_yj1=normrnd(0,std_p,1,length);delta_zj1=normrnd(0,std_p,1,length);  
delta_xi2=normrnd(0,std_p,1,length);delta_yi2=normrnd(0,std_p,1,length);delta_zi2=normrnd(0,std_p,1,length);    
delta_xj2=normrnd(0,std_p,1,length);delta_yj2=normrnd(0,std_p,1,length);delta_zj2=normrnd(0,std_p,1,length);  
delta_xi3=normrnd(0,std_p,1,length);delta_yi3=normrnd(0,std_p,1,length);delta_zi3=normrnd(0,std_p,1,length);    
delta_xj3=normrnd(0,std_p,1,length);delta_yj3=normrnd(0,std_p,1,length);delta_zj3=normrnd(0,std_p,1,length);  
delta_xi4=normrnd(0,std_p,1,length);delta_yi4=normrnd(0,std_p,1,length);delta_zi4=normrnd(0,std_p,1,length);    
delta_xj4=normrnd(0,std_p,1,length);delta_yj4=normrnd(0,std_p,1,length);delta_zj4=normrnd(0,std_p,1,length);  
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c1=normrnd(0,std_c,1,length);
delta_c2=normrnd(0,std_c,1,length);
delta_c3=normrnd(0,std_c,1,length);
delta_c4=normrnd(0,std_c,1,length);
%% 参数设置
% x1_far=500;y1_far=500;z1_near=2;
% % x1_far=500;y1_far=500;z1_far=80;
% % x1_far=500-20;y1_far=500-20;z1_far=80;
% x1_near=500-20;y1_near=500-20;z1_far=80;
% x2_far=500;y2_far=-500;z2_near=2;
% % x2_far=500;y2_far=500;z2_far=80;
% % x2_far=500-20;y2_far=-500+20;z2_far=80;
% x2_near=500-20;y2_near=-500+20;z2_far=80;
% x3_far=-500;y3_far=-500;z3_near=2;
% % x3_far=500;y3_far=500;z3_far=80;
% % x3_far=-500+20;y3_far=-500+20;z3_far=80;
% x3_near=-500+20;y3_near=-500+20;z3_far=80;
% x4_far=-500;y4_far=500;z4_near=2;
% % x4_far=500;y4_far=500;z4_far=80;
% % x4_far=-500+20;y4_far=500-20;z4_far=80;
% x4_near=-500+20;y4_near=500-20;z4_far=80;
%%%%%%%%0519改偏移量
line_near=500;
delta=-20; 
%浮标GPS上传位置
x1_near=line_near;y1_near=line_near;z1_near=2;
x2_near=line_near;y2_near=-480;z2_near=2;
x3_near=-480;y3_near=-480;z3_near=2;
x4_near=-480;y4_near=line_near;z4_near=2;

x1=x1_near+delta;y1=y1_near+delta;z1=0;              %第i个信标坐标，i=1,2,3,4
x2=x2_near+delta;y2=y2_near+delta;z2=0;             
x3=x3_near+delta;y3=y3_near+delta;z3=0;              
x4=x4_near+delta;y4=y4_near+delta;z4=0;
x1_far=x1;y1_far=y1;z1_far=80;
x2_far=x2;y2_far=y2;z2_far=80;
x3_far=x3;y3_far=y3;z3_far=80;
x4_far=x4;y4_far=y4;z4_far=80;

% x1_near=500;y1_near=500;z1_near=2;
% x1_far=500;y1_far=500;z1_far=80;
% x2_near=500;y2_near=-500;z2_near=2;
% x2_far=500;y2_far=-500;z2_far=80;
% x3_near=-500;y3_near=-500;z3_near=2;
% x3_far=-500;y3_far=-500;z3_far=80;
% x4_near=-500;y4_near=500;z4_near=2;
% x4_far=-500;y4_far=500;z4_far=80;
[x,y]=meshgrid((-500:10:500)); %目标移动范围
z=300;
esv=1500;
%% 垂直双阵元测深
n=size(x);
for p=1:n(1)
    for q=1:n(2)
        k=1;
        for s=1:length
            dj1=sqrt((x1_far-x(p,q))^2+(y1_far-y(p,q))^2+(z1_far-z)^2);
            di1=sqrt((x1_near-x(p,q))^2+(y1_near-y(p,q))^2+(z1_near-z)^2);
            timedelay1=(di1/esv)-(dj1/esv);

            dj2=sqrt((x2_far-x(p,q))^2+(y2_far-y(p,q))^2+(z2_far-z)^2);
            di2=sqrt((x2_near-x(p,q))^2+(y2_near-y(p,q))^2+(z2_near-z)^2);
            timedelay2=(di2/esv)-(dj2/esv);
            
            dj3=sqrt((x3_far-x(p,q))^2+(y3_far-y(p,q))^2+(z3_far-z)^2);
            di3=sqrt((x3_near-x(p,q))^2+(y3_near-y(p,q))^2+(z3_near-z)^2);
            timedelay3=(di3/esv)-(dj3/esv);
            
            dj4=sqrt((x4_far-x(p,q))^2+(y4_far-y(p,q))^2+(z4_far-z)^2);
            di4=sqrt((x4_near-x(p,q))^2+(y4_near-y(p,q))^2+(z4_near-z)^2);
            timedelay4=(di4/esv)-(dj4/esv);
            
            c1=esv+delta_c1(s);
            c2=esv+delta_c2(s);
            c3=esv+delta_c3(s);
            c4=esv+delta_c4(s);
            
            tao1=timedelay1+delta_t1(s);
            tao2=timedelay2+delta_t2(s);
            tao3=timedelay3+delta_t3(s);
            tao4=timedelay4+delta_t4(s);
            
            posi1=[x1_near+delta_xi1(s) y1_near+delta_yi1(s) z1_near+delta_zi1(s)];
            posj1=[x1_far+delta_xj1(s) y1_far+delta_yj1(s) z1_far+delta_zj1(s)];
            
            posi2=[x2_near+delta_xi2(s) y2_near+delta_yi2(s) z2_near+delta_zi2(s)];
            posj2=[x2_far+delta_xj2(s) y2_far+delta_yj2(s) z2_far+delta_zj2(s)];
            
            posi3=[x3_near+delta_xi3(s) y3_near+delta_yi3(s) z3_near+delta_zi3(s)];
            posj3=[x3_far+delta_xj3(s) y3_far+delta_yj3(s) z3_far+delta_zj3(s)];
            
            posi4=[x4_near+delta_xi4(s) y4_near+delta_yi4(s) z4_near+delta_zi4(s)];
            posj4=[x4_far+delta_xj4(s) y4_far+delta_yj4(s) z4_far+delta_zj4(s)];
            
            poss=[x(p,q)+delta_xs(s) y(p,q)+delta_ys(s)];
            
%             zs1=Chuizhi(posi1,posj1,poss,c1,tao1);
%             zs2=Chuizhi(posi2,posj2,poss,c2,tao2);
%             zs3=Chuizhi(posi3,posj3,poss,c3,tao3);
%             zs4=Chuizhi(posi4,posj4,poss,c4,tao4);
            zs1=Chuizhi_ruan(posi1,posj1,poss,c1,tao1);
            zs2=Chuizhi_ruan(posi2,posj2,poss,c2,tao2);
            zs3=Chuizhi_ruan(posi3,posj3,poss,c3,tao3);
            zs4=Chuizhi_ruan(posi4,posj4,poss,c4,tao4);
            
            zs=(zs1+zs2+zs3+zs4)/4;
            
            delta_z(s)=abs(zs-z);
        end
        delta_Z(p,q)=mean(delta_z);
    end
end
%% 画图
figure
h=pcolor(x,y,delta_Z);hold on;
set(h,'edgecolor','none','facecolor','interp');
plot(x1_near,y1_near,'r*',x1_far,y1_far,'b*');hold on;
plot(x2_near,y2_near,'r*',x2_far,y2_far,'b*');hold on;
plot(x3_near,y3_near,'r*',x3_far,y3_far,'b*');hold on;
plot(x4_near,y4_near,'r*',x4_far,y4_far,'b*');hold on;
colorbar;
caxis([min(delta_Z(:)) 30]);
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);
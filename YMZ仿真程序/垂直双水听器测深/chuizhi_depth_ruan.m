close all;clear all;clc;
%% 参数设置
x_near=0;y_near=0;z_near=2;
x_far=-20;y_far=-20;z_far=80;
x=500;y=500;
z=(0:300);
esv=1500;
%% 误差设置
std_t=0.001;
std_p=1.5;
std_c=1;

length=10000;
delta_t=normrnd(0,std_t,1,length);  %随机生成一个均值为0，标准差为2 的1*1000随机数矩阵

delta_xi=normrnd(0,std_p,1,length);delta_yi=normrnd(0,std_p,1,length);delta_zi=normrnd(0,std_p,1,length);    
delta_xj=normrnd(0,std_p,1,length);delta_yj=normrnd(0,std_p,1,length);delta_zj=normrnd(0,std_p,1,length);  
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c=normrnd(0,std_c,1,length);
%% 垂直双阵元测深
n=size(z);
for p=1:n(2)
    for s=1:length
        dj=sqrt((x_far-x)^2+(y_far-y)^2+(z_far-z(p))^2);
        di=sqrt((x_near-x)^2+(y_near-y)^2+(z_near-z(p))^2);
        timedelay=(di/esv)-(dj/esv);

        c=esv+delta_c(s);
        tao=timedelay+delta_t(s);
        xi=x_near+delta_xi(s);
        yi=y_near+delta_yi(s);
        zi=z_near+delta_zi(s);
        xj=x_far+delta_xj(s);
        yj=y_far+delta_yj(s);
        zj=z_far+delta_zj(s);
        xs=x+delta_xs(s);
        ys=y+delta_ys(s);

        E=(zj-zi)/(c*tao);
        F=((xj-xs)^2+(yj-ys)^2+zj^2-(xi-xs)^2-(yi-ys)^2-zi^2)/(2*c*tao)+(c*tao)/2;
        if (tao>=0)
            zs=(2*(E*F-zj)+sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
        else
            zs=(2*(E*F-zj)-sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
        end
        delta_z(s)=abs(zs-z(p));
    end
    delta_Z(p)=mean(delta_z);
end
%% 画图
figure
plot(z,delta_Z,'b','LineWidth',3);hold on;
plot(z_near,18,'r*',z_far,18,'b*');hold on;
xlabel('目标实际深度/m','FontSize',14);
ylabel('深度测量误差/m','FontSize',14);

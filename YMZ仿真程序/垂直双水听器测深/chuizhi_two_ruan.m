clear all;clc;

%% 误差设置
std_t=0.001;
std_p=1.5;
std_h=1.5;
std_target=0;
std_c=1;

length=10000;
delta_t=normrnd(0,std_t,1,length);  %随机生成一个均值为0，标准差为2 的1*1000随机数矩阵

delta_xi=normrnd(0,std_p,1,length);delta_yi=normrnd(0,std_p,1,length);delta_zi=normrnd(0,std_h,1,length);    
delta_xj=normrnd(0,std_p,1,length);delta_yj=normrnd(0,std_p,1,length);delta_zj=normrnd(0,std_h,1,length);  
delta_xs=normrnd(0,std_target,1,length);delta_ys=normrnd(0,std_target,1,length);

delta_c=normrnd(0,std_c,1,length);
%% 参数设置
x_near=0;y_near=0;z_near=2;
x_far=-20;y_far=-20;z_far=80;
[x,y]=meshgrid((-500:10:500)); %目标移动范围
z=300;
esv=1500;
%% 垂直双阵元测深
n=size(x);
for p=1:n(1)
    for q=1:n(2)
        k=1;
        for s=1:length
            dj=sqrt((x_far-x(p,q))^2+(y_far-y(p,q))^2+(z_far-z)^2);
            
            di=sqrt((x_near-x(p,q))^2+(y_near-y(p,q))^2+(z_near-z)^2);
            timedelay=(di/esv)-(dj/esv);

            c=esv+delta_c(s);
            tao=timedelay+delta_t(s);
            xi=x_near+delta_xi(s);
            yi=y_near+delta_yi(s);
            zi=z_near+delta_zi(s);
            xj=x_far+delta_xj(s);
            yj=y_far+delta_yj(s);
            zj=z_far+delta_zj(s);
            xs=x(p,q)+delta_xs(s);
            ys=y(p,q)+delta_ys(s);
            
            E=(zj-zi)/(c*tao);
            F=((xj-xs)^2+(yj-ys)^2+zj^2-(xi-xs)^2-(yi-ys)^2-zi^2)/(2*c*tao)+(c*tao)/2;
            if (tao>=0)
                zs=(2*(E*F-zj)+sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
            else
                zs=(2*(E*F-zj)-sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
            end
            delta_z(s)=abs(zs-z);
        end
        delta_Z(p,q)=mean(delta_z);
    end
end
%% 画图
figure
h=pcolor(x,y,delta_Z);hold on;
set(h,'edgecolor','none','facecolor','interp');
plot(x_near,y_near,'r*',x_far,y_far,'b*');hold on;
colorbar;
caxis([min(delta_Z(:)) 30]);
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);
minDelta =  min(delta_Z(:));
% figure
% x_range = -1000:10:1000;
% y_range = -1000:10:1000;
% [X,Y] = meshgrid(x_range,y_range);
% h=surf(X,Y,delta_Z);hold on;
% 
% view([0,90]);
% shading interp
% plot(x_near,y_near,'r*',x_far,y_far,'r*');hold on;
% colorbar;
% caxis([min(delta_Z(:)) 30]);
% 
% 
% xlabel('x轴/m','FontSize',14);
% ylabel('y轴/m','FontSize',14);
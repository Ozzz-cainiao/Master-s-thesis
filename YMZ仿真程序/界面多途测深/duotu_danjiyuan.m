close all;clear all;clc;
%% 参数设置
xi=0;yi=0;di=80;
[x,y]=meshgrid((-500:10:500)); %目标移动范围
z=300;
esv=1500;
%% 误差设置
std_t=0.001;
std_p=1.5;
std_c=1;

length=10000;
delta_t=normrnd(0,std_t,1,length);  %随机生成一个均值为0，标准差为2 的1*1000随机数矩阵

delta_xi=normrnd(0,std_p,1,length);delta_yi=normrnd(0,std_p,1,length);delta_zi=normrnd(0,std_p,1,length);    
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c=normrnd(0,std_c,1,length);
%% 界面多途测深
n=size(x);
for p=1:n(1)
    for q=1:n(2)
        for s=1:length
            ri_rel=sqrt((xi-x(p,q))^2+(yi-y(p,q))^2);%目标到阵元的真实水平距离
            timedelay=(sqrt(ri_rel^2+(z+di)^2)-sqrt(ri_rel^2+(z-di)^2))/esv;%直达声和反射声的真实时延差
            
            c=esv+delta_c(s);%声速加入误差
            tao=timedelay+delta_t(s);%时延差加入误差
            ri=sqrt((xi+delta_xi(s)-x(p,q)-delta_xs(s))^2+(yi+delta_yi(s)-y(p,q)-delta_ys(s))^2);%加入目标定位误差后的目标到阵元水平距离
                                
            Ri=c*tao;
            zs=sqrt((ri^2-(Ri^2/4-di^2))/(4*di^2/(Ri^2)-1));

            delta_z(s)=abs(zs-z);
        end
        delta_Z(p,q)=mean(delta_z);
    end
end
%% 画图
figure
h=pcolor(x,y,delta_Z);hold on;
set(h,'edgecolor','none','facecolor','interp');
colorbar;
caxis([min(delta_Z(:)) 30]);
plot(0,0,'b*');hold on;
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);
close all;
clear all;clc;
%% 参数设置
xi=0;yi=0;di=80;
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
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c=normrnd(0,std_c,1,length);
%% 界面多途测深
n=size(z);
for p=1:n(2)
    for s=1:length
        ri_rel=sqrt((xi-x)^2+(yi-y)^2);%目标到阵元的真实水平距离
        timedelay=(sqrt(ri_rel^2+(z(p)+di)^2)-sqrt(ri_rel^2+(z(p)-di)^2))/esv;%直达声和反射声的真实时延差

        c=esv+delta_c(s);%声速加入误差
        tao=timedelay+delta_t(s);%时延差加入误差
        
        ri=sqrt((xi+delta_xi(s)-x-delta_xs(s))^2+(yi+delta_yi(s)-y-delta_ys(s))^2);%加入目标定位误差后的目标到阵元水平距离

        Ri=c*tao;
        zs=sqrt((ri^2-(Ri^2/4-(di+delta_zi(s))^2))/(4*(di+delta_zi(s))^2/(Ri^2)-1));

        if zs-z(p) > 0
            delta_z(s)=abs(zs-z(p));
        end
        
    end
    delta_Z(p)=(mean(delta_z));
end
%% 画图
figure
plot(z,delta_Z,'b','LineWidth',3);hold on;
plot(80,3,'b*');
xlabel('目标实际深度/m','FontSize',14);
ylabel('深度测量误差/m','FontSize',14);
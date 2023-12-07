% close all;
clear all;clc;
%% 参数设置
xi1=500;yi1=500;di1=80;
xi2=500;yi2=-500;di2=80;
xi3=-500;yi3=-500;di3=80;
xi4=-500;yi4=500;di4=80;
% xi1=200;yi1=200;di1=40;
% xi2=200;yi2=-200;di2=40;
% xi3=-200;yi3=-200;di3=40;
% xi4=-200;yi4=200;di4=40;
x=0;y=0;
z=(0:300);
esv=1500;
%% 误差设置
std_t=0.001;
std_p=1.5;
std_c=1;

length=1000;
delta_t1=normrnd(0,std_t,1,length);  
delta_t2=normrnd(0,std_t,1,length);
delta_t3=normrnd(0,std_t,1,length);
delta_t4=normrnd(0,std_t,1,length);%随机生成一个均值为0，标准差为2 的1*1000随机数矩阵

delta_xi1=normrnd(0,std_p,1,length);delta_yi1=normrnd(0,std_p,1,length);delta_zi1=normrnd(0,std_p,1,length);    
delta_xi2=normrnd(0,std_p,1,length);delta_yi2=normrnd(0,std_p,1,length);delta_zi2=normrnd(0,std_p,1,length);    
delta_xi3=normrnd(0,std_p,1,length);delta_yi3=normrnd(0,std_p,1,length);delta_zi3=normrnd(0,std_p,1,length);    
delta_xi4=normrnd(0,std_p,1,length);delta_yi4=normrnd(0,std_p,1,length);delta_zi4=normrnd(0,std_p,1,length);    
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c1=normrnd(0,std_c,1,length);
delta_c2=normrnd(0,std_c,1,length);
delta_c3=normrnd(0,std_c,1,length);
delta_c4=normrnd(0,std_c,1,length);
%% 界面多途测深
n=size(z);
for p=1:n(2)
    for s=1:length
        ri_rel1=sqrt((xi1-x)^2+(yi1-y)^2);%目标到阵元的真实水平距离
        timedelay1=(sqrt(ri_rel1^2+(z(p)+di1)^2)-sqrt(ri_rel1^2+(z(p)-di1)^2))/esv;%直达声和反射声的真实时延差

        ri_rel2=sqrt((xi2-x)^2+(yi2-y)^2);
        timedelay2=(sqrt(ri_rel2^2+(z(p)+di2)^2)-sqrt(ri_rel2^2+(z(p)-di2)^2))/esv;

        ri_rel3=sqrt((xi3-x)^2+(yi3-y)^2);
        timedelay3=(sqrt(ri_rel3^2+(z(p)+di3)^2)-sqrt(ri_rel3^2+(z(p)-di3)^2))/esv;

        ri_rel4=sqrt((xi4-x)^2+(yi4-y)^2);
        timedelay4=(sqrt(ri_rel4^2+(z(p)+di4)^2)-sqrt(ri_rel4^2+(z(p)-di4)^2))/esv;

        c1=esv+delta_c1(s);
        c2=esv+delta_c2(s);
        c3=esv+delta_c3(s);
        c4=esv+delta_c4(s);%声速加入误差

        tao1=timedelay1+delta_t1(s);
        tao2=timedelay2+delta_t2(s);
        tao3=timedelay3+delta_t3(s);
        tao4=timedelay4+delta_t4(s);%时延差加入误差
        
        ri1=sqrt((xi1+delta_xi1(s)-x-delta_xs(s))^2+(yi1+delta_yi1(s)-y-delta_ys(s))^2);
        ri2=sqrt((xi2+delta_xi2(s)-x-delta_xs(s))^2+(yi2+delta_yi2(s)-y-delta_ys(s))^2);
        ri3=sqrt((xi3+delta_xi3(s)-x-delta_xs(s))^2+(yi3+delta_yi3(s)-y-delta_ys(s))^2);
        ri4=sqrt((xi4+delta_xi4(s)-x-delta_xs(s))^2+(yi4+delta_yi4(s)-y-delta_ys(s))^2);%加入目标定位误差后的目标到阵元水平距离

        Ri1=c1*tao1;
        zs1=sqrt((ri1^2-(Ri1^2/4-di1^2))/(4*di1^2/(Ri1^2)-1));

        Ri2=c2*tao2;
        zs2=sqrt((ri2^2-(Ri2^2/4-di2^2))/(4*di2^2/(Ri2^2)-1));

        Ri3=c3*tao3;
        zs3=sqrt((ri3^2-(Ri3^2/4-di3^2))/(4*di3^2/(Ri3^2)-1));

        Ri4=c4*tao4;
        zs4=sqrt((ri4^2-(Ri4^2/4-di4^2))/(4*di4^2/(Ri4^2)-1));

        zs=(zs1+zs2+zs3+zs4)/4;

        if zs-z(p) > 0
            delta_z(s)=abs(zs-z(p));
        end
        
    end
    delta_Z(p)=abs(mean(delta_z));
end
%% 画图
figure
plot(z,delta_Z,'b','LineWidth',3);hold on;
plot(80,1,'b*');
xlabel('目标实际深度/m','FontSize',14);
ylabel('深度测量误差/m','FontSize',14);
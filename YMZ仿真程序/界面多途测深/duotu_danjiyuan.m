close all;clear all;clc;
%% ��������
xi=0;yi=0;di=80;
[x,y]=meshgrid((-500:10:500)); %Ŀ���ƶ���Χ
z=300;
esv=1500;
%% �������
std_t=0.001;
std_p=1.5;
std_c=1;

length=10000;
delta_t=normrnd(0,std_t,1,length);  %�������һ����ֵΪ0����׼��Ϊ2 ��1*1000���������

delta_xi=normrnd(0,std_p,1,length);delta_yi=normrnd(0,std_p,1,length);delta_zi=normrnd(0,std_p,1,length);    
delta_xs=normrnd(0,std_p,1,length);delta_ys=normrnd(0,std_p,1,length);

delta_c=normrnd(0,std_c,1,length);
%% �����;����
n=size(x);
for p=1:n(1)
    for q=1:n(2)
        for s=1:length
            ri_rel=sqrt((xi-x(p,q))^2+(yi-y(p,q))^2);%Ŀ�굽��Ԫ����ʵˮƽ����
            timedelay=(sqrt(ri_rel^2+(z+di)^2)-sqrt(ri_rel^2+(z-di)^2))/esv;%ֱ�����ͷ���������ʵʱ�Ӳ�
            
            c=esv+delta_c(s);%���ټ������
            tao=timedelay+delta_t(s);%ʱ�Ӳ�������
            ri=sqrt((xi+delta_xi(s)-x(p,q)-delta_xs(s))^2+(yi+delta_yi(s)-y(p,q)-delta_ys(s))^2);%����Ŀ�궨λ�����Ŀ�굽��Ԫˮƽ����
                                
            Ri=c*tao;
            zs=sqrt((ri^2-(Ri^2/4-di^2))/(4*di^2/(Ri^2)-1));

            delta_z(s)=abs(zs-z);
        end
        delta_Z(p,q)=mean(delta_z);
    end
end
%% ��ͼ
figure
h=pcolor(x,y,delta_Z);hold on;
set(h,'edgecolor','none','facecolor','interp');
colorbar;
caxis([min(delta_Z(:)) 30]);
plot(0,0,'b*');hold on;
xlabel('x��/m','FontSize',14);
ylabel('y��/m','FontSize',14);
close all;
clear all;clc;
%% ��������
xi=0;yi=0;di=80;
x=500;y=500;
z=(0:300);
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
n=size(z);
for p=1:n(2)
    for s=1:length
        ri_rel=sqrt((xi-x)^2+(yi-y)^2);%Ŀ�굽��Ԫ����ʵˮƽ����
        timedelay=(sqrt(ri_rel^2+(z(p)+di)^2)-sqrt(ri_rel^2+(z(p)-di)^2))/esv;%ֱ�����ͷ���������ʵʱ�Ӳ�

        c=esv+delta_c(s);%���ټ������
        tao=timedelay+delta_t(s);%ʱ�Ӳ�������
        
        ri=sqrt((xi+delta_xi(s)-x-delta_xs(s))^2+(yi+delta_yi(s)-y-delta_ys(s))^2);%����Ŀ�궨λ�����Ŀ�굽��Ԫˮƽ����

        Ri=c*tao;
        zs=sqrt((ri^2-(Ri^2/4-(di+delta_zi(s))^2))/(4*(di+delta_zi(s))^2/(Ri^2)-1));

        if zs-z(p) > 0
            delta_z(s)=abs(zs-z(p));
        end
        
    end
    delta_Z(p)=(mean(delta_z));
end
%% ��ͼ
figure
plot(z,delta_Z,'b','LineWidth',3);hold on;
plot(80,3,'b*');
xlabel('Ŀ��ʵ�����/m','FontSize',14);
ylabel('��Ȳ������/m','FontSize',14);
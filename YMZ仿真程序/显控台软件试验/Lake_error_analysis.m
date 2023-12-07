clc;clear;close all;
load Lake_error_analysis.mat
%% �������
dt = 0.1;
T = 291;
Z = zeros(2,T/dt);             %Ŀ��켣
mode=1;
if mode ==1
    Z0 =[40,-0.2,]';           %Ŀ���ʼ�˶�״̬[z,vz]';
else
    Z0 =[40,0]';           %Ŀ���ʼ�˶�״̬[z,vz]';
end
%----------����ֱ���˶�-------------
F1 = [1,dt;
      0,1];
%----------��ֹ-------------  
F2 = [1,0;
      0,1];
%-----------������ʵ�켣----------
k=1;
for t = dt:dt:T
    if t==dt
        Z(:,k)=Z0;
    elseif t>dt&&t<75
        Z(:,k)=F2*Z(:,k-1);                          %��ֹ
    elseif t>=75&&t<250
        Z(:,k)=F1*Z(:,k-1);                          %�����˶�����  
    elseif t>=250&&t<=T+1
        Z(:,k)=F2*Z(:,k-1);                          %��ֹ     
    end
    k=k+1;
end
z=Z(1,:);
%% ����ˮƽ
%��ֵ
num=length(z);
chazhix = 1:1:num/10;
chazhiy1 = shipx(1:num/10);
chazhiy2 = shipy(1:num/10);
xuhao    = 0:0.1:num/10-0.1;
x = interp1(chazhix,chazhiy1,xuhao,'spline');
y = interp1(chazhix,chazhiy2,xuhao,'spline');

%RMSE���
for period=1:length(xs)
    deltax(period) = (xs(period)-x(period))^2;
    deltay(period) = (ys(period)-y(period))^2;
    delta(period)  = sqrt(deltax(period) + deltay(period));
end
sumdelta = sum(delta.^2);
fang_shuiping =sqrt(sumdelta/length(xs));

%RMSE���
for period=1:length(xs)
    deltaz(period) = zs(period)-z(period);
end
sumdeltaz = sum(deltaz.^2);
fang_z =sqrt(sumdeltaz/length(zs));

lenship = length(shipx);
figure
plot(xs,ys,'r.');hold on;
plot(shipx(1:lenship-20),shipy(1:lenship-20),'b.');
legend('Ŀ�����ֵ','Ŀ����ʵֵ');
xlabel('X/m');
ylabel('Y/m');

figure
plot(zs,'r.');hold on;
plot(z,'b.');
legend('Ŀ�����ֵ','Ŀ����ʵֵ');
xlabel('֡���/m');
ylabel('���/m');

figure
plot(delta,'b.');
xlabel('֡���/m');
ylabel('���/m');
figure
plot(abs(deltaz),'b.');
xlabel('֡���/m');
ylabel('���/m');
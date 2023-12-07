clc;clear;close all;
line_near=500;
delta=-3;
 
%����GPS�ϴ�λ��
x1_near=line_near;y1_near=line_near;z1_near=2;

x2_near=line_near;y2_near=20;z2_near=2;

x3_near=20;y3_near=20;z3_near=2;

x4_near=20;y4_near=line_near;z4_near=2;

x1=x1_near+delta;y1=y1_near+delta;z1=30;              %��i���ű����꣬i=1,2,3,4
x2=x2_near+delta;y2=y2_near+delta;z2=30;             
x3=x3_near+delta;y3=y3_near+delta;z3=30;              
x4=x4_near+delta;y4=y4_near+delta;z4=30;
x1_far=x1;y1_far=y1;z1_far=30;
x2_far=x2;y2_far=y2;z2_far=30;
x3_far=x3;y3_far=y3;z3_far=30;
x4_far=x4;y4_far=y4;z4_far=30;
z_far=30;z_near=2;
%% Ŀ��켣Ԥ��
T_source = 0.1; %��������
dt = T_source;%�۲�����
T = 40;       %�۲�ʱ��
c = 1500;     %����
mode = 1;     %ģʽ1���˶�  ģʽ2����ֹ
%% Ŀ��켣����ˮƽ
X = zeros(4,T/dt);             %Ŀ��켣
if mode ==1
    v = 0.1;                       %Ŀ����ٶ�
    X0 =[150,5,200,2.5]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
else
    v = 0;                       %Ŀ����ٶ�
    X0 =[250,0,250,0]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
end

%----------����ֱ���˶�-------------
F1 = [1,dt,0,0;
    0,1,0,0;
    0,0,1,dt;
    0,0,0,1,];
%----------��ʱ��ת���˶�-------------
F2 = [1,sin(v*dt)/v,0,-(1-cos(v*dt))/v;
    0,cos(v*dt),0,-sin(v*dt);
    0,(1-cos(v*dt))/v,1,sin(v*dt)/v;
    0,sin(v*dt),0,cos(v*dt)];
%----------˳ʱ��ת���˶�-------------
F3 = [1,-sin(-v*dt)/v,0,(1-cos(-v*dt))/v;
    0,cos(-v*dt),0,-sin(-v*dt);
    0,-(1-cos(-v*dt))/v,1,-sin(-v*dt)/v;
    0,sin(-v*dt),0,cos(-v*dt);];
%-----------������ʵ�켣----------
k=1;
for t = dt:dt:T
    if t==dt
        X(:,k)=X0;
    elseif t>dt&&t<10
        X(:,k)=F1*X(:,k-1);                          %�����˶�����
    elseif t>=10&&t<20
        if mode == 2
            F=F1;
        else
            F=F3;
        end
        X(:,k)=F*X(:,k-1);                          %��ʱ��ת���˶�����
    elseif t>=20&&t<30
        if mode == 2
            F=F1;
        else
            F=F2;
        end
        X(:,k)=F*X(:,k-1);
    elseif t>=30&&t<=40+1
        X(:,k)=F1*X(:,k-1);                          %�����˶�����
    end
    k=k+1;
end
%-----------������ʵ�켣----------
% k=1;
% for t = dt:dt:T
%     if t==dt
%         X(:,k)=X0;
%     elseif t>dt&&t<T+1
%         X(:,k)=F1*X(:,k-1);                          %�����˶�����
%     end
%     k=k+1;
% end


x=X(1,:);
y=X(3,:);
%% Ŀ��켣�������
Z = zeros(2,T/dt);             %Ŀ��켣
if mode ==1
    Z0 =[60,-1,]';           %Ŀ���ʼ�˶�״̬[z,vz]';
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
    elseif t>dt&&t<41
        Z(:,k)=F1*Z(:,k-1);                          %�����˶�����
%     elseif t>=10&&t<20
%         Z(:,k)=F2*Z(:,k-1);                          %��ֹ
%     elseif t>=20&&t<30
%         Z(:,k)=F1*Z(:,k-1);                          %�����˶�����  
%     elseif t>=30&&t<=40+1
%         Z(:,k)=F2*Z(:,k-1);                          %��ֹ     
    end
    k=k+1;
end
z=Z(1,:);
num=length(Z);%Ŀ��켣����
figure
plot(x,y,'r.');
title('Ŀ��ˮƽλ��');
xlim([0,500]);
ylim([0,500]);
figure
plot(z,'r');
title('Ŀ�����');
fidt = fopen(['C:\data\receiveData\','targetreceivData.txt'],'w');
fprintf(fidt,'%s\t','period');
fprintf(fidt,'%s\t','lat');
fprintf(fidt,'%s\t','lon');
fprintf(fidt,'%s\t','x');
fprintf(fidt,'%s\t','y');
fprintf(fidt,'%s\t','z');
fprintf(fidt,'\r\n');
for itd = 1: num
    if mod(itd,10)==1
        [latt(itd),longt(itd)] = ACalculateDistance(109.60, 17.98361111,x(itd), y(itd),1);
%     [xt(itd),yt(itd)] = CalculateDistance(109.60, 17.98361111,longt(itd), latt(itd),1);
    fprintf(fidt,'%d\t',floor(itd/10)+1);
    fprintf(fidt,'%f\t\t',latt(itd));
    fprintf(fidt,'%f\t\t',longt(itd));
    fprintf(fidt,'%f\t\t',x(itd));
    fprintf(fidt,'%f\t\t',y(itd));
    fprintf(fidt,'%f\t\t',z(itd));
    
    fprintf(fidt,'\r\n');
    end
    
end

%% �������
diedai=1;%100�ε���

std_p=0;
delta_x1=normrnd(0,std_p,1,diedai);delta_y1=normrnd(0,std_p,1,diedai);
delta_x2=normrnd(0,std_p,1,diedai);delta_y2=normrnd(0,std_p,1,diedai);
delta_x3=normrnd(0,std_p,1,diedai);delta_y3=normrnd(0,std_p,1,diedai);
delta_x4=normrnd(0,std_p,1,diedai);delta_y4=normrnd(0,std_p,1,diedai);

std_t=0;
delta_t=normrnd(0,std_t,1,num);
std_angle=0/180*pi;%��λΪpi
delta_angle1=normrnd(0,std_angle,1,diedai);
delta_angle2=normrnd(0,std_angle,1,diedai);
delta_angle3=normrnd(0,std_angle,1,diedai);
delta_angle4=normrnd(0,std_angle,1,diedai);
%% ����Ƕ�
%��������ĽǶ�
for i=1:num
    alpha1(i,:)=atan2(y(i)-y1,x(i)-x1)+delta_angle1;%200�����100�ε���
    alpha2(i,:)=atan2(y(i)-y2,x(i)-x2)+delta_angle2;
    alpha3(i,:)=atan2(y(i)-y3,x(i)-x3)+delta_angle3;
    alpha4(i,:)=atan2(y(i)-y4,x(i)-x4)+delta_angle4;
end
figure
plot(alpha1,'r');hold on;
plot(alpha2,'b');hold on;
plot(alpha3,'g');hold on;
plot(alpha4,'m');
% xlim([1:150]);
legend('��Ԫ1','��Ԫ2','��Ԫ3','��Ԫ4');
title('Ŀ�귽λ');


%% ����ʱ��
    x1_far = x1_far+delta_x1;y1_far = y1_far+delta_x1;
    x1_near = x1_near+delta_x1;y1_near = y1_near+delta_x1;
    x2_far = x2_far+delta_x1;y2_far = y2_far+delta_x1;
    x2_near = x2_near+delta_x1;y2_near = y2_near+delta_x1;
    x3_far = x3_far+delta_x1;y3_far = y3_far+delta_x1;
    x3_near = x3_near+delta_x1;y3_near = y3_near+delta_x1;
    x4_far = x4_far+delta_x1;y4_far = y4_far+delta_x1;
    x4_near = x4_near+delta_x1;y4_near = y4_near+delta_x1;
    z_near = z1_near + delta_x1;z_far = z1_far + delta_x1;
    %����λ���
%         for i=1:num
%     dis1_far=sqrt((x1_far-x(i))^2+(y1_far-y(i))^2);%ˮƽ����
%     dis1_near=sqrt((x1_near-x(i))^2+(y1_near-y(i))^2);%ˮƽ����
%     taoH1(i)=-(sqrt(dis1_far^2+(z(i)-z_far)^2)-sqrt(dis1_near^2+(z(i)-z_near)^2))/c ;%��ֱ��-Զֱ��--�����
%     taoZ1(i)=-(sqrt(dis1_far^2+(z(i)-z_far)^2)-sqrt(dis1_far^2+(z(i)+z_far)^2))/c ;%Զ����-Զֱ��--�����
%     
%     dis2_far=sqrt((x2_far-x(i))^2+(y2_far-y(i))^2);%ˮƽ����
%     dis2_near=sqrt((x2_near-x(i))^2+(y2_near-y(i))^2);%ˮƽ����
%     taoH2(i)=-(sqrt(dis2_far^2+(z(i)-z_far)^2)-sqrt(dis2_near^2+(z(i)-z_near)^2))/c ;%��ֱ��-Զֱ��--�����
%     taoZ2(i)=-(sqrt(dis2_far^2+(z(i)-z_far)^2)-sqrt(dis2_far^2+(z(i)+z_far)^2))/c ;%Զ����-Զֱ��--�����
%     
%     dis3_far=sqrt((x3_far-x(i))^2+(y3_far-y(i))^2);%ˮƽ����
%     dis3_near=sqrt((x3_near-x(i))^2+(y3_near-y(i))^2);%ˮƽ����
%     taoH3(i)=-(sqrt(dis3_far^2+(z(i)-z_far)^2)-sqrt(dis3_near^2+(z(i)-z_near)^2))/c ;%��ֱ��-Զֱ��--�����
%     taoZ3(i)=-(sqrt(dis3_far^2+(z(i)-z_far)^2)-sqrt(dis3_far^2+(z(i)+z_far)^2))/c ;%Զ����-Զֱ��--�����
%     
%     dis4_far=sqrt((x4_far-x(i))^2+(y4_far-y(i))^2);%ˮƽ����
%     dis4_near=sqrt((x4_near-x(i))^2+(y4_near-y(i))^2);%ˮƽ����
%     taoH4(i)=-(sqrt(dis4_far^2+(z(i)-z_far)^2)-sqrt(dis4_near^2+(z(i)-z_near)^2))/c ;%��ֱ��-Զֱ��--�����
%     taoZ4(i)=-(sqrt(dis4_far^2+(z(i)-z_far)^2)-sqrt(dis4_far^2+(z(i)+z_far)^2))/c ;%Զ����-Զֱ��--�����
%     end
    for i=1:num
    dis1_far=sqrt((x1_far-x(i))^2+(y1_far-y(i))^2);%ˮƽ����
    dis1_near=sqrt((x1_near-x(i))^2+(y1_near-y(i))^2);%ˮƽ����
    taoH1(i)=-(sqrt(dis1_far^2+(z(i)-z_far)^2)-sqrt(dis1_near^2+(z(i)-z_near)^2))/c + delta_t(i);%��ֱ��-Զֱ��--�����
    taoZ1(i)=-(sqrt(dis1_far^2+(z(i)-z_far)^2)-sqrt(dis1_far^2+(z(i)+z_far)^2))/c + delta_t(i);%Զ����-Զֱ��--�����
    
    dis2_far=sqrt((x2_far-x(i))^2+(y2_far-y(i))^2);%ˮƽ����
    dis2_near=sqrt((x2_near-x(i))^2+(y2_near-y(i))^2);%ˮƽ����
    taoH2(i)=-(sqrt(dis2_far^2+(z(i)-z_far)^2)-sqrt(dis2_near^2+(z(i)-z_near)^2))/c + delta_t(i);%��ֱ��-Զֱ��--�����
    taoZ2(i)=-(sqrt(dis2_far^2+(z(i)-z_far)^2)-sqrt(dis2_far^2+(z(i)+z_far)^2))/c + delta_t(i);%Զ����-Զֱ��--�����
    
    dis3_far=sqrt((x3_far-x(i))^2+(y3_far-y(i))^2);%ˮƽ����
    dis3_near=sqrt((x3_near-x(i))^2+(y3_near-y(i))^2);%ˮƽ����
    taoH3(i)=-(sqrt(dis3_far^2+(z(i)-z_far)^2)-sqrt(dis3_near^2+(z(i)-z_near)^2))/c + delta_t(i);%��ֱ��-Զֱ��--�����
    taoZ3(i)=-(sqrt(dis3_far^2+(z(i)-z_far)^2)-sqrt(dis3_far^2+(z(i)+z_far)^2))/c + delta_t(i);%Զ����-Զֱ��--�����
    
    dis4_far=sqrt((x4_far-x(i))^2+(y4_far-y(i))^2);%ˮƽ����
    dis4_near=sqrt((x4_near-x(i))^2+(y4_near-y(i))^2);%ˮƽ����
    taoH4(i)=-(sqrt(dis4_far^2+(z(i)-z_far)^2)-sqrt(dis4_near^2+(z(i)-z_near)^2))/c + delta_t(i);%��ֱ��-Զֱ��--�����
    taoZ4(i)=-(sqrt(dis4_far^2+(z(i)-z_far)^2)-sqrt(dis4_far^2+(z(i)+z_far)^2))/c + delta_t(i);%Զ����-Զֱ��--�����
    end


%% �洢ʱ��

fid = fopen(['C:\data\receiveData\','buoy1#receivData.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','Angle');
fprintf(fid,'%s\t','TimeDelay1');
fprintf(fid,'%s\t','TimeDelay2');
fprintf(fid,'\r\n');
for itd = 1: num
    fprintf(fid,'%d\t',itd); 
    fprintf(fid,'%f\t',alpha1(itd));
    fprintf(fid,'%f\t',taoZ1(itd));
    fprintf(fid,'%f\t',taoH1(itd)); 
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy2#receivData.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','Angle');
fprintf(fid,'%s\t','TimeDelay1');
fprintf(fid,'%s\t','TimeDelay2');
fprintf(fid,'\r\n');
for itd = 1: num
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%f\t',alpha2(itd));
    fprintf(fid,'%f\t',taoZ2(itd));
    fprintf(fid,'%f\t',taoH2(itd));
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy3#receivData.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','Angle');
fprintf(fid,'%s\t','TimeDelay1');
fprintf(fid,'%s\t','TimeDelay2');
fprintf(fid,'\r\n');
for itd = 1: num
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%f\t',alpha3(itd));
    fprintf(fid,'%f\t',taoZ3(itd));
    fprintf(fid,'%f\t',taoH3(itd));
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy4#receivData.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','Angle');
fprintf(fid,'%s\t','TimeDelay1');
fprintf(fid,'%s\t','TimeDelay2');
fprintf(fid,'\r\n');
for itd = 1: num
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%f\t',alpha4(itd));
    fprintf(fid,'%f\t',taoZ4(itd));
    fprintf(fid,'%f\t',taoH4(itd));
    fprintf(fid,'\r\n');
end

%% �洢��Ԫ��γ��
fid = fopen(['C:\data\receiveData\','buoy1#Position.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','x_near');
fprintf(fid,'%s\t','y_near');
fprintf(fid,'%s\t','z_near');
fprintf(fid,'%s\t','x_far');
fprintf(fid,'%s\t','y_far');
fprintf(fid,'%s\t','z_far');
fprintf(fid,'\r\n');
num10=num/10;
for itd = 1: num10
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%.2f\t',x1_near);
    fprintf(fid,'%.2f\t',y1_near);
    fprintf(fid,'%.2f\t',z1_near);
    fprintf(fid,'%.2f\t',x1_far);
    fprintf(fid,'%.2f\t',y1_far);
    fprintf(fid,'%.2f\t',z1_far);
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy2#Position.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','x_near');
fprintf(fid,'%s\t','y_near');
fprintf(fid,'%s\t','z_near');
fprintf(fid,'%s\t','x_far');
fprintf(fid,'%s\t','y_far');
fprintf(fid,'%s\t','z_far');
fprintf(fid,'\r\n');
for itd = 1: num/10
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%.2f\t',x2_near);
    fprintf(fid,'%.2f\t',y2_near);
    fprintf(fid,'%.2f\t',z2_near);
    fprintf(fid,'%.2f\t',x2_far);
    fprintf(fid,'%.2f\t',y2_far);
    fprintf(fid,'%.2f\t',z2_far);
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy3#Position.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','x_near');
fprintf(fid,'%s\t','y_near');
fprintf(fid,'%s\t','z_near');
fprintf(fid,'%s\t','x_far');
fprintf(fid,'%s\t','y_far');
fprintf(fid,'%s\t','z_far');
fprintf(fid,'\r\n');
for itd = 1: num/10
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%.2f\t',x3_near);
    fprintf(fid,'%.2f\t',y3_near);
    fprintf(fid,'%.2f\t',z3_near);
    fprintf(fid,'%.2f\t',x3_far);
    fprintf(fid,'%.2f\t',y3_far);
    fprintf(fid,'%.2f\t',z3_far);
    fprintf(fid,'\r\n');
end
fid = fopen(['C:\data\receiveData\','buoy4#Position.txt'],'w');
fprintf(fid,'%s\t','period');
fprintf(fid,'%s\t','x_near');
fprintf(fid,'%s\t','y_near');
fprintf(fid,'%s\t','z_near');
fprintf(fid,'%s\t','x_far');
fprintf(fid,'%s\t','y_far');
fprintf(fid,'%s\t','z_far');
fprintf(fid,'\r\n');
for itd = 1: num/10
    fprintf(fid,'%d\t',itd);
    fprintf(fid,'%.2f\t',x4_near);
    fprintf(fid,'%.2f\t',y4_near);
    fprintf(fid,'%.2f\t',z4_near);
    fprintf(fid,'%.2f\t',x4_far);
    fprintf(fid,'%.2f\t',y4_far);
    fprintf(fid,'%.2f\t',z4_far);
    fprintf(fid,'\r\n');
end

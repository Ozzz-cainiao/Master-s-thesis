close all;clear all;clc;

%% ��������
x1=1000;y1=1000;z1=0;%��i���ű����꣬i=1,2,3,4
x2=1000;y2=0;z2=0;             
x3=0;y3=0;z3=0;              
x4=0;y4=1000;z4=0; 
z=0;%Ŀ�����0

T=1;%ͬ��������Ϊ0.1s
v=40;%Ŀ�꺽��40m/s
esv=1500;%������ʵֵ
%% Ŀ��켣Ԥ��
x_begin=180;
y_begin=250;%Ŀ����ʼ�㣨-800��800��

theta=pi/6;%Ŀ�꺽�нǶ���������30��

num=200;%Ŀ����400���켣��
for i=1:num
    x(i)=x_begin+(i-1)*v*0.1*T*cos(theta);
    y(i)=y_begin+(i-1)*v*0.1*T*sin(theta);
end

figure%Ŀ����ʵ�켣
plot(x,y,'b*','MarkerSize',3);hold on
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro',x4,y4,'ro');hold on
axis equal
axis([0 1000 0 1000]);
%% �������
std_t=0.001;
std_p=3;
std_c=1;

length=100;
delta_t1=normrnd(0,std_t,1,length);  %�������һ����ֵΪ0����׼��Ϊ2 ��1*1000���������
delta_t2=normrnd(0,std_t,1,length);     
delta_t3=normrnd(0,std_t,1,length);
delta_t4=normrnd(0,std_t,1,length);

delta_x1=normrnd(0,std_p,1,length);delta_y1=normrnd(0,std_p,1,length);dz1=0;    
delta_x2=normrnd(0,std_p,1,length);delta_y2=normrnd(0,std_p,1,length);dz2=0;    
delta_x3=normrnd(0,std_p,1,length);delta_y3=normrnd(0,std_p,1,length);dz3=0;    
delta_x4=normrnd(0,std_p,1,length);delta_y4=normrnd(0,std_p,1,length);dz4=0;   

delta_c=normrnd(0,std_c,1,length);
%% ģ����ѧ��������
%����GPS�ϴ�λ��
pos_x1=x1+delta_x1;pos_y1=y1+delta_y1;%100�ε���
pos_x2=x2+delta_x2;pos_y2=y2+delta_y2;
pos_x3=x3+delta_x3;pos_y3=y3+delta_y3;
pos_x4=x4+delta_x4;pos_y4=y4+delta_y4;
for i=1:length
    pos1(i,:)=[pos_x1(i) pos_y1(i) z1];%100�ε�����[x y z]����
    pos2(i,:)=[pos_x2(i) pos_y2(i) z2];
    pos3(i,:)=[pos_x3(i) pos_y3(i) z3];
    pos4(i,:)=[pos_x4(i) pos_y4(i) z4];
end
%�������������
c=esv+delta_c;%100�ε���
%ʵ�ʾ���
dis1=sqrt((x-x1).^2+(y-y1).^2);%400��Ŀ�꺽��
dis2=sqrt((x-x2).^2+(y-y2).^2);
dis3=sqrt((x-x3).^2+(y-y3).^2);
dis4=sqrt((x-x4).^2+(y-y4).^2);
%ʵ��ʱ��
time1=dis1./esv;
time2=dis2./esv;
time3=dis3./esv;
time4=dis4./esv;
%���������ʱ��
for i=1:num
    t1(i,:)=time1(i)+delta_t1;%400�����100�ε���
    t2(i,:)=time2(i)+delta_t2;
    t3(i,:)=time3(i)+delta_t3;
    t4(i,:)=time4(i)+delta_t4;  
end
%ͬ���������ʱ��
tb_t1=mod(t1,T);%400�����100�ε���
tb_t2=mod(t2,T);
tb_t3=mod(t3,T);
tb_t4=mod(t4,T);
%% ����ģ��
k=1;
for p=1:num%400��
    p
    for mmohu1=0:0
        for mmohu2=0:0
            for mmohu3=0:0
                for mmohu4=0:0
                    for s=1:length%100�ε���
                        tt1=tb_t1(p,s)+mmohu1;
                        tt2=tb_t2(p,s)+mmohu2;
                        tt3=tb_t3(p,s)+mmohu3;
                        tt4=tb_t4(p,s)+mmohu4;

                        track(1,:)=Soltrack(pos1(s,:),pos2(s,:),pos3(s,:),tt1,tt2,tt3,z,c(s));
                        track(2,:)=Soltrack(pos1(s,:),pos3(s,:),pos4(s,:),tt1,tt3,tt4,z,c(s));
                        track(3,:)=Soltrack(pos1(s,:),pos4(s,:),pos3(s,:),tt1,tt4,tt3,z,c(s));
                        track(4,:)=Soltrack(pos2(s,:),pos3(s,:),pos1(s,:),tt2,tt3,tt4,z,c(s));
                        track(5,:)=Soltrack(pos2(s,:),pos4(s,:),pos3(s,:),tt2,tt4,tt3,z,c(s));
                        track(6,:)=Soltrack(pos3(s,:),pos4(s,:),pos1(s,:),tt3,tt4,tt1,z,c(s));

                        sum_x=0;sum_y=0;sum_x1=0;sum_y1=0;
                        for i=1:6
                            lisan(i,1)=track(i,1);
                            lisan(i,2)=track(i,2);
                            lisan(i,3)=sqrt((track(i,1)-x(p))^2+(track(i,2)-y(p))^2);
                        end
                        paixu=sortrows(lisan,3);
                        for i=1:4
                            sum_x=sum_x+paixu(i,1);
                            sum_y=sum_y+paixu(i,2);
                        end
                        xx(s)=sum_x/4;
                        yy(s)=sum_y/4;
                    end 
                    res(k,1)=mean(xx);
                    res(k,2)=mean(yy);
                    k=k+1;
                end
            end
        end
    end
end

%% ��ͼ
figure%Ŀ����ʵ�켣
plot(x,y,'b*','MarkerSize',5);hold on
plot(res(:,1),res(:,2),'r.');hold on
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro',x4,y4,'ro');hold on
axis equal
axis([0 1000 0 1000]);
xlabel('x��/m','FontSize',14);
ylabel('y��/m','FontSize',14);
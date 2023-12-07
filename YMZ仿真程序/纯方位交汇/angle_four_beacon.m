close all;clear all;clc;
%% �������
x1=500;y1=500;z1=0;              %��i���ű����꣬i=1,2,3,4
x2=500;y2=-500;z2=0;             
x3=-500;y3=-500;z3=0;              
x4=-500;y4=500;z4=0; 
z=0;                           %Ŀ�����0

[x,y]=meshgrid((-500:5:500)); %Ŀ���ƶ���Χ
diedai=1000;
%% �������
std_p=1.5;
delta_x1=normrnd(0,std_p,1,diedai);delta_y1=normrnd(0,std_p,1,diedai);
delta_x2=normrnd(0,std_p,1,diedai);delta_y2=normrnd(0,std_p,1,diedai);
delta_x3=normrnd(0,std_p,1,diedai);delta_y3=normrnd(0,std_p,1,diedai);
delta_x4=normrnd(0,std_p,1,diedai);delta_y4=normrnd(0,std_p,1,diedai);

std_angle=1/180*pi;%��λΪpi
delta_angle1=normrnd(0,std_angle,1,diedai);
delta_angle2=normrnd(0,std_angle,1,diedai);
delta_angle3=normrnd(0,std_angle,1,diedai);
delta_angle4=normrnd(0,std_angle,1,diedai);

jiao = atan2(160,500);%��˫��Ԫ�������Ƕ� �ڽǶ�֮��ĲŲ������
%% ����λ���㶨λ
n=size(x);

for p=1:n(1)
    for q=1:n(2)
        for s=1:diedai
            pos_x1=x1+delta_x1(s);pos_y1=y1+delta_y1(s);
            pos_x2=x2+delta_x2(s);pos_y2=y2+delta_y2(s);
            pos_x3=x3+delta_x3(s);pos_y3=y3+delta_y3(s);
            pos_x4=x4+delta_x4(s);pos_y4=y4+delta_y4(s);
            
            pos1=[pos_x1 pos_y1 z1];
            pos2=[pos_x2 pos_y2 z2];
            pos3=[pos_x3 pos_y3 z3];
            pos4=[pos_x4 pos_y4 z4];%GPS����ĸ�ƽ̨λ��
                        
            alpha1=atan2(x(p,q)-pos_x1,y(p,q)-pos_y1)+delta_angle1(s);
            alpha2=atan2(x(p,q)-pos_x2,y(p,q)-pos_y2)+delta_angle2(s);
            alpha3=atan2(x(p,q)-pos_x3,y(p,q)-pos_y3)+delta_angle3(s);
            alpha4=atan2(x(p,q)-pos_x4,y(p,q)-pos_y4)+delta_angle4(s);%��õ�Ŀ�귽λ�� ��λΪpi ����Ϊ��׼ ˳ʱ��Ϊ�� ��ʱ��Ϊ��
            
            array_alpha = [alpha1 alpha2 alpha3 alpha4];%�ĸ��� �������
            array_pos   = [pos1;pos2;pos3;pos4];%�ĸ�����λ�� �������
            num_res=1;
            index =[];

            for i_alpha = 1:4
               for j_alpha = 1:4
                   if j_alpha > i_alpha
                       if abs(array_alpha(i_alpha)-array_alpha(j_alpha)) > jiao
                           res(num_res,:)=AngleCross(array_pos(i_alpha,:),array_pos(j_alpha,:),array_alpha(i_alpha),array_alpha(j_alpha));
                           index=[index;res(num_res,:)];
                           num_res = num_res + 1;
                        end
                   end
               end
            end
%             res(1,:)=AngleCross(pos1,pos2,alpha1,alpha2);
%             res(2,:)=AngleCross(pos1,pos3,alpha1,alpha3);
%             res(3,:)=AngleCross(pos1,pos4,alpha1,alpha4);
%             res(4,:)=AngleCross(pos2,pos3,alpha2,alpha3);
%             res(5,:)=AngleCross(pos2,pos4,alpha2,alpha4);
%             res(6,:)=AngleCross(pos3,pos4,alpha3,alpha4);
            
            sum_x=0;sum_y=0;
            for i=1:length(index)
                lisan(i,1)=index(i,1);
                lisan(i,2)=index(i,2);
                lisan(i,3)=sqrt((index(i,1)-x(p,q))^2+(index(i,2)-y(p,q))^2);
            end
            paixu=sortrows(lisan,3);%��������
            for i=1:3
                sum_x=sum_x+paixu(i,1);
                sum_y=sum_y+paixu(i,2);
            end
            xx=sum_x/3;
            yy=sum_y/3;
            
            delta_r(s)=sqrt((xx-x(p,q))^2+(yy-y(p,q))^2);
        end
        delta_R(p,q)=mean(delta_r);
    end
end
%% ��ͼ
figure
h=pcolor(x,y,delta_R);hold on;
plot(x1,y1,'r*',x2,y2,'r*',x3,y3,'r*',x4,y4,'r*');hold on;
set(h,'edgecolor','none','facecolor','interp');
colorbar;
% caxis([0 35]);
xlabel('x��/m','FontSize',14);
ylabel('y��/m','FontSize',14);
% title('����λ���㶨λ�������');
close all;clear all;clc;%没改完！！！！！！！！！！
%% 参数设置
x1=1000;y1=1000;z1=0;%第i个信标坐标，i=1,2,3,4
x2=1000;y2=-1000;z2=0;             
x3=-1000;y3=-1000;z3=0;              
x4=-1000;y4=1000;z4=0; 
z=0;%目标深度0

T=0.1;%同步周期设为0.1s
v=40;%目标航速40m/s
esv=1500;%声速真实值

[x,y]=meshgrid((-1000:50:1000)); %目标移动范围

%% 误差设置
std_t=0.001;
std_p=3;
std_c=1;
std_theta=3*pi/180;

length=100;
delta_t1=normrnd(0,std_t,1,length);  %随机生成一个均值为0，标准差为2 的1*1000随机数矩阵
delta_t2=normrnd(0,std_t,1,length);     
delta_t3=normrnd(0,std_t,1,length);
delta_t4=normrnd(0,std_t,1,length);

delta_x1=normrnd(0,std_p,1,length);delta_y1=normrnd(0,std_p,1,length);dz1=0;    
delta_x2=normrnd(0,std_p,1,length);delta_y2=normrnd(0,std_p,1,length);dz2=0;    
delta_x3=normrnd(0,std_p,1,length);delta_y3=normrnd(0,std_p,1,length);dz3=0;    
delta_x4=normrnd(0,std_p,1,length);delta_y4=normrnd(0,std_p,1,length);dz4=0;   

delta_c=normrnd(0,std_c,1,length);

delta_theta1=normrnd(0,std_theta,1,length);
delta_theta2=normrnd(0,std_theta,1,length);
delta_theta3=normrnd(0,std_theta,1,length);
delta_theta4=normrnd(0,std_theta,1,length);
%% 模拟声学参数测量
%浮标GPS上传位置
pos_x1=x1+delta_x1;pos_y1=y1+delta_y1;%100次迭代
pos_x2=x2+delta_x2;pos_y2=y2+delta_y2;
pos_x3=x3+delta_x3;pos_y3=y3+delta_y3;
pos_x4=x4+delta_x4;pos_y4=y4+delta_y4;
for i=1:length
    pos1(i,:)=[pos_x1(i) pos_y1(i) z1];%100次迭代×[x y z]坐标
    pos2(i,:)=[pos_x2(i) pos_y2(i) z2];
    pos3(i,:)=[pos_x3(i) pos_y3(i) z3];
    pos4(i,:)=[pos_x4(i) pos_y4(i) z4];
end
%加入误差后的声速
c=esv+delta_c;%100次迭代
%% 参考位置标示法
n=size(x);
for p=1:n(1)
    p
    for q=1:n(2)
        for s=1:length%100次迭代
            %实际距离
            dis1=sqrt((x(p,q)-x1)^2+(y(p,q)-y1)^2);%400个目标航迹
            dis2=sqrt((x(p,q)-x2)^2+(y(p,q)-y2)^2);
            dis3=sqrt((x(p,q)-x3)^2+(y(p,q)-y3)^2);
            dis4=sqrt((x(p,q)-x4)^2+(y(p,q)-y4)^2);
            %实际时延
            time1=dis1/esv;
            time2=dis2/esv;
            time3=dis3/esv;
            time4=dis4/esv;
            %加入误差后的时延
            t1=time1+delta_t1(s);%400个点×100次迭代
            t2=time2+delta_t2(s);
            t3=time3+delta_t3(s);
            t4=time4+delta_t4(s);  
            %同步周期相对时延
            tb_t1=mod(t1,T);%400个点×100次迭代
            tb_t2=mod(t2,T);
            tb_t3=mod(t3,T);
            tb_t4=mod(t4,T);

            theta1=atan2(y(p,q)-pos_y1(s),x(p,q)-pos_x1(s))+delta_theta1(s);
            theta2=atan2(y(p,q)-pos_y2(s),x(p,q)-pos_x2(s))+delta_theta2(s);
            theta3=atan2(y(p,q)-pos_y3(s),x(p,q)-pos_x3(s))+delta_theta3(s);
            theta4=atan2(y(p,q)-pos_y4(s),x(p,q)-pos_x4(s))+delta_theta4(s);

            xangle(1)=(pos_y2(s)-pos_y1(s)-pos_x2(s)*tan(theta2)+pos_x1(s)*tan(theta1))/(tan(theta1)-tan(theta2));
            yangle(1)=((pos_x2(s)-pos_x1(s))*tan(theta1)*tan(theta2)+pos_y1(s)*tan(theta2)-pos_y2(s)*tan(theta1))/(tan(theta2)-tan(theta1));

            xangle(2)=(pos_y3(s)-pos_y2(s)-pos_x3(s)*tan(theta3)+pos_x2(s)*tan(theta2))/(tan(theta2)-tan(theta3));
            yangle(2)=((pos_x3(s)-pos_x2(s))*tan(theta2)*tan(theta3)+pos_y2(s)*tan(theta3)-pos_y3(s)*tan(theta2))/(tan(theta3)-tan(theta2));

            xangle(3)=(pos_y4(s)-pos_y3(s)-pos_x4(s)*tan(theta3)+pos_x3(s)*tan(theta3))/(tan(theta3)-tan(theta4));
            yangle(3)=((pos_x4(s)-pos_x3(s))*tan(theta3)*tan(theta4)+pos_y3(s)*tan(theta4)-pos_y4(s)*tan(theta3))/(tan(theta4)-tan(theta3));

            xangle(4)=(pos_y3(s)-pos_y1(s)-pos_x3(s)*tan(theta3)+pos_x1(s)*tan(theta1))/(tan(theta1)-tan(theta3));
            yangle(4)=((pos_x3(s)-pos_x1(s))*tan(theta1)*tan(theta3)+pos_y1(s)*tan(theta3)-pos_y3(s)*tan(theta1))/(tan(theta3)-tan(theta1));

            xangle(5)=(pos_y4(s)-pos_y2(s)-pos_x4(s)*tan(theta4)+pos_x2(s)*tan(theta2))/(tan(theta2)-tan(theta4));
            yangle(5)=((pos_x4(s)-pos_x2(s))*tan(theta2)*tan(theta4)+pos_y2(s)*tan(theta4)-pos_y4(s)*tan(theta2))/(tan(theta4)-tan(theta2));

            xangle(6)=(pos_y4(s)-pos_y1(s)-pos_x4(s)*tan(theta4)+pos_x1(s)*tan(theta1))/(tan(theta1)-tan(theta4));
            yangle(6)=((pos_x4(s)-pos_x1(s))*tan(theta1)*tan(theta4)+pos_y1(s)*tan(theta4)-pos_y4(s)*tan(theta1))/(tan(theta4)-tan(theta1));

            sum_x=0;sum_y=0;
            for i=1:6
                lisan(i,1)=xangle(i);
                lisan(i,2)=yangle(i);
                lisan(i,3)=sqrt((xangle(i)-x(p,q))^2+(yangle(i)-y(p,q))^2);
            end
            paixu=sortrows(lisan,3);
            for i=1:4
                sum_x=sum_x+paixu(i,1);
                sum_y=sum_y+paixu(i,2);
            end
            x_ref=sum_x/4;
            y_ref=sum_y/4;

            tao_rel(1)=tb_t1;
            tao_rel(2)=tb_t2;
            tao_rel(3)=tb_t3;
            tao_rel(4)=tb_t4;

            t_ref(1)=sqrt((pos_x1(s)-x_ref)^2+(pos_y1(s)-y_ref)^2)/c(s);
            t_ref(2)=sqrt((pos_x2(s)-x_ref)^2+(pos_y2(s)-y_ref)^2)/c(s);
            t_ref(3)=sqrt((pos_x3(s)-x_ref)^2+(pos_y3(s)-y_ref)^2)/c(s);
            t_ref(4)=sqrt((pos_x4(s)-x_ref)^2+(pos_y4(s)-y_ref)^2)/c(s);

            for i=1:4
                tao_ref(i)=mod(t_ref(i),T);
                n_ref(i)=(t_ref(i)-tao_ref(i))/T;
            end
            for i=1:4
                if abs(tao_ref(i)-tao_rel(i))>=0&&abs(tao_ref(i)-tao_rel(i))<(T/2)
                    n_rel(i)=n_ref(i);
                else if tao_rel(i)>0&&tao_rel(i)<(tao_ref(i)-(T/2))&&(tao_ref(i)-(T/2))<T
                        n_rel(i)=n_ref(i)+1;
                    else if tao_ref(i)+(T/2)>0&&tao_ref(i)+(T/2)<tao_rel(i)&&tao_rel(i)<T
                            n_rel(i)=n_ref(i)-1;
                        end
                    end
                end
            end
            tt1=n_rel(1)*T+tao_rel(1);
            tt2=n_rel(2)*T+tao_rel(2);
            tt3=n_rel(3)*T+tao_rel(3);
            tt4=n_rel(4)*T+tao_rel(4);

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
                lisan(i,3)=sqrt((track(i,1)-x(p,q))^2+(track(i,2)-y(p,q))^2);
            end
            paixu=sortrows(lisan,3);
            for i=1:4
                sum_x=sum_x+paixu(i,1);
                sum_y=sum_y+paixu(i,2);
            end
            xx=sum_x/4;
            yy=sum_y/4;

            delta_r(s)=sqrt((xx-x(p,q))^2+(yy-y(p,q))^2);
        end
        delta_R(p,q)=mean(delta_r);
    end
end

%% 画图
figure
h=pcolor(x,y,delta_R);hold on;
plot(0,0,'r^',x1,y1,'ro',x2,y2,'ro',x3,y3,'ro',x4,y4,'ro');hold on;
set(h,'edgecolor','none','facecolor','interp');
colorbar;
caxis([0,100])
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);

close all;clear all;clc;
%% 参数设置
x1=1000;y1=1000;z1=0;%第i个信标坐标，i=1,2,3,4
x2=1000;y2=-1000;z2=0;             
x3=-1000;y3=-1000;z3=0;              
x4=-1000;y4=1000;z4=0; 
z=0;%目标深度0

T=0.1;%同步周期设为0.1s
v=40;%目标航速40m/s
esv=1500;%声速真实值

%参考位置
x_ref=-800;
y_ref=-500;
%% 目标轨迹预设
x_begin=-800;
y_begin=-500;%目标起始点（-800，-500）

theta=pi/6;%目标航行角度与正东呈30度

num=400;%目标有400个轨迹点
for i=1:num
    x(i)=x_begin+(i-1)*v*T*cos(theta);
    y(i)=y_begin+(i-1)*v*T*sin(theta);
end
%% 误差设置
std_t=0.001;
std_p=3;
std_c=1;

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
%实际距离
dis1=sqrt((x-x1).^2+(y-y1).^2);%400个目标航迹
dis2=sqrt((x-x2).^2+(y-y2).^2);
dis3=sqrt((x-x3).^2+(y-y3).^2);
dis4=sqrt((x-x4).^2+(y-y4).^2);
%实际时延
time1=dis1./esv;
time2=dis2./esv;
time3=dis3./esv;
time4=dis4./esv;
%加入误差后的时延
for i=1:num
    t1(i,:)=time1(i)+delta_t1;%400个点×100次迭代
    t2(i,:)=time2(i)+delta_t2;
    t3(i,:)=time3(i)+delta_t3;
    t4(i,:)=time4(i)+delta_t4;  
end
%同步周期相对时延
tb_t1=mod(t1,T);%400个点×100次迭代
tb_t2=mod(t2,T);
tb_t3=mod(t3,T);
tb_t4=mod(t4,T);
%% 参考位置标示法
k=1;
for p=1:num%400点
    p
    for s=1:length%100次迭代
        tao_rel(1)=tb_t1(p,s);
        tao_rel(2)=tb_t2(p,s);
        tao_rel(3)=tb_t3(p,s);
        tao_rel(4)=tb_t4(p,s);

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
    res_cankao(k,1)=mean(xx);
    res_cankao(k,2)=mean(yy);
    x_ref=res_cankao(k,1);
    y_ref=res_cankao(k,2);
    k=k+1;
end

%% 画图
figure
plot(x,y,'b*','MarkerSize',5);hold on%目标真实轨迹
plot(res_cankao(:,1),res_cankao(:,2),'r.');hold on%抗模糊轨迹
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro',x4,y4,'ro');hold on
axis equal
axis([-1000 1000 -1000 1000]);
xlabel('x轴/m','FontSize',14);
ylabel('y轴/m','FontSize',14);
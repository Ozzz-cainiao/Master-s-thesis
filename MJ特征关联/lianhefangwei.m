%%%%%Ŀ����
clc
clear all;
close all;
buoyNum=3;
d=2;
x0=11;
% zhengquelvhuizong=zeros(length(d),5);
% for ddi=1:length(d)
% targetx=[x0-2*d(ddi),x0-d(ddi),x0,x0+d(ddi),x0+2*d(ddi)];
% targetx=[x0-2*d(ddi),x0-d(ddi),x0,x0+d(ddi),x0+2*d(ddi)];targetx=[x0,x0+d,x0-d,x0+2*d,x0-2*d]; 
deltanum=1;%1��ʾ��1���������
delta=1:2:30;
zhengquelvhuizong=zeros(length(delta),5);
for ddelta=1:length(delta)
for targetNumcount=2
% targetx=[x0-2*d,x0-d,x0,x0+d,x0+2*d]; 
targetx=[x0,x0+d,x0-d,x0+2*d,x0-2*d]; 
% targetx=[x0,x0+d(ddi),x0-d(ddi),x0+2*d(ddi),x0-2*d(ddi)]; 
targety=[10,10,10,10,10];
targetNum=targetNumcount;
zhenx=[0,10,20];
zheny=[0,0,0];
w=pi/2;


thetai=zeros(1,targetNum);
thetai_du=zeros(1,targetNum);
thetaj=zeros(1,targetNum);
thetaj_du=zeros(1,targetNum);
thetak=zeros(1,targetNum);
thetak_du=zeros(1,targetNum);

% function [theta_rad, theta_du]=target_fangwei(arrx,arry,xs,ys)
%%����1,�н�thetai��%%����2���н�thetaj��%%����3���н�thetak
for i=1:targetNum
        [thetai(i),thetai_du(i)]=target_fangwei(zhenx(1),zheny(1),targetx(i),targety(i));              
end
for i=1:targetNum
        [thetaj(i),thetaj_du(i)]=target_fangwei(zhenx(2),zheny(2),targetx(i),targety(i));              
end
for i=1:targetNum
        [thetak(i),thetak_du(i)]=target_fangwei(zhenx(3),zheny(3),targetx(i),targety(i));              
end

%%%%%%����Ŀ�������нǣ��ʣ������ʱ���Ƕ�������ֵ+1.5�㣿����

 delta_theta=( delta( ddelta))*pi/180;%%��׼����1.5�������1.5*1.5
%  delta_theta=(2.5)*pi/180;%%��׼����1.5�������1.5*1.5
% delta_theta=(sqrt(1.5))*pi/180;
shiyanNum=1000;
sumcount=zeros(targetNum,1);
sumx=zeros(targetNum,1);
sumy=zeros(targetNum,1);
errorsum=zeros(targetNum,1);
erroravg=zeros(targetNum,1);
count_zhengque=zeros(targetNum,1);
count_nn=zeros(targetNum,1);
count_xiguanliandayuR=zeros(targetNum,1);
while(shiyanNum)
randtheta=normrnd(0,delta_theta);
% randtheta=0.0070;
randthetadu=randtheta*180/pi;
theta=zeros(3,targetNum);
theta_du=zeros(3,targetNum);
for buoyNum=1:3
for i=1:targetNum
        [theta(buoyNum,i),theta_du(buoyNum,i)]=target_fangwei(zhenx(buoyNum),zheny(buoyNum),targetx(i),targety(i));  
        theta(buoyNum,i)=theta(buoyNum,i)+randtheta;
        theta_du(buoyNum,i)=theta_du(buoyNum,i)+1.5;
end
end



%%%%%%�������i�����j,k����
Rijk=zeros(1,2);
for buoyNum=2:3
    Rijk(buoyNum-1)=sqrt((zhenx(buoyNum)-zhenx(1))^2+(zheny(buoyNum)-zheny(1))^2);
end
%%%%%%������ά����DLK��i,j,n��n��ʾ��n��Ŀ��
for n=1:targetNum
    D1lk(:,:,n)=zeros(targetNum,targetNum);
   d1lk(:,:,n)=1./zeros(targetNum);%%�ų�һЩD1lk>G��D1lk,infΪ��
   G1lk(:,:,n)=zeros(targetNum,targetNum);
    az(:,:,n)=zeros(targetNum*targetNum,1);%%a�����������
    bz(:,:,n)=zeros(targetNum*targetNum,1);%%����ǰԪ��λ��
    rz(:,:,n)=zeros(targetNum*targetNum,1);%%����ǰԪ������
    cz(:,:,n)=zeros(targetNum*targetNum,1);%%����ǰԪ������
    nmin(1,1,n)=1;%%���ʼ�����������Сֵ
    lamudaxiguanlian(:,:,n)=1./zeros(targetNum);
end


    n=1;
    if(targetNum>=n)
for i=1:targetNum%%ÿһ���ǵ�pi
    for j=1:targetNum%%ÿһ���ǵ�Qj
        [D1lk(i,j,n), G1lk(i,j,n)]=caclDG(Rijk(1),theta(1,1),w,theta(2,i),Rijk(2),theta(3,j),w,delta_theta);
        if(D1lk(i,j,n)<G1lk(i,j,n))
            d1lk(i,j,n)=D1lk(i,j,n);
        end
    end
end
    end






%����i��2�������ߣ�����D2lk%%%%%%caclDG(r1,����1�����Ŀ��н�,theta_ij,theta_j,r2,theta_k,theta_ik,delta_theta)
n=2;
if(targetNum>=n)
for i=1:targetNum%%ÿһ���ǵ�pi
    for j=1:targetNum%%ÿһ���ǵ�Qj
        [D1lk(i,j,n), G1lk(i,j,n)]=caclDG(Rijk(1),theta(1,2),w,theta(2,i),Rijk(2),theta(3,j),w,delta_theta);
        if(D1lk(i,j,n)<G1lk(i,j,n))
            d1lk(i,j,n)=D1lk(i,j,n);
        end
    end
end
end


 
%����i��3�������ߣ�����D2lk%%%%%%caclDG(r1,����1�����Ŀ��н�,theta_ij,theta_j,r2,theta_k,theta_ik,delta_theta)
n=3;
if(targetNum>=n)
for i=1:targetNum%%ÿһ���ǵ�pi
    for j=1:targetNum%%ÿһ���ǵ�Qj
        [D1lk(i,j,n), G1lk(i,j,n)]=caclDG(Rijk(1),theta(1,3),w,theta(2,i),Rijk(2),theta(3,j),w,delta_theta);
        if(D1lk(i,j,n)<G1lk(i,j,n))
            d1lk(i,j,n)=D1lk(i,j,n);
        end
    end
end
end

 

%����i��4�������ߣ�����D2lk%%%%%%caclDG(r1,����1�����Ŀ��н�,theta_ij,theta_j,r2,theta_k,theta_ik,delta_theta)
n=4;
if(targetNum>=n)
for i=1:targetNum%%ÿһ���ǵ�pi
    for j=1:targetNum%%ÿһ���ǵ�Qj
        [D1lk(i,j,n), G1lk(i,j,n)]=caclDG(Rijk(1),theta(1,4),w,theta(2,i),Rijk(2),theta(3,j),w,delta_theta);
        if(D1lk(i,j,n)<G1lk(i,j,n))
            d1lk(i,j,n)=D1lk(i,j,n);
        end
    end
end
end


%����i��5�������ߣ�����D2lk%%%%%%caclDG(r1,����1�����Ŀ��н�,theta_ij,theta_j,r2,theta_k,theta_ik,delta_theta)
n=5;
if(targetNum>=n)
for i=1:targetNum%%ÿһ���ǵ�pi
    for j=1:targetNum%%ÿһ���ǵ�Qj
        [D1lk(i,j,n), G1lk(i,j,n)]=caclDG(Rijk(1),theta(1,5),w,theta(2,i),Rijk(2),theta(3,j),w,delta_theta);
        if(D1lk(i,j,n)<G1lk(i,j,n))
            d1lk(i,j,n)=D1lk(i,j,n);
        end
    end
end
end

 
shiyanNum=shiyanNum-1;%%ÿ���1�Σ������������1
for cacln=1:targetNum
    [sj,thetajERChengWeizhi,thetakERChengWeizhi,sk]=caclthetaERChengsame(targetNum,buoyNum,d1lk,cacln);%sj��ʾ�ֹ�����Ԫ�ظ���
    if(sj>=1)
       thetajERCheng=zeros(1,sj);%%
       thetakERCheng=zeros(1,sk);%%

      for tempa=1:sj   
    thetajERCheng(1,tempa)=theta(2,thetajERChengWeizhi(tempa));    
      end
   for tempa=1:sk
     thetakERCheng(1,tempa)=theta(3,thetakERChengWeizhi(tempa));
   end 
%    flag_zhengqueA=1;%%��ȷ���������־λ1
flagjk=0%�ж�PjQk��j==k==cacln
sumAi=0;
% flagtargetpos=0;
flagtargetpos(targetNum,1)=0;
        for i=1:sj
           
    [xs,ys]=gujitargetxy_4(thetajERCheng(i), thetakERCheng(i),zhenx(2),zheny(2),zhenx(3),zheny(3));
    xsall(1,i,cacln)=xs;
    ysall(1,i,cacln)=ys;
    lamuda(cacln,i)=cacllamuda(zhenx,zheny,xs,ys,theta(1,cacln),thetajERCheng(i),thetakERCheng(i),delta_theta,thetai,thetaj,thetak,thetajERChengWeizhi,thetakERChengWeizhi,i);
    menxianr=sqrt((0.5*targetx(1,cacln))^2+(0.5*targety(1,cacln))^2);
    currentr=sqrt((targetx(1,cacln)-xs)^2+(targety(1,cacln)-ys)^2);
%      count_xiguanliandayuR(cacln,1)= count_xiguanliandayuR(cacln,1)+1;
                if(lamuda(cacln,i)<10&&currentr<menxianr)
                    if(thetajERChengWeizhi(i)==cacln&&thetakERChengWeizhi(i)==cacln)
%                         count_nn(cacln,1)= count_nn(cacln,1)+1;
%                         flagjk=1;
                         flagtargetpos(cacln,1)=i;
                    end

lamudaxiguanlian(thetajERChengWeizhi(i),thetakERChengWeizhi(i),cacln)=lamuda(cacln,i);
                end
                
        end
    end
end
%%����lamuda��Сֵ
minpos=1./zeros(targetNum,2);
flgminpos=zeros(targetNum,1);
for tn=1:targetNum
   minvalue=min(min(lamudaxiguanlian(:,:,tn)));
   if(minvalue==Inf)
       continue;
   end
   [row,column]=find(minvalue==lamudaxiguanlian(:,:,tn));
   minpos(tn,1)=row(1);
   minpos(tn,2)=column(1);
   if(row(1)==column(1)&&column(1)==tn)
   flagminpos(tn,1)=1;
   if(flagtargetpos(tn,1)>0)
   count_nn(tn,1)= count_nn(tn,1)+1;
   testtest=flagtargetpos(tn,1);
   test1=xsall(1,flagtargetpos(tn,1),tn);
                         sumx(tn,1)=xsall(1,flagtargetpos(tn,1),tn)+ sumx(tn,1);
                sumy(tn,1)=ysall(1,flagtargetpos(tn,1),tn)+ sumy(tn,1);
                sumcount(tn,1)=sumcount(tn,1)+1;
                errorsum(tn,1)=sqrt((xsall(1,flagtargetpos(tn,1),tn)-xs)^2+(ysall(1,flagtargetpos(tn,1),tn)-ys)^2)+errorsum(tn,1);
   end
   end
end

end
  tarcal=zeros(targetNum,2);
 zhengquelv=zeros(targetNum,2);
 
 for i=1:targetNum
    tarcal(i,1)=sumx(i,1)/sumcount(i,1);
    tarcal(i,2)=sumy(i,1)/sumcount(i,1);
    erroravg(i,1)= errorsum(i,1)/sumcount(i,1);
%     zhengquelv(i,1)=count_zhengque(i,1)/count_xiguanliandayuR(i,1); 
zhengquelv(i,1)=count_nn(i,1)/1000;
 end
%  zhengquelvever=sum(zhengquelvhuizong(:,1))/targetNum;
for j=1:targetNum
 zhengquelvhuizong(deltanum,j)=zhengquelv(j,1);%j��ʾ��Ŀ��
end
 zhengquelvever(deltanum,targetNum)=sum(zhengquelvhuizong(deltanum,:))/targetNum; 
%  zhengquelvdelta(targetNum,delta)=zhengquelvever;
 
  D1lk=[];
   d1lk=[];%%�ų�һЩD1lk>G��D1lk,infΪ��
   G1lk=[];
    az=[];%%a�����������
    bz=[];%%����ǰԪ��λ��
    rz=[];%%����ǰԪ������
    cz=[];%%����ǰԪ������
     nmin=[];%%���ʼ�����������Сֵ
    lamudaxiguanlian=[];
    
    
 end
 deltanum=deltanum+1;
end
% end
figure
% for i=2:5
%   h1=plot(delta,zhengquelvever(:,2)','r*')
%   hold on
 h2= plot(delta,zhengquelvever(:,2)','b*')
  hold on
%   h3=plot(delta,zhengquelvever(:,4)','m*')
%   hold on
%   h4=plot(delta,zhengquelvever(:,5)','g*')
%   hold on
%   hold on
% end
legend(h2(1),'3��Ŀ��')
 ylim([0,1.2])
 xlim([1,29])
 xlabel('��λ�ǲ�������/��')
ylabel('Ŀ����ȷ������')
% axis equal
  caclerro=zeros(targetNum,1);
for i=1:targetNum
    caclerro(i,1)=sqrt((tarcal(i,1)-targetx(i))^2+(tarcal(i,2)-targety(i))^2);
end
%%%����x��y

% % [xs1,xs2,ys1,ys2]=caclxy(zhenx(1),zheny(1),theta(1,cacln),zhenx(2),zheny(2),zhenx(3),zheny(3),thetajERCheng,thetakERCheng);
%%%%test
% % thetajERCheng=theta(2,2);
% % thetakERCheng=theta(3,2);
% % [xs1,xs2,ys1,ys2]=caclxy(zhenx(1),zheny(1),theta(1,2),zhenx(2),zheny(2),zhenx(3),zheny(3),thetajERCheng,thetakERCheng);
%%%����ԭ�㷨������ٵ㣬Ŀ��λ��
%%����Ŀ��λ��function [xs1,xs2,ys1,ys2]=caclxy(x1,y1,theta1,x2,y2,x3,y3,theta2,theta3)
% Xijorignal=zeros(targetNum,targetNum); 
% Yijorignal=zeros(targetNum,targetNum);
% Xikorignal=zeros(targetNum,targetNum); 
% Yikorignal=zeros(targetNum,targetNum);
% for i=1:targetNum
%     for j=1:targetNum
%     
%         [Xijorignal(i,j),Xikorignal(i,j),Yijorignal(i,j),Yikorignal(i,j)]=caclxy(zhenx(1),zheny(1),theta(1,i),zhenx(2),zheny(2),zhenx(3),zheny(3),theta(2,j),theta(3,j));
%     
%     end
% end
% Xijorignal_plot=zeros(targetNum*targetNum,1);
% Yijorignal_plot=zeros(targetNum*targetNum,1);
% Xikorignal_plot=zeros(targetNum*targetNum,1);  
% Yikorignal_plot=zeros(targetNum*targetNum,1);
% for i=1:targetNum
%     for j=1:targetNum
%         if(Xijorignal(i,j)==NaN||Yijorignal(i,j)==NaN||Yijorignal(i,j)<0)
%             continue;
%         end
%         if(Xijorignal(i,j)<100&&Yijorignal(i,j)<100)
%         Xijorignal_plot((i-1)*targetNum+j,1)=Xijorignal(i,j);
%         Yijorignal_plot((i-1)*targetNum+j,1)=Yijorignal(i,j);
%         end
%         
%     end
% end
% for i=1:targetNum
%     for j=1:targetNum
%          if(Xikorignal(i,j)==NaN||Yikorignal(i,j)==NaN||Yikorignal(i,j)<0)
%             continue;
%         end
% if(Xikorignal(i,j)<100&&Yikorignal(i,j)<100)
%         Xikorignal_plot((i-1)*targetNum+j,1)=Xikorignal(i,j);
%         Yikorignal_plot((i-1)*targetNum+j,1)=Yikorignal(i,j);
% end
%     end
% end

% figure
% 
% % plot(Xikorignal,Yikorignal,'*')
% % % plot(Xijorignal_plot,Yijorignal_plot,'*')
% % hold on
% % plot(targetx,targety,'or')
% % hold on
% % zhenx=[0,10,20];
% % zheny=[0,0,0];
% % plot(zhenx,zheny,'k*');
% for i=1:targetNum
%     for j=1:targetNum
%          if(Xikorignal(i,j)<60&&Yikorignal(i,j)<60)
%         %��1����
%         k=(Yikorignal(i,j)-zheny(1))/(Xikorignal(i,j)-zhenx(1));
% %         k=k*(x-zhenx(1))+zheny(1);
%         x=[zhenx(1),Xikorignal(i,j)];
%         y=[zheny(1),Yikorignal(i,j)];
%         line=k*(x-zhenx(1))+zheny(1);
%         temp=max(Xikorignal(Xikorignal<60));
%         xy=min(Xikorignal(Xikorignal<60)):max(temp,zhenx(3))+2;
%         line1=k*xy+zheny(1);
%         plot(xy,line1,'r')
%         ylim([0,max(Yikorignal(Xikorignal<60))]);
%         hold on
%         %��3����
%        
% k=(Yikorignal(i,j)-zheny(3))/(Xikorignal(i,j)-zhenx(3));
% %         k=k*(x-zhenx(1))+zheny(1);
%         x=[zhenx(3),Xikorignal(i,j)];
%         y=[zheny(3),Yikorignal(i,j)];
%         line=k*(x-zhenx(3))+zheny(3);
%         temp=max(Xikorignal(Xikorignal<60));
%         xy=min(Xikorignal(Xikorignal<60)):max(temp,zhenx(3))+2;
%         line1=k*(xy-zhenx(3))+zheny(3);
%         plot(xy,line1,'y')
%         ylim([0,max(Yikorignal(Yikorignal<60))]);
%         hold on
%         %��2����
%         k=(Yijorignal(i,j)-zheny(2))/(Xijorignal(i,j)-zhenx(2));
% %         k=k*(x-zhenx(1))+zheny(1);
%         x=[zhenx(2),Xijorignal(i,j)];
%         y=[zheny(2),Yijorignal(i,j)];
%         line=k*(x-zhenx(2))+zheny(2);
%         temp=max(Xijorignal(Xijorignal<60));
%         xy=min(Xikorignal(Xikorignal<60)):max(temp,zhenx(3))+2;
%         line1=k*(xy-zhenx(2))+zheny(2);
%         plot(xy,line1,'g')
%         ylim([0,max(Yijorignal(Yijorignal<60))]);
%         hold on
%         end
%     end
% end
% % figure
% h1=plot(Xikorignal,Yikorignal,'m*')
% hold on 
% h2=plot(Xijorignal,Yijorignal,'c*')
% hold on
% % ploty=[targety,targety,targety,targety,targety];
% 
% h3=plot(targetx,targety,'ob')
% hold on
% 
% h4=plot(zhenx,zheny,'ok')
% legend([h1(1),h2(1),h3(1),h4(1)],'��1����3�����߽���','��1����2�����߽���','Ŀ����ʵλ��','������λ��')
% xlabel('x/km')
% ylabel('y/km')
% % legend(h2,'��1����2�����߽���')
% figure
% % ploty=[targety,targety,targety,targety,targety];
% 
% h5=plot(targetx,targety,'ob')
% hold on
% h6=plot(zhenx,zheny,'ok')
% legend([h5(1),h6(1)],'Ŀ����ʵλ��','������λ��')
% ylim([0,30])
% xlabel('x/km')
% ylabel('y/km')
% figure
% h7=plot(targetx,targety,'ob')
% hold on
% h8=plot(zhenx,zheny,'ok')
% hold on
% h9=plot(tarcal(:,1),tarcal(:,2),'dr')
% legend([h7(1),h8(1),h9(1)],'Ŀ����ʵλ��','������λ��','�����㷨��λ��')
% ylim([0,30])
% xlabel('x/km')
% ylabel('y/km')


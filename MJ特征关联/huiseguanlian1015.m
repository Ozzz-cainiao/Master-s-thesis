%%%%%%%%%%����Ӧ��Ȩ��ɫ����1015���汾1%%%%%%%%%%
%%%%M��ʾ��ͧ��Ŀ����������N��ʾĿ������Rͬһʱ�̵����ں�����������
clc
clear all;
close all;
M=3;
N=3;
R=M*N;
canshu=6;
%�ز�
f=[2004.45;3123.23;4312.12];
dianping=[45;23;12];
signal=[4;1;2];
width=[323.87;123.34;202.23];
target=[4;3;2];
tiaozhi=[2;1;2];
X=[f,dianping,signal,width,target,tiaozhi];
Xrec=zeros(R,canshu);
%  xavg=zeros(1,R);
% for i=1:R

% mod(1,3)
for i=1:R
    for j=1:canshu
        if(mod(i,N)~=0)%������,i%N
        Xrec(i,j)=X(mod(i,N),j)+normrnd(0,1);
        else
         Xrec(i,j)=X(N,j)+normrnd(0,1);   
        end
    end
end
%%%%����1��ƽ��ȥ����
xavg=sum(Xrec)/R;%��ʽ3.1
sjsum=zeros(1,canshu);
for j=1:canshu
    for i=1:R
        sjsum(1,j)=sjsum(1,j)+(Xrec(i,j)-xavg(1,j))^2;
    end
end
sj=sqrt(sjsum/R);
Xi=zeros(R,canshu);
for i=1:R
    for j=1:canshu
        Xi(i,j)=(Xrec(i,j)-xavg(1,j))/sj(1,j);
    end
end
Rab=zeros(R,canshu);
%%%%%%�����������xij-min(i,j)������max(xij)-min(xij)
% Xi=zeros(R,canshu);
% [max_lie,index]=max(Xrec,[],1);%��ÿ�����ֵ
% [min_lie,index2]=min(Xrec,[],1);
% for j=1:canshu
%     for i=1:R
%         Xi(i,j)=( Xrec(i,j)-min_lie(1,j))/(max_lie(1,j)-min_lie(1,j));
%     end
% end
        
for i=1:R
Rab=cacldetla(Xi,R,canshu,i);
end



    
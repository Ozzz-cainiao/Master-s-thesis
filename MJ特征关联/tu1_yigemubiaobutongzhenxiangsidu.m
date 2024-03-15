%%%%%%%%%%����Ӧ��Ȩ��ɫ����1015���汾1%%%%%%%%%%
%%%%M��ʾ��ͧ��Ŀ����������N��ʾĿ������Rͬһʱ�̵����ں����������� ��ͼ1��ͬһ��Ŀ�꣬��ͬ��۲��������ƶȣ���1,2�յ�Ŀ��1�����ƶ�
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

Xsum=sum(X);%�������
beta=zeros(1,canshu);
for j=1:canshu
	beta(1,j)=Xsum(1,j)/Xsum(1,1);%����ÿһ�У�ÿ����������ֵ�����������ʱ�˱���
end
%
T=1200;%ʵ���յ�1200�������ݣ��������ƶ�
y_xiangsidu=zeros(1,T);
buoy1=1;%��1�յ�1��Ŀ��
buoy2=4;%��2�յ�1��Ŀ��
for t=1:T
	for i=1:R
		for j=1:canshu
			if(mod(i,N)~=0)%������,i%N
				Xrec(i,j)=X(mod(i,N),j)+normrnd(0,1)*beta(1,j);
			else
				Xrec(i,j)=X(N,j)+normrnd(0,1)*beta(1,j);
			end
		end
	end
	%% ƽ��ȥ����
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

	%%
	Rab=zeros(R,R);%����
	Rabrec=zeros(1,R);%ÿ���յ�

	for i=1:R
		Rabrec=cacldetla_plie(Xi,R,canshu,i);
		Rab(i,:)=Rabrec;
	end

	for j=1:R
		for i=1:R
			Rab(i,j)=Rab(j,i);
		end
	end
	y_xiangsidu(1,t)=Rab(buoy1,buoy2);
end
figure
t=1:T;
plot(t,y_xiangsidu)
% title('ͬһĿ�겻ͬ�۲��������ƶ�');
xlabel('����')
ylabel('���ƶ�')
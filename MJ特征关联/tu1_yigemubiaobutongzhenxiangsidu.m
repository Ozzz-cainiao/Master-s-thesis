%%%%%%%%%%自适应熵权灰色关联1015，版本1%%%%%%%%%%
%%%%M表示舰艇数目（阵数），N表示目标数，R同一时刻到达融合中心数据数 绘图1：同一个目标，不同阵观测样本相似度：阵1,2收到目标1，相似度
clc
clear all;
close all;
M=3;
N=3;
R=M*N;
canshu=6;
%载波
f=[2004.45;3123.23;4312.12];
dianping=[45;23;12];
signal=[4;1;2];
width=[323.87;123.34;202.23];
target=[4;3;2];
tiaozhi=[2;1;2];
X=[f,dianping,signal,width,target,tiaozhi];
Xrec=zeros(R,canshu);

Xsum=sum(X);%按列求和
beta=zeros(1,canshu);
for j=1:canshu
	beta(1,j)=Xsum(1,j)/Xsum(1,1);%计算每一列（每个参数）数值倍数，加误差时乘倍数
end
%
T=1200;%实验收到1200周期数据，计算相似度
y_xiangsidu=zeros(1,T);
buoy1=1;%阵1收到1号目标
buoy2=4;%阵2收到1号目标
for t=1:T
	for i=1:R
		for j=1:canshu
			if(mod(i,N)~=0)%不等于,i%N
				Xrec(i,j)=X(mod(i,N),j)+normrnd(0,1)*beta(1,j);
			else
				Xrec(i,j)=X(N,j)+normrnd(0,1)*beta(1,j);
			end
		end
	end
	%% 平移去量纲
	xavg=sum(Xrec)/R;%公式3.1
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
	Rab=zeros(R,R);%汇总
	Rabrec=zeros(1,R);%每次收到

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
% title('同一目标不同观测样本相似度');
xlabel('周期')
ylabel('相似度')
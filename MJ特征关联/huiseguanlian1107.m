%%%%%%%%%%自适应熵权灰色关联1015，版本1%%%%%%%%%%
%%%%M表示舰艇数目（阵数），N表示目标数，R同一时刻到达融合中心数据数
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
for i=1:R
    for j=1:canshu
        if(mod(i,N)~=0)%不等于,i%N
        Xrec(i,j)=X(mod(i,N),j)+normrnd(0,1)*beta(1,j);
        else
         Xrec(i,j)=X(N,j)+normrnd(0,1)*beta(1,j);   
        end
    end
end
%%%%方法1：平移去量纲
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
Rab=zeros(R,R);%汇总
Rabrec=zeros(R,R);%每次收到

for i=1:R
Rabrec=cacldetla_plie(Xi,R,canshu,i);
Rab(i,:)=Rabrec;
end


for j=1:R
    for i=1:R
        Rab(i,j)=Rab(j,i);
    end
end

Mui=zeros(2*R,2*R);
Ci=zeros(R,2*R);
Cinum=zeros(1,2*R);
for i=1:R
    Cinum(1,i)=Cinum(1,i)+1;
end
for j=1:R
    Ci(1,j)=j;
end
for i=1:R
    for j=1:R
        if(i~=j)
        Mui(i,j)=Rab(i,j);
        end
    end
end
for i=1:2*R
    Ci_3wei(:,:,i)=zeros(R,2*R);
end
Vc=zeros(1,R);%存储分类准则函数
 ronghe_lei=zeros(1,R);       

numnot0=1;%找一个矩阵里每一列最后一个不为0数在第几个，如果有一列numnot0=9说明融合一个目标
round=1;%融合几次
while(numnot0<R)
    [index_x,index_y]=find(Mui==max(max(Mui)));%查找相似度最大位置，表示index_x，index_y融合
    for i=1:R+round
        if(i==index_x(1))%index_x那一行置0，这个类不存在
            Mui(i,:)=0;
%             numNew=Cinum(1,R+round);%R+round融合后新类在第几列，那一列Ci表示谁和谁融合，Cinum,numNew表示新类目前包含几个类融合行数
            numOld=Cinum(1,index_x(1));%被融合类内有几个原始类,行数
            for k=1:numOld
                Cinum(1,R+round)=Cinum(1,R+round)+1;%新类包含元素个数增加
                 numNew=Cinum(1,R+round);
                Ci(numNew,R+round)=Ci(k,index_x(1));
                
            end
            Ci(:,index_x(1))=0;%被融合类那一列内所含原始类0
            Cinum(1,index_x(1))=0;
       
        elseif(i==index_y(1))%index_y，这个类不存在
            Mui(i,:)=0;
            numOld=Cinum(1,index_y(1));%被融合类内有几个原始类,行数
            for k=1:numOld
                Cinum(1,R+round)=Cinum(1,R+round)+1;%新类包含元素个数增加
                 numNew=Cinum(1,R+round);
                Ci(numNew,R+round)=Ci(k,index_y(1));
                
            end
             Ci(:,index_y(1))=0;
            Cinum(1,index_y(1))=0;

        else
            Mui(i,R+round)=max(Mui(i,index_x(1)),Mui(i,index_y(1)));
            Mui(R+round,i)=Mui(i,R+round);
           Mui(i,index_x(1))=0;%没变那些类与原类相似度=0，重新计算新类
           Mui(i,index_y(1))=0;
        end
    end
    %%补充新列-》新行，前面那个判断R《index_y那改程序
    %查找一个矩阵每列第一个不为0数字位置，当某一列不为0数字在第9行，说明都融合
[Sw,lei_num]=Sw3(Xi, Ci,Cinum,R,round,Rab);
Sb=Sb2(Xi, Ci,Cinum,R,round,Rab);
Vc(1,round)=Sb+Sw;
Ci_3wei(:,:,round)=Ci(:,:);
ronghe_lei(1,round)=lei_num;

    numnot0=max(Cinum);
    round=round+1;
end
[result,result_round]=max(Vc);%分类准则函数最大时是第几轮融合
result_lei=ronghe_lei(1,result_round);
    
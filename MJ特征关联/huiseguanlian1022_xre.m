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
X=[f,dianping,signal,width,target,tiaozhi]; % 三组特征
Xrec=zeros(R,canshu); % 观测特征
%  xavg=zeros(1,R);
% for i=1:R

% mod(1,3)
%
Xsum=sum(X);
beta=zeros(1,canshu);
for j=1:canshu
   beta(1,j)=Xsum(1,j)/Xsum(1,1);
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
%%%%%%方法二：极差法xij-min(i,j)，除，max(xij)-min(xij)
% Xi=zeros(R,canshu);
% [max_lie,index]=max(Xrec,[],1);%找每列最大值
% [min_lie,index2]=min(Xrec,[],1);
% for j=1:canshu
%     for i=1:R
%         Xi(i,j)=( Xrec(i,j)-min_lie(1,j))/(max_lie(1,j)-min_lie(1,j));
%     end
% end
%%cacldetla_plie(Xi,R,canshu,i);     Xi输入，R一共几行数据，第i行与其他行做关联
for i=1:R
Rabrec=cacldetla_plie(Xi,R,canshu,i);
Rab(i,:)=Rabrec(i,:);
end


for j=1:R
    for i=1:R
        Rab(i,j)=Rab(j,i);
    end
end
% Mui=zeros(R,2*R);
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
% Ci=zeros(R,1);
%  for i=1:R
%      if(i==4||i==7)
%          Ci(4,1)= Ci(4,1)+sum(Xi(i,:).^2,2);
%      else
%          Ci(i,1)=(sum(Xi(i,:).^2,2));
%      end
%  end
%  Ci(4,1)=1/ Ci(4,1);
% s=Ci(1,1)+Ci(2,1)+Ci(3,1)+Ci(5,1)+Ci(6,1)+Ci(8,1)+Ci(9,1);
% s=s+Ci(4,1)*(2*Rab(4,7)+2);
% s=s/8;
% cishu=9;
% firstnot0=1;
% % num0=1;
% % kk=1;
% % while (num0<R)
% %     [x,y]=find(Mui==max(max(Mui)));
% %    for i=1:R
% %        if(i==x(1))
% %            Mui(i,:)=0;
% %            if(x(1)<y(1))
% %            Ci(x(1),numCi(x(1)))=y(1);
% %            numCi(x(1))=numCi(x(1))+1;
% %            Ci(y(1),:)=0;
% %            numCi(y(1))=0;
% %        else
% %             Ci(y(1),numCi(y(1)))=x(1);
% %            numCi(y(1))=numCi(y(1))+1;
% %            Ci(x(1),:)=0;
% %            numCi(x(1))=0;
% %        end
% %        elseif(i==y(1))
% %            Mui(i,:)=0;
% %        else
% %        Mui(i,R+kk)=max(Mui(i,x(1)),Mui(i,y(1)));
% %        Mui(i,x(1))=0;
% %        Mui(i,y(1))=0;
% %        
% %        end
% %        [firstnot0,kkk]=find(Ci(1,:)~=0);
% %        num0=firstnot0(1);
% %        kk=kk+1;
% %    end
% % end
% % numnot0=1;%找一个矩阵里每一列最后一个不为0数在第几个，如果有一列numnot0=9说明融合一个目标
% % round=1;%融合几次
% % while(numnot0)
% %     [index_x,index_y]=find(Mui==max(max(Mui)));%查找相似度最大位置，表示index_x，index_y融合
% %     for i=1:R
% %         if(i==index_x(1))%index_x那一行置0，这个类不存在
% %             Mui(i,:)=0;
% %             numNew=Cinum(1,R+round);%R+round融合后新类在第几列，那一列Ci表示谁和谁融合，Cinum,numNew表示新类目前包含几个类融合行数
% %             numOld=Cinum(1,index_x(1));%被融合类内有几个原始类,行数
% %             for k=1:numOld
% %                 Cinum(1,R+round)=Cinum(1,R+round)+1;%新类包含元素个数增加
% %                  numNew=Cinum(1,R+round);
% %                 Ci(numNew,R+round)=Ci(k,index_x(1));
% %                 
% %             end
% %             Ci(:,index_x(1))=0;%被融合类那一列内所含原始类0
% %             Cinum(1,index_x(1))=0;
% %        
% %         elseif(i==index_y(1)||i==R&&R<index_y(1))%index_y，这个类不存在
% %             if (i<=R)
% %             Mui(i,:)=0;
% %             end
% %             numOld=Cinum(1,index_y(1));%被融合类内有几个原始类,行数
% %             for k=1:numOld
% %                 Cinum(1,R+round)=Cinum(1,R+round)+1;%新类包含元素个数增加
% %                  numNew=Cinum(1,R+round);
% %                 Ci(numNew,R+round)=Ci(k,index_y(1));
% %                 
% %             end
% %              Ci(:,index_y(1))=0;
% %             Cinum(1,index_y(1))=0;
% %         elseif(i==R&&R<index_y(1))%%第10列表示新类，没有第10行
% %             numOld=Cinum(1,index_y(1));%被融合类内有几个原始类,行数
% %             for k=1:numOld
% %                 Cinum(1,R+round)=Cinum(1,R+round)+1;%新类包含元素个数增加
% %                  numNew=Cinum(1,R+round);
% %                 Ci(numNew,R+round)=Ci(k,index_y(1));
% %                 
% %             end
% %              Ci(:,index_y(1))=0;
% %             Cinum(1,index_y(1))=0;
% %         else
% %             Mui(i,R+round)=max(Mui(i,index_x(1)),Mui(i,index_y(1)));
% %            Mui(i,index_x(1))=0;%没变那些类与原类相似度=0，重新计算新类
% %            Mui(i,index_y(1))=0;
% %         end
% %     end
% %     查找一个矩阵每列第一个不为0数字位置，当某一列不为0数字在第9行，说明都融合
% %     numnot0=max(Cinum);
% %     round=round+1;
% % end
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
%             if (i<=R)
            Mui(i,:)=0;
%             end
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
%     Sw1(Xi,Ci,Cinum,R,round,Rab);
%     Sb(Xi,Ci,Cinum,R,round,Rab);
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
    
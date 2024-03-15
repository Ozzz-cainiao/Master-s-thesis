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

Xsum=sum(X);%�������
beta=zeros(1,canshu);
for j=1:canshu
   beta(1,j)=Xsum(1,j)/Xsum(1,1);%����ÿһ�У�ÿ����������ֵ�����������ʱ�˱���
end
%
for i=1:R
    for j=1:canshu
        if(mod(i,N)~=0)%������,i%N
        Xrec(i,j)=X(mod(i,N),j)+normrnd(0,1)*beta(1,j);
        else
         Xrec(i,j)=X(N,j)+normrnd(0,1)*beta(1,j);   
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
Rab=zeros(R,R);%����
Rabrec=zeros(R,R);%ÿ���յ�

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
Vc=zeros(1,R);%�洢����׼����
 ronghe_lei=zeros(1,R);       

numnot0=1;%��һ��������ÿһ�����һ����Ϊ0���ڵڼ����������һ��numnot0=9˵���ں�һ��Ŀ��
round=1;%�ںϼ���
while(numnot0<R)
    [index_x,index_y]=find(Mui==max(max(Mui)));%�������ƶ����λ�ã���ʾindex_x��index_y�ں�
    for i=1:R+round
        if(i==index_x(1))%index_x��һ����0������಻����
            Mui(i,:)=0;
%             numNew=Cinum(1,R+round);%R+round�ںϺ������ڵڼ��У���һ��Ci��ʾ˭��˭�ںϣ�Cinum,numNew��ʾ����Ŀǰ�����������ں�����
            numOld=Cinum(1,index_x(1));%���ں������м���ԭʼ��,����
            for k=1:numOld
                Cinum(1,R+round)=Cinum(1,R+round)+1;%�������Ԫ�ظ�������
                 numNew=Cinum(1,R+round);
                Ci(numNew,R+round)=Ci(k,index_x(1));
                
            end
            Ci(:,index_x(1))=0;%���ں�����һ��������ԭʼ��0
            Cinum(1,index_x(1))=0;
       
        elseif(i==index_y(1))%index_y������಻����
            Mui(i,:)=0;
            numOld=Cinum(1,index_y(1));%���ں������м���ԭʼ��,����
            for k=1:numOld
                Cinum(1,R+round)=Cinum(1,R+round)+1;%�������Ԫ�ظ�������
                 numNew=Cinum(1,R+round);
                Ci(numNew,R+round)=Ci(k,index_y(1));
                
            end
             Ci(:,index_y(1))=0;
            Cinum(1,index_y(1))=0;

        else
            Mui(i,R+round)=max(Mui(i,index_x(1)),Mui(i,index_y(1)));
            Mui(R+round,i)=Mui(i,R+round);
           Mui(i,index_x(1))=0;%û����Щ����ԭ�����ƶ�=0�����¼�������
           Mui(i,index_y(1))=0;
        end
    end
    %%��������-�����У�ǰ���Ǹ��ж�R��index_y�Ǹĳ���
    %����һ������ÿ�е�һ����Ϊ0����λ�ã���ĳһ�в�Ϊ0�����ڵ�9�У�˵�����ں�
[Sw,lei_num]=Sw3(Xi, Ci,Cinum,R,round,Rab);
Sb=Sb2(Xi, Ci,Cinum,R,round,Rab);
Vc(1,round)=Sb+Sw;
Ci_3wei(:,:,round)=Ci(:,:);
ronghe_lei(1,round)=lei_num;

    numnot0=max(Cinum);
    round=round+1;
end
[result,result_round]=max(Vc);%����׼�������ʱ�ǵڼ����ں�
result_lei=ronghe_lei(1,result_round);
    
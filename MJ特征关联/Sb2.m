%%%%%%���������뺯��Sb
function [Sb]=Sb2(Xi, Ci,Cinum,R,round,Rab)
abs_Xi=abs(Xi);
sum_Xi_orig=sum(abs_Xi,2);%ԭʼXi�������ֵ���
sum_Xi=sum_Xi_orig;
sum_Xi_1=1./sum_Xi;
% sum_Xi_2=sum_Xi_1.^2;
lei_num=0;%����C��ּ���
for i=1:R+round
    combinnum=Cinum(1,i);%��i���ں���(��i��)����ԭʼ�����
    if(combinnum>0)
        lei_num=lei_num+1;
    end
end
Sb=0;
% sumcr=0;
for i=1:R+round
    sumcr=0;
    if(Cinum(1,i)>0)
    for j=1:R+round
        if(i~=j)
        sumr=0;
        xnum=Cinum(1,i);%r(x,y)��x�м�����Ci�Ǹ�����
        ynum=Cinum(1,j);
        if(ynum>0)
        for w=1:Cinum(1,i)
                for p=1:Cinum(1,j)
                    testw=Ci(w,i);%ԭʼ����Ѽ����j��i��
                    testp= Ci(p,j);
                    sumr=sumr+Rab(testw,testp);
                end
        end
        sumcr=sumcr+(1/(xnum*ynum))*sumr;
        end
        end
    end
    Sb=Sb+sumcr;
    end
end
Sb=Sb/2;
Sb=(1/(lei_num*(lei_num-1)))*Sb;%��=1ʱ,Sb=1/0
end
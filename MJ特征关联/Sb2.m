%%%%%%计算类间分离函数Sb
function [Sb]=Sb2(Xi, Ci,Cinum,R,round,Rab)
abs_Xi=abs(Xi);
sum_Xi_orig=sum(abs_Xi,2);%原始Xi矩阵绝对值求和
sum_Xi=sum_Xi_orig;
sum_Xi_1=1./sum_Xi;
% sum_Xi_2=sum_Xi_1.^2;
lei_num=0;%集合C里分几类
for i=1:R+round
    combinnum=Cinum(1,i);%第i个融合类(第i列)，含原始类个数
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
        xnum=Cinum(1,i);%r(x,y)里x有几个，Ci那个集合
        ynum=Cinum(1,j);
        if(ynum>0)
        for w=1:Cinum(1,i)
                for p=1:Cinum(1,j)
                    testw=Ci(w,i);%原始类编号鸭，第j，i列
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
Sb=(1/(lei_num*(lei_num-1)))*Sb;%类=1时,Sb=1/0
end
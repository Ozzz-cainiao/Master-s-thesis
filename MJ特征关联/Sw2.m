%%%%%%计算类内聚合度Sw和类间分离函数
function Sw2(Xi, Ci,Cinum,R,round,Rab)
abs_Xi=abs(Xi);
sum_Xi_orig=sum(abs_Xi,2);%原始Xi矩阵绝对值求和
sum_Xi=sum_Xi_orig;
sum_Xi_1=1./sum_Xi;
sum_Xi_2=sum_Xi_1.^2;
lei_num=0;%集合C里分几类
for i=1:R+round
    combinnum=Cinum(1,i);%第i个融合类(第i列)，含原始类个数
    if(combinnum>0)
        lei_num=lei_num+1;
    end
%     for n=1:combinnum
%         yuanlei=Ci(n,i);%第n行，第i列，原始类
%         sum_Xi(i,1)=sum_Xi(i,1)+sum_Xi_orig(yuanlei,1);        
%     end
%     sum_Xi(i,1)=sum_Xi_orig(i,1);
%     sum_Xi_1(i,1)=1/sum_Xi(i,1);
end
% Sw=0;
% 
%     for i=1:R+round
%         sumr=0;
%         if(Cinum(1,i)==1)%只有1个原始类
%             Sw= sum_Xi_1(i,1)*sum_Xi_1(i,1)*1+Sw;%调试看一下是否每次都Sw初始0
%         elseif(Cinum(1,i)>=2)
%             for j=1:Cinum(1,i)
%                 for p=j+1:Cinum(1,i)
%                     testj=Ci(j,i);%原始类编号鸭，第j，i列
%                     testp= Ci(p,i);
%                     sumr=sumr+Rab(testj,testp);
%                 end
%             end
%             CiRxy=sum_Xi_1(i,1)*(Cinum(1,i)*1+2*sumr);
%             Sw=Sw+(CiRxy);
%         end
%     end
%     Sw=(1/lei_num)*Sw;
%     Sw=0;
%     for i=R+1:R+round
%         if(Cinum(1,i)>=2)
%           for j=1:Cinum(1,i)
%                 for p=j+1:Cinum(1,i)
%                     testj=Ci(j,i);%原始类编号鸭，第j，i列
%                     testp= Ci(p,i);
%                     Sw=Sw+2*sum_Xi_1(testj,1)*sum_Xi_1(testp,1)*Rab(testj,testp);
%                 end
%             end  
%         end
%     end
% for i=1:R
%     combinnum=Cinum(1,i);%第i个融合类(第i列)，含原始类个数
%     if(combinnum>0)
%         Sw=Sw+sum_Xi_2(i,1);
%     end
% end 
% Sw=Sw+sum(sum_Xi_2);
%     Sw=(1/lei_num)*Sw;
    Sw=0;
   
    for i=1:R+round
         sumr=0;
        if(Cinum(1,i)==1&&i<=R)%只有前9类里原始类数1
            Sw=Sw+Rab(i,i);
        elseif(Cinum(1,i)>=2)
          for j=1:Cinum(1,i)
                for p=j+1:Cinum(1,i)
                    testj=Ci(j,i);%原始类编号鸭，第j，i列
                    testp= Ci(p,i);
                  sumr=sumr+2*Rab(testj,testp);
                end
          end 
          combinnum=Cinum(1,i);
          sumr=sumr+combinnum*1;
            Sw=Sw+(1/(combinnum*combinnum))*sumr;
        end
    end
    Sw=(1/lei_num)*Sw;
end
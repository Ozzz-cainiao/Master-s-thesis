%%%%%%%%%%%计算正确率
	
Ci_2wei=Ci_3wei(:,:,result_round);
row=size(Ci_2wei,1);
column=R+result_round;%最后一个有数列在第几列
flag=1;%1表示正常
right=0;%不正确
for i=1:R%判断前9行是否第一个数0,0表示这些类都去融合
   if(Ci_2wei(1,i)~=0)
       flag=0;
       break;
   end
end
% if(flag~=0)%前R列满足都融合才继续判断，融合对不对
%    for i=R+1:column
%        b=Ci_2wei(:,i);%R*1列数
%        [minnum,ind]=min(b(find(b~=0)));%找1列不等于0最小值
%        for j=1:R
%            temp=(b(j,1)-minnum)/N;
%            if((b(j,1)~=minnum)&&(rem(temp,1)~=0))
%                flag=0;
%                break;
%            end
%        end
%    end
% end
if(flag~=0)
for i=R+1:column
       b=Ci_2wei(:,i);%R*1列数
       [minnum,ind]=min(b(find(b~=0)));%找1列不等于0最小值
        for j=1:R
           temp=(b(j,1)-minnum)/N;
                if((b(j,1)~=minnum)&(rem(temp,1)~=0)&(b(j,1)~=0))
                 flag=0;
                 break;
                 end
       end
end
end
if(flag==1)
    right=1;
end
end
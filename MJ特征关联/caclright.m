%%%%%%%%%%%������ȷ��
	
Ci_2wei=Ci_3wei(:,:,result_round);
row=size(Ci_2wei,1);
column=R+result_round;%���һ���������ڵڼ���
flag=1;%1��ʾ����
right=0;%����ȷ
for i=1:R%�ж�ǰ9���Ƿ��һ����0,0��ʾ��Щ�඼ȥ�ں�
   if(Ci_2wei(1,i)~=0)
       flag=0;
       break;
   end
end
% if(flag~=0)%ǰR�����㶼�ںϲż����жϣ��ں϶Բ���
%    for i=R+1:column
%        b=Ci_2wei(:,i);%R*1����
%        [minnum,ind]=min(b(find(b~=0)));%��1�в�����0��Сֵ
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
       b=Ci_2wei(:,i);%R*1����
       [minnum,ind]=min(b(find(b~=0)));%��1�в�����0��Сֵ
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
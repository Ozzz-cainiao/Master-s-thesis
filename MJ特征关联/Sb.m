%%%%%%���������뺯��Sb
function Sb(Xi, Ci, Cinum, R, round, Rab)
abs_Xi = abs(Xi);
sum_Xi_orig = sum(abs_Xi, 2); %ԭʼXi�������ֵ���
sum_Xi = sum_Xi_orig;
sum_Xi_1 = 1 ./ sum_Xi;
% sum_Xi_2=sum_Xi_1.^2;
lei_num = 0; %����C��ּ���
for i = 1:R + round
    combinnum = Cinum(1, i); %��i���ں���(��i��)����ԭʼ�����
    if (combinnum > 0)
        lei_num = lei_num + 1;
    end
end
Sb = 0;
for i = 1:R + round
    for j = i + 1:R + round
        %         xnum=Cinum(1,i);%r(x,y)��x�м�����Ci�Ǹ�����
        %         ynum=Cinum(1,j);
        for w = 1:Cinum(1, i)
            for p = 1:Cinum(1, j)
                testw = Ci(w, i); %ԭʼ����Ѽ����j��i��
                testp = Ci(p, j);
                Sb = Sb + sum_Xi_1(testw, 1) * sum_Xi_1(testp) * Rab(testw, testp);
            end
        end
    end
end
Sb = 2 * Sb;
Sb = (1 / (lei_num * (lei_num - 1))) * Sb;
end
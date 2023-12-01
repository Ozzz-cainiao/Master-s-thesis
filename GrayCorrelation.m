%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\GrayCorrelation.m
% 版本: v1.0
% 作者: 网络
% 联系方式: https://blog.csdn.net/weixin_51545953/article/details/111029419
% 日期: 2023-11-22
% 描述: 灰色关联算法，从网上找的代码
% 输入:  
% 输出:  
%**************************************************************************

%% 应用一：分析产业对GDP的影响程度
clear;clc;
load data.mat;
r = size(data,1);
c = size(data,2);
%第一步，对变量进行预处理，消除量纲的影响（大家在使用时需要注意自己的数据量纲是否相同）
%avg = repmat(mean(data),r,1);
%data = data./avg;
%定义母序列和子序列
Y = data(:,1); %母序列
X = data(:,2:c); %子序列
Y2 = repmat(Y,1,c-1); %把母序列向右复制到c-1列
absXi_Y = abs(X-Y2)
a = min(min(absXi_Y)) %全局最小值
b = max(max(absXi_Y)) %全局最大值
ro = 0.5; %分辨系数取0.5
gamma = (a+ro*b)./(absXi_Y+ro*b) %计算子序列中各个指标与母序列的关联系数
disp("子序列中各个指标的灰色关联度分别为：");
ans = mean(gamma)



%% 应用二：灰色关联分析评价河流情况
clear;clc;
load X.mat;
%获取行数列数
r = size(X,1);
c = size(X,2);
%首先，把我们的原始指标矩阵正向化
%第二列中间型--->极大型
middle = input("请输入最佳的中间值：");
M = max(abs(X(:,2)-middle));
for i=1:r
      X(i,2) = 1-abs(X(i,2)-middle)/M;
end
%第三列极小型--->极大型
max_value = max(X(:,3)); 
X(:,3) = abs(X(:,3)-max_value);
%第四列区间型--->极大型
a = input("请输入区间的下界：");
b = input("请输入区间的下界：");
M = max(a-min(X(:,4)),max(X(:,4))-b);
for i=1:r
       if (X(i,4)<a)
            X(i,4) = 1-(a-X(i,4))/M;
       elseif (X(i,4)<=b&&X(i,4)>=a)
           X(i,4) = 1;
       else
           X(i,4) = 1-(X(i,4)-b)/M;
       end
end
disp("正向化后的矩阵为：");
disp(X);
%把正向化后的矩阵进行预处理，消除量纲的影响
avg = repmat(mean(X),r,1);
new_X = X./avg;
%将预处理后的矩阵每一行的最大值取出，当成母序列(虚构的)
Y = max(new_X,[],2);
%计算各个指标和母序列的灰色关联度
%先把new_X矩阵所有元素都减去母序列中同行的元素，并取绝对值
Y2 = repmat(Y,1,c);
new_X = abs(new_X-Y2);
a = min(min(new_X)); %全矩阵最小值
b = max(max(new_X)); %全矩阵最大值
ro = 0.5;
new_X = (a+ro*b)./(new_X+ro*b);
disp("各个指标对于母序列的灰色关联度为：");
gamma = mean(new_X)
%计算各个指标的权重
disp("各个指标的权重为：");
weight = gamma./(sum(gamma,2))
%-------------------------------------------------------------------------------------------------------
%继续TOPSIS的步骤：对正向化后的矩阵X进行标准化（原矩阵除以每一列元素平方之和的开方）
temp1 = X.*X;               %先让每每一个元素平方
temp2 = sum(temp1);         %再对每一列求和
temp3 = temp2.^0.5;         %再把结果开方
temp4 = repmat(temp3,r,1);  %把开方后的结果按行复制r行
disp("******标准化后的矩阵为：");
Z = X./temp4               %原矩阵除以每一列元素平方之和的开方
Z_max = max(Z)           %获得Z每一列中最大的元素
Z_min = min(Z)           %获得Z每一列中最小的元素
D_max = sum(weight.*(Z-repmat(Z_max,r,1)).^2,2).^0.5
D_min = sum(weight.*(Z-repmat(Z_min,r,1)).^2,2).^0.5
disp("该矩阵得分为：")
S = D_min./(D_max+D_min)
disp("矩阵归一化后得分为：");
S = S./(repmat(sum(S),r,1))

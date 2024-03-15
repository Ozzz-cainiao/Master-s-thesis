%% 计算灰色关联度
% cacldetla_plie
% input：
%	Xi:去量纲化后的目标特征序列矩阵
%	R：报文个数
%	canshu： 特征个数
%	k：当前计算的哪条报文
% output:
%	Rabrec：灰色关联度
% author: 
% date : 2023-5-21
% version: v1
%% 
function [Rabrec]=cacldetla_plie(Xi,R,canshu,k)
%计算任意两个特征序列和之间的第i个特征差异值
for i = 1 : R
	% delta(i,j)第i个向量和当前指定报文特征的差异值
	delta(i,:) = abs(Xi(i,:) - Xi(k,:));
	a = min(delta(i,:));
	b = max(delta(i,:));
	if (b == 0) 
		b = 0.00000001;
	end
	ro = 0.5;
	yipusilo(i,:) = (a + ro * b)./(delta(i,:) + ro * b);

end
% 计算Xi(j)和Xk(j)的灰色关联系数
yipuxilo(find(isnan(yipusilo) == 1)) = 0;
yipusilo;% 9*6矩阵


%% 计算灰色关联度 
% 定义灰色关联度为所有关联系数的平均值
Rabrec = (sum(yipusilo,2)/canshu)';% 是个1*9的矩阵












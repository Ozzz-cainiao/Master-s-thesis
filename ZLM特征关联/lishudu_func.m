%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM特征关联\lishudu_func.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-03-13
% 描述: 这是自己想的使用隶属度计算的一个demo，提供3组特征向量，计算他们的隶属度
% 输入:  
% 输出:  
%**************************************************************************


clc
% f1和f2的特征值
f1 = [299, 399, 480];
f2 = [320, 440];
% f3的特征值
f3 = [300, 398, 481];

% 定义隶属度函数
num_lines_membership = @(x) trapmf(x, [2, 3, 4, 5]); % 线谱数量隶属度函数为以元素个数3为中心的梯形分布
freq_membership = @(x, center) normpdf(x, center, 3); % 频率隶属度函数为以特征值为中心的高斯分布，方差为3
nearest_value = @(x, array) array(find(abs(array - x) == min(abs(array - x)), 1)); % 定义一个函数找到最接近的特征值

% 计算f3对f1和f2的隶属度
f3_num_lines_membership = num_lines_membership(length(f3));
% 计算f3对f1最近特征值的隶属度
f3_freq_membership = sum(arrayfun(@(x) freq_membership(x, nearest_value(x, f1)), f3)); % 求平均值

% 计算f2对f1的隶属度
f2_num_lines_membership = num_lines_membership(length(f2));
f2_freq_membership = sum(arrayfun(@(x) freq_membership(x, nearest_value(x, f1)), f2)); % 求平均值

% 得到结果再加权

% 显示结果
disp("f3对线谱数量的隶属度：" + num2str(f3_num_lines_membership));
disp("f3对线谱频率的隶属度：" + num2str(f3_freq_membership));
disp("f2对线谱数量的隶属度：" + num2str(f2_num_lines_membership));
disp("f2对线谱频率的隶属度：" + num2str(f2_freq_membership));



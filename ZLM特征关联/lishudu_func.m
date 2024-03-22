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
clear all
% % f1和f2的特征值
% f1 = [299, 399, 480];
% f2 = [320, 440];
% % f3的特征值
% f3 = [300, 398, 481];
% 
% % 定义隶属度函数
% % 线谱数量隶属度函数为以元素个数3为中心的梯形分布
% num_lines_membership = @(x) trapmf(x, [2, 3, 4, 5]); 
% % 频率隶属度函数为以特征值为中心的高斯分布，方差为3
% freq_membership = @(x, center) normpdf(x, center, 3); 
% % 定义一个函数找到最接近的特征值
% nearest_value = @(x, array) array(find(abs(array - x) == min(abs(array - x)), 1)); 
% 
% % 计算f3对f1和f2的隶属度
% f3_num_lines_membership = num_lines_membership(length(f3));
% % 计算f3对f1最近特征值的隶属度
% f3_freq_membership = sum(arrayfun(@(x) freq_membership(x, nearest_value(x, f1)), f3)); % 求平均值
% 
% % 计算f2对f1的隶属度
% f2_num_lines_membership = num_lines_membership(length(f2));
% f2_freq_membership = sum(arrayfun(@(x) freq_membership(x, nearest_value(x, f1)), f2)); % 求平均值



% 得到结果再加权

% 显示结果
% disp("f3对线谱数量的隶属度：" + num2str(f3_num_lines_membership));
% disp("f3对线谱频率的隶属度：" + num2str(f3_freq_membership));
% disp("f2对线谱数量的隶属度：" + num2str(f2_num_lines_membership));
% disp("f2对线谱频率的隶属度：" + num2str(f2_freq_membership));

% 线谱数量， 线谱频率, 调制谱的基频是轴频，叶片数、谐波个数、水声目标识别
feature1 = {5, {460, 580, 650, 790, 880}, 160, 4, 3}; 
feature2 = {8, {380, 420, 460, 550, 620, 710, 790, 880}, 400, 3, 1}; % 线谱数量， 线谱频率
feature3 = {5, {320, 455, 560, 730, 890}, 420, 2, 2}; % 线谱数量， 线谱频率
feature4 = {4, {460, 580, 650, 790}, 160, 4, 3};
res = match_degree(feature1, feature3);
load("C:\Users\Lenovo\Desktop\a.mat");
load("C:\Users\Lenovo\Desktop\b.mat");
res2 = match_degree(a, b);
nn = 10;

function res = match_degree(a, b)

% 定义隶属度函数
num_lines_membership = @(x) trapmf(x, [a{1}-2, a{1}, a{1}+1, a{1}+2]);
freq_membership = @(x, center) normpdf(x, center, 2); % 频率隶属度函数为以特征值为中心的高斯分布，方差为3
nearest_value = @(x, array) array{cellfun(@(y) norm(y - x), array) == min(cellfun(@(y) norm(y - x), array))};

% 计算b对a隶属度
b_num_lines_membership = num_lines_membership(b{1});
% b_freq_membership = sum(cellfun(@(x) freq_membership(x, nearest_value(x, a{2})), b{2})); % 求和
b_freq_membership = 0;
for i = 1:numel(b{2})
    b_freq_membership = b_freq_membership + freq_membership(b{2}{i}, nearest_value(b{2}{i}, a{2}));
end

% 将隶属度加权
res = 0.2 * b_num_lines_membership + 0.8 * b_freq_membership;
end
% function res = nearest_value(x, array)
%     min_diff = inf;
%     nearest_val = [];
%     for i = 1:numel(array)
%         diff = norm(array{i} - x);
%         if diff < min_diff
%             min_diff = diff;
%             nearest_val = array{i};
%         end
%     end
%     res = nearest_val;
% end

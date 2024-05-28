%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\ZLM多维分配抗异常\main.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-04-30
% 描述: ZLM多维分配抗异常参量（抗虚警）
%       算法可行性分析，要输出关联结果
% 输入:
% 输出:
%**************************************************************************
clc
clear
close all
warning('off')

%%
num = 1; % 蒙特卡罗次数
res = zeros(num, 1); % 计算正确率
record = cell(num, 1);
sample = cell(num, 1);

tic;
lambda = 0.5; % 泊松分布的参数（虚警率）
num_false_alarms = poissrnd(lambda);
% 生成均匀分布的虚警信号值
if num_false_alarms > 0
    false_alarm_values = randi([-180, 180], 1, num_false_alarms);
else
    false_alarm_values = [];
end
disp(['生成的虚警信号数量为：', num2str(num_false_alarms), ...
    ' 生成的虚警信号值为：', num2str(false_alarm_values)]);

%% ==========================参数===============================
times = 5;
var2d = 1; % 单位 度
var2 = var2d * (pi / 180)^2; % 角度测量方差单位rad，角度测量误差服从零均值高斯分布
Fai = pi / 2;
% DCGT参数 M、Q、I
M = 5; % 选取最少线点数
I = 10; % 并行次优条数
% MDADA参数 area
area = -3:0.5:3;
% MDADA参数 area
detla = 0.3; % 成功关联的门限

%% ==========================布放===============================
node = [0, 0; 4, 0; 4, 4; 0, 4] .* 1e3; % 节点位置
source = [0.5, 2.05; 0.1, 2; 2, 0.5; 3.5, 2; 2.3, 2; 2, 3.5] .* 1e3;

S = size(node, 1); % 节点数
choNum = [1, 3, 4];
choSource = source(choNum, :); % 选择的三个目标
N = size(choSource, 1);
% atan2d [-180,180]
angR = atan2d((choSource(:, 2) - node(:, 2).'), (choSource(:, 1) - node(:, 1).')); % 平台测得的目标角度
fs = 50e3;
% Q = [0, 10, 15, 15, 20, 20];
Q = 20;
PD = 1; % 检测概率

%% ==========================生成测量===============================
zk = angR + sqrt(var2d) * randn(N, S); % 方位测量生成
Fk = mat2cell(num2cell(zk), N, ones(1, S)); % 分割方位测量矩阵
% Fk每个平台的测向，对Fk添加虚警
Fk1 = Fk;
while num_false_alarms > 0
    chosP = randi([1, size(node, 1)]);
    temp = false_alarm_values(num_false_alarms);
    Fk1{chosP}{end + 1} = temp;
    num_false_alarms = num_false_alarms - 1;
end
ZZ = cellfun(@(x, y) x, Fk, 'un', 0); % 正确关联测量集
[~, posZ] = arrayfun(@(x) sort(cell2mat(x{1}(:, 1))), ZZ, 'un', 0); % 排序之后的顺序
[~, posR] = cellfun(@(x) sort(x), posZ, 'un', 0); % 正确关联顺序
% posR = cat(2, posR{:});
ns = cellfun(@(x) size(x, 1), ZZ); % 每个平台测量数目
Z = cellfun(@(u, v) u(v, :), ZZ, posZ, 'un', 0); % 所有测量 Z是排序后的结果

ZZ1 = cellfun(@(x, y) x, Fk1, 'un', 0); % 带有虚警的正确关联测量集
[~, posZ1] = arrayfun(@(x) sort(cell2mat(x{1}(:, 1))), ZZ1, 'un', 0); % 排序之后的顺序
% 正确关联顺序 [a,b,c,d] 第一个角度现在在第a位，第二个角度现在在第b位
[~, posR1] = cellfun(@(x) sort(x), posZ1, 'un', 0);
% posR1 = cat(2, posR1{:});
ns1 = cellfun(@(x) size(x, 1), ZZ1); % 每个平台测量数目
Z1 = cellfun(@(u, v) u(v, :), ZZ1, posZ1, 'un', 0); % 所有测量 Z1是排序后的结果

%% 多维分配
[outMDADA] = mdada(Z1, node, area, [var2, PD, Fai]); % 1:S关联结果 S+1:S+2位置 S+3 代价

%% 找到异常参量

posX = mat2cell(outMDADA(:, 1:S)', ones(S, 1))';
% 初始化结果
different_elements = {};
different_indices = {};
% 遍历每个cell并比较元素
for i = 1:numel(posR1)
    % 将cell转换为排序后的向量
    vec1 = sort(posR1{i});
    vec2 = sort(posX{i});
    [different, idx] = setxor(vec1, vec2);
    different_elements{i} = different;
    different_indices{i} = idx;
end
false_value = [];
% 显示结果
for i = 1:numel(posR1)
    if ~isempty(different_elements{i})
        %                 fprintf('%d 元素是：%s', i, mat2str(different_elements{i}));
        false_value = cell2mat(Z1{i}(different_elements{i}));
        %                 fprintf("                      虚警的值为 %f\n", false_value);
    end
end
if ~isempty(false_alarm_values)
    res(count) = 1;
    record{count} = false_value;
    sample{count} = false_alarm_values;
end
toc;

% 画出所有平台的测向线，包括虚警和漏报
xgrid = -1e3:10:5e3;
Ylim  = [-1e3, 5e3];
y = cell(size(node, 1), 1);
% fig = figure('Units', 'centimeters', 'Position', [10, 10, 12, 12 / 4 * 3]); % 左下宽高;
fig = figure('Units', 'centimeters', 'Position', [20, 5, 10, 7.5]);
% 下方将直线修改为射线
for s = 1:size(node, 1)
    theta = cell2mat(Fk1{s}(:, 1));
    xp = node(s, 1);
    yp = node(s, 2);
    % 计算射线的终点
    end_x = xp + 8e3 * cosd(theta);
    end_y = yp + 8e3 * sind(theta);
    % 绘制射线
    hold on;
    h = arrayfun(@(i) plot([xp, end_x(i)], [yp, end_y(i)], '--', 'Color', '#808080'), 1:length(theta));
    hold off;
end
% 限制坐标范围
xlim([-500, 4500]);
ylim([-500, 4500]);
hold on
s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
s2 = scatter(choSource(:, 1), choSource(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100);
legend([h(end), s1, s2], '方位测量', '观测站', '目标', 'FontSize', 12, 'Location', 'eastoutside')
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 10)
ylabel('北向坐标/m', 'FontSize', 10)
title("各平台的测向线情况")
disp("Finish");

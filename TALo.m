%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\TALo.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-26
% 描述: 各种定位方法的集中调用，kmeans验证函数
% 输入:
% 输出:
%**************************************************************************

function [] = TALo(target_info_matrix, node)

%% 输入参数，当前测量的角度和时延 每一列是一个目标
numOfSource = size(target_info_matrix, 2);
numOfPlatForm = size(target_info_matrix, 1);
T_num = size(target_info_matrix{1, 1}, 1);
tMatrix = cell(numOfSource, 1);
aMatrix = cell(numOfSource, 1);
res = cell(numOfSource, 1);
for i = 1:numOfSource

    %% 针对不同平台
    % 提取有用信息
    tMatrix{i} = zeros(T_num, numOfPlatForm);
    aMatrix{i} = zeros(T_num, numOfPlatForm);
    for j = 1:numOfPlatForm
        % 获取当前结构体数组
        oneTarget = target_info_matrix{j, i};

        % 提取结构体中的信息
        t_delayInfo = [oneTarget.t_delay]; %
        angInfo = [oneTarget.angle];
        % 将提取到的信息存储到矩阵中的相应列
        tMatrix{i}(:, j) = t_delayInfo;
        aMatrix{i}(:, j) = angInfo;
    end
end


% %% 画图
% figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);
% hold on
% for i = 1:numOfSource
%     plot(res{i}(:, 1), res{i}(:, 2), '.');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% hold off
% legend('目标1', '目标2', 'Location', 'eastoutside')
% title("kmeans&TDOA/AOA定位结果")
% set(gca, 'Box', 'on')
% disp("结束");

%% 针对某点时刻进行解算
i = 1;
t = 200;
% 使用 isnan 函数生成逻辑矩阵，其中非NaN元素为true，NaN元素为false
nanLogicalMatrix = ~isnan(tMatrix{i});
res = cell(3, 1);
loc = cell(3, 1);
% 使用 sum 函数按行求和，得到每行非NaN元素的数量
nonNaNCountPerRow = sum(nanLogicalMatrix, 2);
if nonNaNCountPerRow(t) >= 2
    % 获取该行中非NaN元素的索引
    nonNaNIndices = find(~isnan(tMatrix{i}(t, :)));
    % 提取出对应的平台时延和角度
    currentT = tMatrix{i}(t, nonNaNIndices);
    % 对第2号测量单元加时延
    currentT(2) = currentT(2) + 1;
    currentA = aMatrix{i}(t, nonNaNIndices);
    nodeT = node(nonNaNIndices, :);
    nodeT = [nodeT, zeros(length(nonNaNIndices), 1)];

    %% 计算TDOA/AOA
    [res{1}, loc{1, :}] = TA(currentT, currentA, nodeT);

    %% 计算TDOA
    if size(node, 1) > 3
        [res{2}, loc{2, :}] = myTDOA(currentT, nodeT);
    else
        [res{2}, loc{2, :}] = TDOA(currentT, nodeT, 4);
    end

    %% 计算AOA
    [res{3}, loc{3, :}] = AOA(currentA, nodeT);
end

%% 如何将结果综合起来
Res = res{1}; % 5
Res = [Res; res{2}]; % 4
Res = [Res; res{3}]; % 6
rowsToRemove = any(isnan(Res), 2) | any(isinf(Res), 2); % 找到包含nan或inf的行号
Res = Res(~rowsToRemove, :); % 去除包含nan或inf的行

Loc = loc{1}; % 5
Loc = [Loc; loc{2}]; % 4
Loc = [Loc; loc{3}]; % 6
Loc = Loc(~rowsToRemove, :);

%% 计算聚集度
rho = 200;
gamma = 0.005;
[respie] = jujidu(Res, Loc, rho, gamma);
% disp("Finish")

%% 找到异常的参量
indicesInRes = find(ismember(Res, respie)); % 使用 find 查找子集元素在父数组中的索引
halfIndex = numel(indicesInRes) / 2;
firstHalfIndices = indicesInRes(1:halfIndex); % 找到保留的行号
locpie = Loc(firstHalfIndices, :);

% 先将这些cell拼接
LLoc = unique(horzcat(Loc{:}));
llocpie = unique(horzcat(locpie{:}));

% 使用 setdiff 找到在 Res 中但不在 respie 中的元素
nonSubsetElementsInRes = setdiff(LLoc, llocpie);
fprintf("异常参量是: %s\n", nonSubsetElementsInRes);

%% 计算支持度
zhichidu(Res, firstHalfIndices);

%% 画图
% figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);
figure
hold on
plot(Res(:, 1), Res(:, 2), 'b*')
plot(respie(:, 1), respie(:, 2), 'r*');
plot(mean(respie(:, 1)), mean(respie(:, 2)), 'bp', 'MarkerFaceColor', 'b');
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend("C2", "C1", "C1中心", "观测平台");
hold off
title("目标位置初测值（全局）")
% figure('Units', 'centimeters', 'Position', [15, 5, 20, 11.24 / 15 * 15]);

end

%% 实现TDOA/AOA算法  参考公式 王静飞论文
% 输入参数 1个目标的时延和方位矩阵
% 输出参数 当前目标的时间定位结果
function [Res] = myTA(tMatrix, aMatrix, node)
time = size(tMatrix, 1);
% 使用 isnan 函数生成逻辑矩阵，其中非NaN元素为true，NaN元素为false
nanLogicalMatrix = ~isnan(tMatrix);

% 使用 sum 函数按行求和，得到每行非NaN元素的数量
nonNaNCountPerRow = sum(nanLogicalMatrix, 2);
Res = nan(time, 2);
for t = 1:time
    if nonNaNCountPerRow(t) >= 2
        % 可以进行计算了，寻找对应平台的索引
        % 获取该行中非NaN元素的索引
        nonNaNIndices = find(~isnan(tMatrix(t, :)));
        % 提取出对应的平台时延和角度
        currentT = tMatrix(t, nonNaNIndices);
        currentA = aMatrix(t, nonNaNIndices);
        nodeT = node(nonNaNIndices, :);

        [res, loc] = TA(currentT, currentA, nodeT);
        [Res(t, 1), Res(t, 2)] = jujidu(res, loc, 100);
    end
end
end

%% 实现TDOA算法 这个程序专用 在内部集成了4平台选3的功能
function [res, loc] = myTDOA(currentT, node)
% 开始4选3
if size(node, 1) > 3
    combinationsT = nchoosek(currentT, 3);
    % 使用nchoosek函数生成所有可能的组合的索引
    combinationIndices = nchoosek(1:size(node, 1), 3);
    for index = 1:size(combinationsT, 1)
        timeDelay = combinationsT(index, :);
        nodeT = node(combinationIndices(index, :), :);
        [res(index, :), ~]= TDOA(timeDelay, nodeT, 4);
        % 自我获取组合
        loc{index, :} = arrayfun(@(x)"t"+num2str(x), combinationIndices(index, :));
    end
end
end

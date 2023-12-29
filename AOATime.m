%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\AOATime.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-25
% 描述: 真实值，纯方位，时空关联
% 输入: 所测的方位序列 angM1, node, numOfSource, t_obs, T
% 输出: 最小二乘定位结果
%**************************************************************************

function [] = AOATime(varargin)
angM1 = varargin{1};
node = varargin{2};
numOfSource = varargin{3};
t_obs = varargin{4};
T = varargin{5};


% 预分配一个与cellMatrix相同大小的矩阵来存储第一个元素
Elements = cell(numOfSource, 1);

% 先假设每个目标都是对应的顺序
% 对每个目标进行分别计算
for k = 1:numOfSource
    Elements{k} = nan(size(angM1));
    % 循环遍历每个cell
    for i = 1:size(angM1, 1)
        for j = 1:size(angM1, 2)
            % 提取当前cell中的第一个元素
            currentCell = angM1{i, j};
            % 检查cell是否为空
            if ~isempty(currentCell)
                % 检查是否有多个元素
                if length(currentCell) < k
                    Elements{k}(i, j) = NaN; % 例如，将空值用NaN表示
                else
                    % 提取第一个元素并存储到新的矩阵中
                    Elements{k}(i, j) = currentCell(k);
                end
            else
                % 处理空cell的情况（根据你的需求进行操作）
                Elements{k}(i, j) = NaN; % 例如，将空值用NaN表示
            end
        end
    end

end

res = cell(numOfSource, 1);

%% LSM + 非时空关联
for k = 1:numOfSource
    res{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    for i = 1:size(Elements{k}, 1)
        [res{k}(i, 1), res{k}(i, 2)] = LSM(Elements{k}(i, :), node);
    end
end

%% LSM + 时空关联
dmin = 5; % 阈值
maxCount = 20;
c = 1500;
numofPlatForm = size(node, 1);
Tres = cell(numOfSource, 1);
for k = 1:numOfSource
    Tres{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    % 这里从5开始
    for i = 1:size(Elements{k}, 1)
        d = 1e3;
        oldTargetX = res{k}(i, 1);
        oldTargetY = res{k}(i, 2);
        iii = 0;
        while iii == 0 || d > dmin
            iii = iii + 1;
            % 计算当前点到各平台的距离
            dis = zeros(1, numofPlatForm);
            dtau = zeros(1, numofPlatForm);
            newTime = zeros(1, numofPlatForm);
            newAlpha = zeros(1, numofPlatForm);
            for j = 1:numofPlatForm
                dis(j) = sqrt((oldTargetX - node(j, 1))^2+(oldTargetY - node(j, 2))^2);
            end
            % 找一个基准时间 距离最长的，别的时间都要往后走
            [~, maxIndex] = max(dis);

            % 重新寻找位置解算
            for j = 1:numofPlatForm
                dtau(j) = (dis(maxIndex) - dis(j)) / c; % 距离长的向前找
                if i - dtau(j) / T < 1
                    newTime(j) = 1;
                elseif (i - dtau(j) / T) / T == 0
                    newTime(j) = i - dtau(j) / T;
                else
                    newTime(j) = round(i - dtau(j) / T); % 向下取整
                end
                newAlpha(j) = Elements{k}(newTime(j), j);
            end
            [targetX, targetY] = LSM(newAlpha, node);
            % 计算两次迭代之间的距离
            d = pdist([[oldTargetX, oldTargetY]; [targetX, targetY]]);
            %             i
            %             oldTargetX
            %             oldTargetY
            %             newTime
            if d < dmin || iii > maxCount
                oldTargetX = targetX;
                oldTargetY = targetY;
                break;
            else
                oldTargetX = targetX;
                oldTargetY = targetY;
            end

        end
        % 输出结果
        [Tres{k}(i, 1), Tres{k}(i, 2)] = deal(oldTargetX, oldTargetY);
    end
end

figure
hold on
for i = 1:numOfSource
    plot(res{i}(:, 1), res{i}(:, 2), '.');
    plot(Tres{i}(:, 1), Tres{i}(:, 2), '*');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
hold off
legend('目标1LSM', '目标1LSM+时空关联', '目标2LSM', '目标2LSM+时空关联', 'Location', 'eastoutside')
title("LSM定位结果")
set(gca, 'Box', 'on')
%% 两两组合定位
res2 = cell(numOfSource, 1);
for k = 1:numOfSource
    res2{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    for i = 1:size(Elements{k}, 1)
        [res2{k}(i, 1), res2{k}(i, 2)] = AOA1(Elements{k}(i, :), node);
    end
end
% figure
% hold on
% for i = 1:numOfSource
%     plot(res2{i}(:, 1), res2{i}(:, 2), '.');
%     %     plot(Tres{i}(:, 1), Tres{i}(:,2));
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% hold off
% title("AOA定位结果")

%% 两两组合定位 + 时空关联
Tres2 = cell(numOfSource, 1);
for k = 1:numOfSource
    Tres2{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    % 这里从5开始
    for i = 1:size(Elements{k}, 1)
        d = 1e3;
        oldTargetX = res2{k}(i, 1);
        oldTargetY = res2{k}(i, 2);
        iii = 1;
        while iii == 1 || d > dmin
            iii = iii + 1;
            if iii > maxCount
                break;
            end
            % 计算当前点到各平台的距离
            dis = zeros(1, numofPlatForm);
            dtau = zeros(1, numofPlatForm);
            newTime = zeros(1, numofPlatForm);
            newAlpha = zeros(1, numofPlatForm);
            for j = 1:numofPlatForm
                dis(j) = sqrt((oldTargetX - node(j, 1))^2+(oldTargetY - node(j, 2))^2);
            end
            % 找一个基准时间 距离最长的，别的时间都要往后走
            [~, maxIndex] = max(dis);
            % 重新寻找位置解算
            for j = 1:numofPlatForm
                dtau(j) = (dis(maxIndex) - dis(j)) / c; % 距离长的向前找
                newTime(j) = floor(i - dtau(j) / T);
                if newTime(j) < 1
                    newTime(j) = 1; % 新的索引
                end
                newAlpha(j) = Elements{k}(newTime(j), j);
            end
            [targetX, targetY] = AOA1(newAlpha, node);
            % 计算两次迭代之间的距离
            d = pdist([[oldTargetX, oldTargetY]; [targetX, targetY]]);
            if d < dmin
                break;
            else
                oldTargetX = targetX;
                oldTargetY = targetY;
            end
        end
        % 输出结果
        [Tres2{k}(i, 1), Tres2{k}(i, 2)] = deal(targetX, targetY);
    end
end

% figure
% hold on
% for i = 1:numOfSource
%     %     plot(res{i}(:, 1), res{i}(:,2));
%     plot(Tres2{i}(:, 1), Tres2{i}(:, 2), '.');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% hold off
% title("AOA+时空关联定位结果")

%% 使用抗异常参量的结果 + 两两组合LSM定位

res3 = cell(numOfSource, 1);
for k = 1:numOfSource
    res3{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    for i = 1:size(Elements{k}, 1)
        [res3{k}(i, 1), res3{k}(i, 2)] = AOA2(Elements{k}(i, :), node);
    end
end

%% 两两组合定位 + 时空关联
Tres3 = cell(numOfSource, 1);
for k = 1:numOfSource
    Tres3{k} = nan(size(Elements{k}, 1), 2); % 时间*1的结果
    % 这里从5开始
    for i = 1:size(Elements{k}, 1)
        d = 1e3;
        oldTargetX = res3{k}(i, 1);
        oldTargetY = res3{k}(i, 2);
        iii = 1;
        while iii == 1 || d > dmin
            iii = iii + 1;
            if iii > maxCount
                break;
            end
            % 计算当前点到各平台的距离
            dis = zeros(1, numofPlatForm);
            dtau = zeros(1, numofPlatForm);
            newTime = zeros(1, numofPlatForm);
            newAlpha = zeros(1, numofPlatForm);
            for j = 1:numofPlatForm
                dis(j) = sqrt((oldTargetX - node(j, 1))^2+(oldTargetY - node(j, 2))^2);
            end
            % 找一个基准时间 距离最长的，别的时间都要往后走
            [~, maxIndex] = max(dis);
            % 重新寻找位置解算
            for j = 1:numofPlatForm
                dtau(j) = (dis(maxIndex) - dis(j)) / c; % 距离长的向前找
                newTime(j) = floor(i - dtau(j) / T);
                if newTime(j) < 1
                    newTime(j) = 1; % 新的索引
                end
                newAlpha(j) = Elements{k}(newTime(j), j);
            end
            [targetX, targetY] = AOA2(newAlpha, node);
            % 计算两次迭代之间的距离
            d = pdist([[oldTargetX, oldTargetY]; [targetX, targetY]]);
            if d < dmin
                break;
            else
                oldTargetX = targetX;
                oldTargetY = targetY;
            end
        end
        % 输出结果
        [Tres3{k}(i, 1), Tres3{k}(i, 2)] = deal(targetX, targetY);
    end
end
figure('Units', 'centimeters','Position', [10 5 20 11.24/15*15]);
hold on
for i = 1:numOfSource
    plot(res3{i}(:, 1), res3{i}(:, 2), '.');
    plot(Tres3{i}(:, 1), Tres3{i}(:, 2), '*');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
hold off
legend('目标1', '目标1+时空关联', '目标2', '目标2+时空关联', 'Location', 'eastoutside')
title("kmeans&AOA+时空关联定位结果")
set(gca, 'Box', 'on')
% varargout = Tres3;
end % function end

%% 两两组合解算目标点
function [EstX, EstY] = AOA1(Zt, node)
% 添加一下判角的条件
% 找到非NaN值的索引
[~, col] = find(~isnan(Zt));
% 使用索引从第二个矩阵中取出相应位置的数
node = node(col, :);
Zt = Zt(:, col);
len = size(node, 1);
k = 0;
for i = 1:len
    for j = i + 1:len
        k = k + 1;
        pos1 = node(i, :);
        pos2 = node(j, :);
        alpha1 = Zt(i);
        alpha2 = Zt(j);
        res(k, :) = AngleCross(pos1, pos2, alpha1, alpha2);
    end
end
EstX = mean(res(:, 1));
EstY = mean(res(:, 2));
end

%% 这个输入的角度是度
function res = AngleCross(pos1, pos2, angle1, angle2)
pos_x1 = pos1(1);
pos_y1 = pos1(2);
pos_x2 = pos2(1);
pos_y2 = pos2(2); %GPS测得两个平台位置
% 将角度转换为弧度
angle1 = angle1 / 180 * pi;
angle2 = angle2 / 180 * pi;
L = sqrt((pos_x1 - pos_x2)^2+(pos_y1 - pos_y2)^2); %基线长度

beta = (atan((pos_y2 - pos_y1)/(pos_x2 - pos_x1))); %基线与正东方向夹角 以pi为单位

R1 = L * abs(cos(angle2+beta)/sin(angle2-angle1));
R2 = L * abs(cos(angle1+beta)/sin(angle2-angle1)); %计算得到的平台到目标的距离

xx1 = pos_x1 + R1 * sin(angle1);
xx2 = pos_x2 + R2 * sin(angle2);
yy1 = pos_y1 + R1 * cos(angle1);
yy2 = pos_y2 + R2 * cos(angle2);

res(1) = (xx1 + xx2) / 2;
res(2) = (yy1 + yy2) / 2;
end

%% 在这个方程中加上抗异常参量
function [EstX, EstY] = AOA2(Zt, node)
len = size(node, 1);
m = 0; % 记录解的个数

for i = 1:len
    for j = i + 1:len
        pos1 = node(i, :);
        pos2 = node(j, :);
        alpha1 = Zt(i);
        alpha2 = Zt(j);

        %         % 加上判角条件 和解算直接融合到一起
        %         beta1 = atand((pos_2(2) - pos1(2))/(pos2(1) - pos1(1))); %基线与正东方向夹角
        %         beta2 = atand((pos_1(2) - pos2(2))/(pos1(1) - pos2(1))); %基线与正东方向夹角
        %         if alpha1 - beta1 < lamda && alpha2 - beta2 < lamda
        %             % 直接将该点标记为异常情况
        %
        %
        %             continue;
        %         else
        %             k = k + 1;
        %             res(k, : ) = LSM([alpha1, alpha2], [pos1;pos2]); % 计算结果
        %             loc(k, : ) = [i, j]; % 记录目标位置与所利用的参量的对应关系
        %         end
        % 不带角度预先判决
        m = m + 1;
        [res(m, 1), res(m, 2)] = LSM([alpha1, alpha2], [pos1; pos2]); % 计算结果
        loc(m, :) = [i, j]; % 记录目标位置与所利用的参量的对应关系
    end
end


end


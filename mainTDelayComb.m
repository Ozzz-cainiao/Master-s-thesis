%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTDelayComb.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-01-08
% 描述: 编写的时延数据时间关联算法 单目标
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.2; %观测周期
T_all = 10; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 1^2; % 角度制  角度误差
pd = 0.9; % 检测概率
% 虚警期望

%% 运动模型
% 这是什么模型？
% 运动方程 x = x_last + v * t + 0.5 * a * t^2;
%匀速运动矩阵
F1 = [1, 0, T, 0; ...
    0, 1, 0, T; ...
    0, 0, 1, 0; ...
    0, 0, 0, 1];
%加速度矩阵
F2 = [0.5 * T^2, 0; ...
    0, 0.5 * T^2; ...
    T, 0; ...
    0, T];

%% 低杂波双目标四平台
% 布放目标
initial_position1 = [4e2, 7e2]; % 初始位置目标1
velocity1 = [20, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);

sourceAll = source1;

% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([2e3, 0]);
platform3 = Platform([2e3, 2e3]);
platform4 = Platform([0, 2e3]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 2e3, 0; 2e3, 2e3; 0, 2e3];
% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
tDelay = cell(1, numOfPlatForm); % 存放测得的时延
tDelayT = cell(1, numOfPlatForm); % 存放测得的时延
% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, T_num+20); % 现在只用来存放方位信息
    tDelay{j} = nan(numOfSource, T_num+20);
    tDelayT{j} = nan(numOfSource, T_num+20);
    for k = 1:numOfSource % 遍历声源
        sourceAll = source1;
        % 创建结构体数组
        numStructs = T_num + 20;
        myStructArray = repmat(struct('angle', nan, 'type', nan, 'fre', nan, 't_delay', nan), numStructs, 1);
        % 填充结构体数组
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = floor(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                % 模糊周期 N = floor(t_delay/dt);
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay-floor(t_delay/dt)*dt);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                tDelay{j}(k, t_Num) = rem(t_delay, dt);
                tDelayT{j}(k, i) = t_delay;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = floor(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay-floor(t_delay/dt)*dt);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                tDelay{j}(k, t_Num) = rem(t_delay, dt);
                tDelayT{j}(k, i) = t_delay;
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

% %% 画出目标运动的实际轨迹
% fig1 = figure('Units', 'centimeters', 'Position', [10, 5, 20, 11.24 / 15 * 15]);
% figure(fig1)
% hold on
% for i = 1:numOfSource
%     plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
% end
% scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
% legend('目标实际轨迹', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% title('目标实际运动轨迹');
% set(gca, 'Box', 'on')
% xlabel('东向坐标/m', 'FontSize', 12)
% ylabel('北向坐标/m', 'FontSize', 12)

%%
t_obs = 2 + T:T:(T_num - 10) * T; % 截取10秒以后的一段数据
angM = cell(length(t_obs), numOfPlatForm);
tDelayM = cell(length(t_obs), numOfPlatForm);
tDelayTM = cell(length(t_obs), numOfPlatForm);
tarInfo = cell(length(t_obs), numOfPlatForm);
position = cell(length(t_obs), 1);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    tDelayM(iii, :) = arrayfun(@(s) tDelay{s}(~isnan(sort(tDelay{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    tDelayTM(iii, :) = arrayfun(@(s) tDelayT{s}(~isnan(sort(tDelay{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    position(iii, :) = arrayfun(@(s) sourceAll(1).Position(t_obs(1)/T+iii-1, :), t_obs(1)/T+iii-1, 'un', 0);
end
% 使用cellfun结合匿名函数，将每个cell中的NaN值清除
angMWithoutNaN = cellfun(@(x) x(~isnan(x)), angM, 'UniformOutput', false);
tDelayMWithoutNaN = cellfun(@(x) x(~isnan(x)), tDelayM, 'UniformOutput', false);
tDelayTMWithoutNaN = cellfun(@(x) x(~isnan(x)), tDelayTM, 'UniformOutput', false);

%% 直接进行TDOA解算
len = length(tDelayMWithoutNaN);
res1 = zeros(len, 2);
timeDelay = zeros(len, numOfPlatForm);
position2 = zeros(len, 2);
Node = [node, zeros(numOfPlatForm, 1)];
%
% for i = 1:len
%     for j = 1:numOfPlatForm
%         if ~isempty(tDelayTMWithoutNaN{i, j})
%             timeDelay(i, j) = tDelayTMWithoutNaN{i, j};
%         end
%     end
%     [res1(i, :), ~] = TDOA(timeDelay(i, :), Node, 4);
% end
%
% figure(fig1)
% plot(res1(:, 1), res1(:, 2), 'r.');
% title("无距离模糊的TDOA解算轨迹");

for i = 1:len
    position2(i, :) = position{i};
    for j = 1:numOfPlatForm
        if ~isempty(tDelayMWithoutNaN{i, j})
            timeDelay(i, j) = tDelayMWithoutNaN{i, j};
        end
    end
    [res1(i, :), ~] = TDOA(timeDelay(i, :), Node, 4);
end

%% 画出目标运动的实际轨迹
fig1 = figure('Units', 'centimeters', 'Position', [10, 5, 20, 11.24 / 15 * 15]);
figure(fig1)
hold on
plot(position2(:, 1), position2(:, 2), '.');
plot(res1(:, 1), res1(:, 2), 'r.');
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标实际轨迹', 'TDOA解算轨迹', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
title("目标轨迹");

%% 加上抗距离模糊
% 结果不对，需要加上抗距离模糊
c = 1500;
Rmax = 2400;
Nmax = Rmax / (c * T); % 最大模糊周期
% res2 = nan(len, 2);
Node = [node(1:3, :), zeros(3, 1)];
% [temp, ~] = TDOA([0.544222176852228	1.09266727679462	1.26586922266437	], Node, 4);

%%
% 根据王静飞论文计算不同平台之间的N
nCompare = zeros(1, numOfPlatForm);
for i = 2:numOfPlatForm
    nCompare(i) = floor(pdist([node(1, :); node(i, :)])/(c * T));
end
trackNum = 0;
res2 = cell(len, 1);
tic;
parfor i1 = 1:len
    disp(i1);
    tDelay = zeros(1, numOfPlatForm);
    EstX = cell(1, 1);
    EstY = cell(1, 1);
    % 四个标举手表决，以中间位置为基准
    % iDelay1为几个标共同的传播周期
    for iDelay1 = 0:Nmax
        tDelay(1) = timeDelay(i1, 1) + iDelay1 * T;
        if tDelay(1) < 0 || tDelay(1) > Nmax * T
            continue;
        end
        % nCompare(2) = 6 记得改
        for i2 = max(1, i1-6):min(len, i1+6)
            for iDelay2 = 0:Nmax
                tDelay(2) = timeDelay(i2, 2) + iDelay2 * T;
                if tDelay(2) < 0 || tDelay(2) > Nmax * T
                    continue;
                end
                for i3 = max(1, i1-9):min(len, i1+9)
                    for iDelay3 = 0:Nmax
                        tDelay(3) = timeDelay(i3, 3) + iDelay3 * T;
                        if tDelay(3) < 0 || tDelay(3) > Nmax * T
                            continue;
                        end
                        for i4 = max(1, i1-6):min(len, i1+6)
                            for iDelay4 = 0:Nmax
                                tDelay(4) = timeDelay(i4, 4) + iDelay4 * T;
                                if tDelay(4) < 0 || tDelay(4) > Nmax * T
                                    continue;
                                end
                                pointOff = nan(4, 2);
                                temp = zeros(1, 2);
                                % 得到了4个平台的trelipie， 用三个平台求解，1个平台判解，四种组合方式
                                % 组合一 123求解 4判解
                                TimeDelay = tDelay(1, 1:3);
                                Node = [node(1:3, :), zeros(3, 1)];
                                [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
                                if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
                                    pointOff(1, :) = temp(1, :);
                                    % 求解算点到基准平台间的时延与所测的时延的差
                                    tC4 = TimeDelay(1) - pdist([node(1, :); temp]) / c;
                                    % 解算点到判解平台间的距离
                                    r4 = pdist([node(4, :); temp]);
                                    nT4 = (r4 / c + tC4 - timeDelay(i1, 1)) / T;
%                                     if abs(nT4-round(nT4)) > T || r4 >
%                                     Rmax || abs(tC4) > T %这里改了

                                    if abs(nT4-round(nT4)) > T || r4 > Rmax || abs(tC4) > T || tC4 < 0
                                        continue;
                                    end
                                else
                                    continue;
                                end

                                %% 组合二 234求解 1判解 2#为基准
                                %                                 disp("组合2");
                                TimeDelay = tDelay(1, 2:4);
                                Node = [node(2:4, :), zeros(3, 1)];
                                [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
                                if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
                                    pointOff(2, :) = temp(1, :);
                                    % 求解算点到基准平台间的时延与所测的时延的差
                                    tC1 = TimeDelay(1) - pdist([node(2, :); temp]) / c;
                                    % 解算点到判解平台间的距离
                                    r1 = pdist([node(1, :); temp]);
                                    nT1 = (r1 / c + tC1 - timeDelay(i2, 2)) / T;
                                    if abs(nT1-round(nT1)) > T || r1 > Rmax || abs(tC1) > T || tC1 < 0
                                        continue;
                                    end
                                else
                                    continue;
                                end

                                %% 组合三 341求解 2判解 3#为基准
                                %                                 disp("组合3");
                                TimeDelay = [tDelay(1, 3:4), tDelay(1, 1)];
                                Node = [[node(3:4, :); node(1, :)], zeros(3, 1)];
                                [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
                                if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
                                    pointOff(3, :) = temp(1, :);
                                    % 求解算点到基准平台间的时延与所测的时延的差
                                    tC2 = TimeDelay(1) - pdist([node(3, :); temp]) / c;
                                    % 解算点到判解平台间的距离
                                    r2 = pdist([node(2, :); temp]);
                                    nT2 = (r2 / c + tC2 - timeDelay(i3, 3)) / T;
                                    if abs(nT2-round(nT2)) > T || r2 > Rmax || abs(tC2) > T || tC2 < 0
                                        continue;
                                    end
                                else
                                    continue;
                                end

                                %% 组合四 412求解 3判解 4#为基准
                                %                                 disp("组合4");
                                TimeDelay = [tDelay(1, 4), tDelay(1, 1:2)];
                                Node = [[node(4, :); node(1:2, :)], zeros(3, 1)];
                                [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
                                if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
                                    pointOff(4, :) = temp(1, :);
                                    % 求解算点到基准平台间的时延与所测的时延的差
                                    tC3 = TimeDelay(1) - pdist([node(2, :); temp]) / c;
                                    % 解算点到判解平台间的距离
                                    r3 = pdist([node(3, :); temp]);
                                    nT3 = (r3 / c + tC3 - timeDelay(i4, 4)) / T;
                                    if abs(nT3-round(nT3)) > T || r3 > Rmax || abs(tC3) > T || tC3 < 0
                                        continue;
                                    end
                                else
                                    continue;
                                end

                                %% 判断是否是有效搜索
                                distances = pdist(pointOff);
                                if (all(distances < 200))
                                    EstX{end+1} = mean(pointOff(:, 1));
                                    EstY{end+1} = mean(pointOff(:, 2));
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    res2{i1} = {EstX, EstY};
end
toc;
% figure
% plot(res2(:, 1), res2(:, 2));
% title("TDOA解算轨迹（不进行时延差数据关联）");
res3 = cell(len, 1);
res4 = zeros(len, 2);
for i = 1:len
    res3{i} = [sort(cell2mat(res2{i}{1}))', sort(cell2mat(res2{i}{2}))'];
    % 计算三分之一的位置
    index = ceil(size(res3{i}(:, 1), 1)/3);
    res4(i, :) = [res3{i}(index, 1), res3{i}(index, 2)];
end
figure
plot(res4(:, 1), res4(:, 2), '.');

%% 异步抗距离模糊  参考JD1CJXPor SolTrackOutofStep_style1 暂不可用
% trackNum = 0;
% for i = 1:len
%     tDelay = zeros(1, numOfPlatForm);
%     if numOfPlatForm == 4
%         % 四个标举手表决，以中间位置为基准
%         % iDelay1为几个标共同的传播周期
%         for iDelay1 = 0:Nmax
%             tDelay(1) = timeDelay(i, 1) + iDelay1 * T;
%             if tDelay(1) < 0 || tDelay(1) > Nmax * T
%                 continue;
%             end
%             for iDelay2 = -Nmax:Nmax
%                 tDelay(2) = timeDelay(i, 2) + (iDelay1 + iDelay2) * T;
%                 if tDelay(2) < 0 || tDelay(2) > Nmax * T
%                     continue;
%                 end
%                 for iDelay3 = -Nmax:Nmax
%                     tDelay(3) = timeDelay(i, 3) + (iDelay1 + iDelay3) * T;
%                     if tDelay(3) < 0 || tDelay(3) > Nmax * T
%                         continue;
%                     end
%                     for iDelay4 = -Nmax:Nmax
%                         tDelay(4) = timeDelay(i, 4) + (iDelay1 + iDelay4) * T;
%                         if tDelay(4) < 0 || tDelay(4) > Nmax * T
%                             continue;
%                         end
%
%                         % 水平位置解算
%                         %% 组合一 123 1#为基准
%                         TimeDelay = tDelay(1, 1: 3);
%                         Node = [node(1: 3, :), zeros(3, 1)];
%                         [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
%                         if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
%                             pointOff(1, :) = temp(1, :);
%                             % 求解算点到基准平台间的时延与所测的时延的差
%                             ts4 = pdist([node(1, :); pointOff]) / c - TimeDelay(1);
%                             % 解算点到判解平台间的距离
%                             r4 = pdist([node(4, :); pointOff]);
%                             nT4 = (r4 / c - (tDelay(4) - ts4)) / T;
%                             if (nT4 - round(nT4)) > 0.1 || r4 > Rmax || ts4 < 0 || ts4 > Nmax * T
%                                 continue;
%                             end
%                         else
%                             continue;
%                         end
%                         rr1 = pdist([node(1, :); pointOff(1, :)]);
%                         rr2 = pdist([node(2, :); pointOff(1, :)]);
%                         rr3 = pdist([node(3, :); pointOff(1, :)]);
%                         % 1 2 交汇，3判解
%                         TimeDelay = [rr1, rr2, rr3] ./ c;
%                         [pointIn(1, :), ~] = TDOA(TimeDelay, Node, 3);
%                         % 1 3 交汇，2判解
%                         TimeDelay = [rr1, rr3, rr2] ./ c;
%                         Node1(1, :) = Node(1, :);
%                         Node1(2, :) = Node(3, :);
%                         Node1(3, :) = Node(2, :);
%                         [pointIn(2, :), ~] = TDOA(TimeDelay, Node1, 3);
%                         % 2 3 交汇，1判解
%                         TimeDelay = [rr2, rr3, rr1] ./ c;
%                         Node1(1, :) = Node(2, :);
%                         Node1(2, :) = Node(3, :);
%                         Node1(3, :) = Node(1, :);
%                         [pointIn(3, :), ~] = TDOA(TimeDelay, Node1, 3);
%                         % 异步解与三个同步解的距离
%                         rError = arrayfun(@(i) pdist([pointIn(i, :); pointOff]), 1:3);
%
%                         % 通过同步解与异步解的位置关系，进行野点的剔除，仅保留异步解
%                         if (all(rError < 900) || (rError(1) < 30 && rError(2) < 30) || (rError(1) < 30 && rError(3) < 30) || (rError(2) < 30 && rError(3) < 30))
%
%                         else
%                             continue;
%                         end
%
%                         %% 组合二 234 2# 为基准
%                         disp("组合2")
%                         TimeDelay = tDelay(1, 2: 4);
%                         Node = [node(2: 4, :), zeros(3, 1)];
%                         [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
%                         if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
%                             pointOff(2, 1) = temp(1, 1);
%                             pointOff(2, 2) = temp(1, 2);
%                             % 求解算点到基准平台间的时延与所测的时延的差
%                             ts1 = pdist([node(2, :); pointOff(2, :)]) / c - TimeDelay(1);
%                             % 解算点到判解平台间的距离
%                             r1 = pdist([node(1, :); pointOff(2, :)]);
%                             nT1 = (r1/c - (tDelay(1) - ts1)) / T;
%                             if abs(nT1 - round(nT1)) > 0.1 || r1 > Rmax || ts1 < 0 || ts1 > T
%                                 continue;
%                             end
%                         else
%                             continue;
%                         end
% %                         rr1 = pdist([node(2, :), pointOff]);
% %                         rr2 = pdist([node(3, :), pointOff]);
% %                         rr3 = pdist([node(4, :), pointOff]);
% %                         % 1 2 交汇，3判解
% %                         TimeDelay = [rr1, rr2, rr3] ./ c;
% %                         [pointIn(1, :), ~] = TDOA(TimeDelay, Node, 3);
% %                         % 1 3 交汇，2判解
% %                         TimeDelay = [rr1, rr3, rr2] ./ c;
% %                         Node1(1, :) = Node(1, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(2, :);
% %                         [pointIn(2, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 2 3 交汇，1判解
% %                         TimeDelay = [rr2, rr3, rr1] ./ c;
% %                         Node1(1, :) = Node(2, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(1, :);
% %                         [pointIn(3, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 异步解与三个同步解的距离
% %                         rError = arrayfun(@(i) pdist([pointIn(i, :); pointOff]), 1:3);
% %                         % 通过同步解与异步解的位置关系，进行野点的剔除，仅保留异步解
% %                         if (all(rError < 90) || (rError(1) < 30 && rError(2) < 30) || (rError(1) < 30 && rError(3) < 30) || (rError(2) < 30 && rError(3) < 30))
% %                             ;
% %                         else
% %                             continue;
% %                         end
%                         %% 组合三 341
%                         TimeDelay = [tDelay(1, 3: 4), tDelay(1, 1)];
%                         Node = [[node(3: 4, :); node(1, :)], zeros(3, 1)];
%                         [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
%                         if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
%                             pointOff(3, 1) = temp(1, 1);
%                             pointOff(3, 2) = temp(1, 2);
%                             % 求解算点到基准平台间的时延与所测的时延的差
%                             ts2 = pdist([node(3, :); pointOff(3, :)]) / c - TimeDelay(1);
%                             % 解算点到判解平台间的距离
%                             r2 = pdist([node(2, :); pointOff(3, :)]);
%                             nT2 = (r2/c - (tDelay(2) - ts3)) / T;
%                             if abs(nT2 - round(nT2)) > 0.1 || r2 > Rmax || ts2 < 0 || ts2 > T
%                                 continue;
%                             end
%                         else
%                             continue;
%                         end
% %                         rr1 = pdist([node(3, :), pointOff]);
% %                         rr2 = pdist([node(4, :), pointOff]);
% %                         rr3 = pdist([node(1, :), pointOff]);
% %                         % 1 2 交汇，3判解
% %                         TimeDelay = [rr1, rr2, rr3] ./ c;
% %                         [pointIn(1, :), ~] = TDOA(TimeDelay, Node, 3);
% %                         % 1 3 交汇，2判解
% %                         TimeDelay = [rr1, rr3, rr2] ./ c;
% %                         Node1(1, :) = Node(1, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(2, :);
% %                         [pointIn(2, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 2 3 交汇，1判解
% %                         TimeDelay = [rr2, rr3, rr1] ./ c;
% %                         Node1(1, :) = Node(2, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(1, :);
% %                         [pointIn(3, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 异步解与三个同步解的距离
% %                         rError = arrayfun(@(i) pdist([pointIn(i, :); pointOff]), 1:3);
% %                         % 通过同步解与异步解的位置关系，进行野点的剔除，仅保留异步解
% %                         if (all(rError < 90) || (rError(1) < 30 && rError(2) < 30) || (rError(1) < 30 && rError(3) < 30) || (rError(2) < 30 && rError(3) < 30))
% %                             ;
% %                         else
% %                             continue;
% %                         end
%                         %% 组合四 412
%                         TimeDelay = [tDelay(1, 4), tDelay(1, 1: 2)];
%                         Node = [[node(4, :); node(1: 2, :)], zeros(3, 1)];
%                         [temp(1, :), ~] = TDOA(TimeDelay, Node, 4);
%                         if ~isnan(temp(1, 1)) && ~isnan(temp(1, 2))
%                             pointOff(4, 1) = temp(1, 1);
%                             pointOff(4, 2) = temp(1, 2);
%                             % 求解算点到基准平台间的时延与所测的时延的差
%                             ts3 = pdist([node(4, :); pointOff(4, :)]) / c - TimeDelay(1);
%                             % 解算点到判解平台间的距离
%                             r3 = pdist([node(3, :); pointOff(4, :)]);
%                             nT3 = (r3/c - (tDelay(3) - ts3)) / T;
%                             if abs(nT3 - round(nT3)) > 0.1 || r3 > Rmax || ts3 < 0 || ts3 > T
%                                 continue;
%                             end
%                         else
%                             continue;
%                         end
% %                         rr1 = pdist([node(4, :), pointOff]);
% %                         rr2 = pdist([node(1, :), pointOff]);
% %                         rr3 = pdist([node(2, :), pointOff]);
% %                         % 1 2 交汇，3判解
% %                         TimeDelay = [rr1, rr2, rr3] ./ c;
% %                         [pointIn(1, :), ~] = TDOA(TimeDelay, Node, 3);
% %                         % 1 3 交汇，2判解
% %                         TimeDelay = [rr1, rr3, rr2] ./ c;
% %                         Node1(1, :) = Node(1, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(2, :);
% %                         [pointIn(2, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 2 3 交汇，1判解
% %                         TimeDelay = [rr2, rr3, rr1] ./ c;
% %                         Node1(1, :) = Node(2, :);
% %                         Node1(2, :) = Node(3, :);
% %                         Node1(3, :) = Node(1, :);
% %                         [pointIn(3, :), ~] = TDOA(TimeDelay, Node1, 3);
% %                         % 异步解与三个同步解的距离
% %                         rError = arrayfun(@(i) pdist([pointIn(i, :); pointOff]), 1:3);
% %                         % 通过同步解与异步解的位置关系，进行野点的剔除，仅保留异步解
% %                         if (all(rError < 90) || (rError(1) < 30 && rError(2) < 30) || (rError(1) < 30 && rError(3) < 30) || (rError(2) < 30 && rError(3) < 30))
% %                             ;
% %                         else
% %                             continue;
% %                         end
%
%                         %% 对解的聚集度进行判断
%                         distances = pdist(pointIn);
%                         if (all(distances < 200))
%                             trackNum = trackNum + 1;
%                             ts(trackNum) = mean([ts1, ts2, ts3, ts4]);
%                             EstX(trackNum) = pointOff(1);
%                             ExtY(trackNum) = pointOff(2);
%
%                         end
%                     end % for iDelay4
%                 end % for iDelay3
%             end % for iDelay2
%         end % for iDelay1
%     end % if numOfPlatForm == 4
%
%
%     fprintf("得到的解的数量为 %d\n", trackNum);
%     if trackNum > 0
%         disp(EstX);
%         disp(EstY);
%     end
% end


% for i = 1:len
%     % 找到t1
%     t1 = tDelayMWithoutNaN{i, 1};
%     nn = 0; % 组合的结果的个数
%     for iDelay1 = 0:Nmax
%         t1_ = t1 + iDelay1 * T;
%         t2 = tDelayMWithoutNaN{i, 2};
%         for iDelay2 = 0:Nmax
%             t2_ = t2 + iDelay2 * T;
%             t3 = tDelayMWithoutNaN{i, 3};
%             for iDelay3 = 0:Nmax
%                 t3_ = t3 + iDelay3 * T;
%                 t4 = tDelayMWithoutNaN{i, 3};
%                 for iDelay4 = 0:Nmax
%                     t4_ = t4 + iDelay4 * T;
%                     timeDelay = [t1_, t2_, t3_, t4_];
%                     % 得到这一种组合的结果
%                     nn = nn + 1;
%                     [temp(nn, :), ~] = TDOA(timeDelay, Node);
%                 end
%             end
%         end
%     end
%
%     % 判断解的聚集度
%
%     disp(temp);
%
%
% end

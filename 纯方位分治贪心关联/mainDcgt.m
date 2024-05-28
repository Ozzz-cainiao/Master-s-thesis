%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainDcgt.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-01-02
% 描述: 这个函数专门用来对分治贪心关联进行仿真，验证算法功能
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.1; %观测周期
T_all = 10; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
% var2d = 1.5^2; % 角度制  角度误差
var2d = 1^2; % 角度制  角度误差
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
initial_position1 = [4e3, 7e3]; % 初始位置目标1

% initial_position1 = [4e3, 7e3]; % 初始位置目标1
velocity1 = [0, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', 2e3, 100, initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [0, 0]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [5e3, 2e3]; % 初始位置目标2
velocity3 = [0, 0]; % 运动速度
acc3 = 0; % 加速度
source3 = SoundSource('CW', [2e3], [100], initial_position3, velocity3, F1, F2, acc3);
initial_position4 = [3e3, 1e3]; % 初始位置目标2
% initial_position4 = [4e3, 1.5e3]; % 初始位置目标2

velocity4 = [0, 0]; % 运动速度
acc4 = 0; % 加速度
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);
initial_position5 = [8e3, 5e3]; % 初始位置目标2
velocity5 = [0, 0]; % 运动速度
acc5 = 0; % 加速度
source5 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position5, velocity5, F1, F2, acc5);

% sourceAll = [source1, source2];
sourceAll = [source1, source2, source3, source4, source5];

% 布放平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];
% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource); % cell矩阵
angR = cell(1, numOfPlatForm); % 存放带误差的角度
realangR = cell(1, numOfPlatForm); % 存放真实的角度
realwuT = cell(1, numOfPlatForm); % 存放真实的角度

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
%     sourceAll = [source1, source2];
sourceAll = [source1, source2, source3, source4, source5];

    angR{j} = nan(numOfSource, T_num+100); % 现在只用来存放方位信息
    realangR{j} = nan(numOfSource, T_num+100);
    for k = 1:numOfSource % 遍历声源
        % 创建结构体数组
        numStructs = T_num + 100;
        myStructArray = repmat(struct('angle', nan, 'type', nan, 'fre', nan, 't_delay', nan), numStructs, 1);
        % 填充结构体数组
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
%                 angR{j}(k, t_Num) = angle + 0;

                realangR{j}(k, t_Num) = angle;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
%                 angR{j}(k, t_Num) = angle + 0; % 这个结果是度
                angR{j}(k, t_Num) = round(angle + sqrt(var2d) * randn, 2);

                realangR{j}(k, t_Num) = angle; % 这个结果是度
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
% fig1 = figure('Units', 'centimeters', 'Position', [10, 5, 20, 11.24 / 15 * 15]);
% figure(fig1)
figure
hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend( '目标1', '目标2','观测站', 'FontSize', 12)
title('目标实际运动轨迹');
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
%%
t_obs = 5:T:T_num * T; % 截取10-50s的数据
angM = cell(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
end

%% 探测节点以及目标位置布置场景
birthPlace = zeros(numOfSource, 2);
for i = 1:numOfSource
    birthPlace(i, 1) = sourceAll(i).Position(1, 1);
    birthPlace(i, 2) = sourceAll(i).Position(1, 2);
end
Ylim    = [0,10e3];
xgrid = 0:10:10e3;
y = cell(numOfPlatForm, 1);
% fig = figure('Units', 'centimeters', 'Position', [20, 5, 20, 11.24 / 15 * 15]);
fig = figure;
for s = 1:numOfPlatForm
    theta = angR{s}(:, 80);
    xp = node(s, 1);
    yp = node(s, 2);
    y{s} = cell2mat(arrayfun(@(x) (repmat(x-xp, length(theta), 1)).*tand(theta)+repmat(yp, length(theta), 1), xgrid, 'un', 0)); % 在反向线上的点舍去
    locsp2 = arrayfun(@(x) y{s}(x, :) <= Ylim(1) | y{s}(x, :) >= Ylim(2), 1:length(theta), 'un', 0); % 超过视距的点
    y{s}(cell2mat(arrayfun(@(v) v, cell2mat(locsp2'), 'un', 0))) = nan; % 超过视距的点舍去
    figure(fig)
    hold on
    h = arrayfun(@(x) plot(xgrid, y{s}(x, :), '--', 'Color', '#808080'), 1:length(theta));
    hold off
end
figure(fig)
hold on
s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
s2 = scatter(birthPlace(:, 1), birthPlace(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100);
legend([h(end), s1, s2], '方位测量', '观测站', '目标', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)

%% 分治贪心关联
% 调用709函数 传入参数 角度 平台数 平台位置 目标数量
[outLoctionCAX, outLoctionCAY, outLoctionSPCX, outLoctionSPCY] = calcAll(angM, numOfPlatForm, node, numOfSource, t_obs, T);

% 计算定位结果的平均值
resX = nanmean(outLoctionSPCX, 2);
resY = nanmean(outLoctionSPCY, 2);
Ylim    = [0,10e3];
xgrid = 0:10:10e3;
y = cell(numOfPlatForm, 1);
% fig = figure('Units', 'centimeters', 'Position', [10, 5, 20, 11.24 / 15 * 15]);
fig = figure;
figure(fig)
hold on
s1 = scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100, 'DisplayName', '平台');
s2 = scatter(birthPlace(:, 1), birthPlace(:, 2), 'rp', 'filled', 'LineWidth', 1, 'SizeData', 100, 'DisplayName', '预设目标');
% s3 = scatter(outLoctionSPCX(:, 40), outLoctionSPCY(:, 40),'bs','LineWidth', 1, 'SizeData', 100);
% 计算第 40 列非 NaN 元素的数量
nonNaNCount = sum(~isnan(outLoctionSPCX(:,25)));
for i = 1 : nonNaNCount
    s3(i) = scatter(outLoctionSPCX(i, 25), outLoctionSPCY(i, 25),'s','LineWidth', 1, 'SizeData', 100,'DisplayName', ['目标', num2str(i)] );
end
legend;
% legend([s1, s2, ], '目标定位结果', '观测站', '目标', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)

%% 计算定位误差 定位结果顺序和原顺序不一致
% birthPlace = sortrows(birthPlace, 1);
% res = sortrows([resX, resY], 1);
% for i = 1 : numOfSource
%     for j = 1 : numOfSource
%         error(i) = min(error(i), pdist2(res(i,:), birthPlace(j, :)));
%     end
% end
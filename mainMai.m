%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainMai.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2024-01-05
% 描述: 用来仿真时延差/方位联合时间关联，来区分相干的脉冲信号，在这里加上周期模糊进行仿真
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.5; %观测周期
T_all = 60; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
var2d = 1^2; % 角度制  角度误差
pd = 0.9; % 检测概率
c = 1500;
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
velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, F1, F2, acc1);

initial_position2 = [7e3, 2e3]; % 初始位置目标2
velocity2 = [10, 10]; % 运动速度
acc2 = 0; % 加速度
source2 = SoundSource('CW', [2e3], [100, 50], initial_position2, velocity2, F1, F2, acc2);

sourceAll = [source1, source2];

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
tDelay = cell(1, numOfPlatForm); % 存放测得的时延

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, T_num+20); % 现在只用来存放方位信息
    tDelay{j} = nan(numOfSource, T_num+20);
    for k = 1:numOfSource % 遍历声源
        % 创建结构体数组
        numStructs = T_num + 20;
        myStructArray = repmat(struct('angle', nan, 'type', nan, 'fre', nan, 't_delay', nan), numStructs, 1);
        % 填充结构体数组
        for i = 1:T_num
            if i == 1
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = floor(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                % 模糊周期 N = floor(t_delay/dt);
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                tDelay{j}(k, t_Num) = t_delay;
                %                 myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay - floor(t_delay/dt) * dt);
                %                 angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                %                 tDelay{j}(k, t_Num) = rem(t_delay, dt);
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = floor(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                myStructArray(t_Num) = struct('angle', angle, 'type', type, 'fre', fre, 't_delay', t_delay);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
                tDelay{j}(k, t_Num) = t_delay;
            end
        end % for i = 1: T_num
        % 将结构体数组存放在当前的位置
        target_info_matrix{j, k} = myStructArray;
    end % for k = 1:numOfSource % 遍历声源
end % for j = 1:numOfPlatForm

%% 画出目标运动的实际轨迹
fig1 = figure('Units', 'centimeters', 'Position', [10, 5, 20, 11.24 / 15 * 15]);
figure(fig1)
hold on
for i = 1:numOfSource
    plot(sourceAll(i).Position(:, 1), sourceAll(i).Position(:, 2), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
title('目标实际运动轨迹');
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)

%%
t_obs = T:T:(T_num - 50) * T; % 截取10秒以后的一段数据
angM = cell(length(t_obs), numOfPlatForm);
tDelayM = cell(length(t_obs), numOfPlatForm);
tarInfo = cell(length(t_obs), numOfPlatForm);
for iii = 1:length(t_obs)
    angM(iii, :) = arrayfun(@(s) angR{s}(~isnan(sort(angR{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
    tDelayM(iii, :) = arrayfun(@(s) tDelay{s}(~isnan(sort(tDelay{s}(:, t_obs(1) / T + iii - 1))), t_obs(1) / T + iii - 1), 1:numOfPlatForm, 'un', 0);
end
% 使用cellfun结合匿名函数，将每个cell中的NaN值清除
angMWithoutNaN = cellfun(@(x) x(~isnan(x)), angM, 'UniformOutput', false);
tDelayMWithoutNaN = cellfun(@(x) x(~isnan(x)), tDelayM, 'UniformOutput', false);
%% 先进行传统的TDOA计算
% 下方参考李晴的博士论文
% 定位系统水声作用距离Rmax = 2400 这个值应该大于基线长度
% 最大非模糊距离R0=T * c = 0.5 * 1500 = 750
% 最大传播周期数 Nmax = Rmax / R0;
% 在这里需要解决模糊周期的问题
R0 = T * c;
Rmax = 12e3; % 设定的
Nmax = Rmax / R0;

%% 先不考虑模糊周期以及相同帧的不匹配问题，就考虑最基础的多目标TDOA定位

%% 使用分治贪心关联获取关联的线序号，然后使用TDOA进行定位
% 调用709函数 传入参数 角度 平台数 平台位置 目标数量
% [outLoctionCAX, outLoctionCAY, outLoctionSPCX, outLoctionSPCY] = calcR(angM, numOfPlatForm, node, numOfSource, t_obs, T);


% 创建一个与原始矩阵相同大小的矩阵，用于存储编号情况
numbered_matrix = cell(size(tDelayMWithoutNaN));

% 遍历原始矩阵的每个元素
for i = 1:size(tDelayMWithoutNaN, 1)
    for j = 1:size(tDelayMWithoutNaN, 2)
        current_cell = tDelayMWithoutNaN{i, j};
        % 判断当前cell的情况并进行编号
        if any(isempty(current_cell)) || any(isnan(current_cell))
            % cell为空或为nan，编号为0
            numbered_matrix{i, j} = 0;
        else
            % cell中有1个或2个double数据，进行相应编号
            numbered_matrix{i, j} = 0:numel(current_cell);
        end
    end
end
% 假设要查找的位置是第11行第3列，序号为2
row_index = 11;
col_index = 3;
target_number = 2;

% 通过序号在原矩阵中查找对应的值
corresponding_value = tDelayMWithoutNaN{row_index, col_index}(target_number);

disp(['在原矩阵中找到的值为: ', num2str(corresponding_value)]);

% 初始化行列索引
row_index = [];
col_index = [];

% 遍历每一行，找到第一次出现1的位置
for i = 1:size(numbered_matrix, 1)
    col_index = find(cellfun(@(x) any(x == 1), numbered_matrix(i, :)), 1, 'first');
    if ~isempty(col_index)
        row_index = i;
        break; % 找到第一次出现1的位置后退出循环
    end
end

% 显示找到的位置
disp(['第一次出现1的位置：行 ', num2str(row_index), ' 列 ', num2str(col_index)]);

% 先把数据提取出来
for k = 1:size(tDelayM, 1)    
    % 首先取出方位数据
    for i1 = 1:numel(angMWithoutNaN{k, 1})
        % 取出S1的方位数据
        a1 = angMWithoutNaN{k, 1}(i1);
        th1 = tDelayMWithoutNaN{k, 1}(i1); % S1只取当前时刻的
        if isnan(a1) || isempty(a1)
            continue;
        else
            SValueT = cell(numOfPlatForm-1, Nmax+1);
            SValueA = cell(numOfPlatForm-1, Nmax+1);
            for i = 2:numOfPlatForm
                for j = k:min(k+Nmax, size(tDelayMWithoutNaN, 1))
                    SValueT{i - 1, j - k + 1} = tDelayMWithoutNaN{j, i};
                    SValueA{i - 1, j - k + 1} = angMWithoutNaN{j, i};
                end
            end
            
            for iDelay1 = k:k + Nmax
                % 取方位
                mm1 = numel(SValueA{1, iDelay1 - k + 1});
                for iDelay2 = k:k + Nmax
                    % 取方位
                    mm2 = numel(SValueA{2, iDelay2 - k + 1});
%                     for iDelay3 = k:k + Nmax
%                         % 取方位
%                         mm3 = numel(SValueA{3, iDelay3 - k + 1});
                        % 开始穷举
                        for m1 = 1:mm1
                            a2 = SValueA{1, iDelay1 - k + 1}(m1);
                            th2 = SValueT{1, iDelay1 - k + 1}(m1);
                            for m2 = 1:mm2
                                a3 = SValueA{2, iDelay2 - k + 1}(m2);
                                th3 = SValueT{2, iDelay2 - k + 1}(m2);
%                                 for m3 = 1 : mm3
% 
%                                 end
                                % 组合
                                combinedData = [a1, a2, a3]; % 这是三个方位角
                                
                                %% 利用方位信息求出时延差 参考公式 JZQ3-9 自己推的公式在iPad上
                                % $\tao = \frac{L}{c}\frac{sin(\theta_i - \beta) - sin(\theta_j - \beta)}{sin(\theta_i + \theta_j)}$ 
                                % 求两平台连线长度和两平台连线与X正的夹角
                                L1 = sqrt((node(1, 1) - node(2, 1))^2 + (node(1, 2) - node(2, 2))^2);
                                L2 = sqrt((node(1, 1) - node(3, 1))^2 + (node(1, 2) - node(3, 2))^2);
                                L3 = sqrt((node(2, 1) - node(3, 1))^2 + (node(2, 2) - node(3, 2))^2);
                                
                                beta1 = atan2d(node(2, 2) - node(1, 2), node(2, 1) - node(1, 1));
                                beta2 = atan2d(node(3, 2) - node(1, 2), node(3, 1) - node(1, 1));
                                beta3 = atan2d(node(3, 2) - node(2, 2), node(3, 1) - node(2, 1));

                                % 根据计算的时延差有几种可能的目标组合 这是方位交汇获得的时延差
                                h1 = L1 / c * (abs(sin(a1 - beta1)) - abs(sin(a2 + beta1))) / sin(a1 + a2);
                                h2 = L2 / c * (abs(sin(a1 - beta2)) - abs(sin(a3 + beta2))) / sin(a1 + a3);
                                h3 = L3 / c * (abs(sin(a3 - beta3)) - abs(sin(a3 + beta3))) / sin(a3 + a2);
                                % 计算由时延值测得的时延差
                                ts1 = abs(th1 - th2);
                                ts2 = abs(th1 - th3);
                                ts3 = abs(th2 - th3);

                                
                                % 使用评价函数进行计算
                                G{iDelay1, iDelay2}(i1, m1, m2) = abs(ts1 - h1) + abs(ts2 - h2) + abs(ts3 - h3);


                                % 解算


                                % 现在加上平台1有三个平台可以解算了
                                % 3个平台解算，1个平台判解
%                                 [res, loc] = TDOA(timeDelay, node);
                                % 判断解是否合理

                            end
                        end

                    
                end
            end
        end

    end


end

%% 这里找个G最小的，如何判断G最小？
% 使用3平台求解，1平台判解
% 截取当前行以后的数据
SValue = cell(3, Nmax+1);

for i = 2:numOfPlatForm
    for k = start_row:end_row
        if i ~= col_index

        end

    end
end


for i = row_index:row_index
    % 参与解算的时延个数
    for iDelay1 = start_row:end_row
        % 判断解的个数
        for iDelay2 = start_row:end_row

        end


    end

end
% 提取指定行号范围的数据
selected_data = cell(end_row-start_row+1, size(numbered_matrix, 2));
for i = start_row:end_row
    for j = 1:size(numbered_matrix, 2)
        selected_data{i - start_row + 1, j} = tDelayM{i, j}(numbered_matrix{i, j} == 1);
    end
end

% 显示提取的数据
disp('提取的数据：');
disp(selected_data);
tk = tDelayM{k, base}; % 时延值


%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\mainTest.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-22
% 描述: 作为面向对象编程的主程序，在这个程序中实现对各个类的实例化
% 输入:
% 输出:
%**************************************************************************

%%
clc
clear
close all

%% 观测数据
T = 0.5; %观测周期
T_all = 50; %观测时间
T_num = T_all / T; %观测次数
dt = T; % 观测周期
var2d = 1.5^2; % 角度制  角度误差

%% 创建平台
platform1 = Platform([0, 0]);
platform2 = Platform([1e4, 0]);
platform3 = Platform([1e4, 1e4]);
platform4 = Platform([0, 1e4]);

platFormAll = [platform1, platform2, platform3, platform4];
node = [0, 0; 1e4, 0; 1e4, 1e4; 0, 1e4];

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

%% 创建运动的声源对象
initial_position1 = [3e3, 3e3]; % 初始位置目标1
velocity1 = [10, 0]; % 运动速度（假设在 x 轴上匀速运动）
acc1 = 0; % 加速度
% source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, acc1);
source1 = SoundSource('CW', [2e3], [100], initial_position1, velocity1, F1, F2, acc1);


initial_position2 = [7e3, 5e3]; % 初始位置目标2
velocity2 = [-15, 15]; % 运动速度
acc2 = 0; % 加速度
% source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, acc2);
source2 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

initial_position3 = [5e3, 5e3]; % 初始位置目标2
velocity3 = [10, 0]; % 运动速度
acc3 = 0; % 加速度
% source3 = SoundSource('CW', [1e3, 2e3], [100, 50], initial_position3, velocity3, acc3);
source3 = SoundSource('CW', [2e3], [100], initial_position3, velocity3, F1, F2, acc3);

initial_position4 = [2e3, 7e3]; % 初始位置目标2
velocity4 = [0, 10]; % 运动速度
acc4 = 0; % 加速度
% source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, acc4);
source4 = SoundSource('CW', [1e3, 2e3], [100, 500], initial_position4, velocity4, F1, F2, acc4);

initial_position5 = [3e3, 5e3]; % 初始位置目标2
velocity5 = [-10, 10]; % 运动速度
acc5 = 0; % 加速度
source5 = SoundSource('LFM', [1e2, 2e2], [100, 50], initial_position2, velocity2, F1, F2, acc2);

sourceAll = [source1, source2, source3, source4, source5];

%% 创建一个多维矩阵来存储目标信息
% 维度1：平台，维度2：时刻，维度3：目标
numOfPlatForm = size(platFormAll, 2);
numOfSource = size(sourceAll, 2);
target_info_matrix = cell(numOfPlatForm, numOfSource, T_all+1); % cell矩阵
angR = cell(1, numOfPlatForm);
t_obs = 0:T:T_num * T + 20;

%% 观测

% 获取每个平台的每个目标信息
for j = 1:numOfPlatForm % 遍历平台
    angR{j} = nan(numOfSource, length(t_obs)); % 现在只用来存放方位信息
    for i = 1:T_num
        for k = 1:numOfSource % 遍历声源
            if i == 1
                %                 angR{j} = repmat(struct('angle', [], 'type', [], 'fre', []), T_all, numOfSource);

                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), 0);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn;
            else
                sourceAll(k) = sourceAll(k).updatePosition();
                [angle, ~, t_delay, type, fre] = platFormAll(j).getTargetInfo(sourceAll(k), dt);
                t_Num = round(t_delay/dt) + i; % 放到此时刻传播时延之前的时刻
                target_info_matrix{j, t_Num, k} = struct('angle', angle, 'type', type, 'fre', fre); %
                %                 angR{j}(t_Num, k) = struct('angle', angle, 'type', type, 'fre', fre);
                angR{j}(k, t_Num) = angle + sqrt(var2d) * randn; % 这个结果是度
            end
        end % for k = 1:numOfSource % 遍历声源
    end % for j = 1:numOfPlatForm % 遍历平台


    % 在这里拆分目标


    % 在这里实时定位
    % 时空关联

    % 这个10是随机设置的


end % for i = 1:T_num
% ==========================进行时空关联定位===============================
area = 0:1000:10e3;
Dth = 500; % 交点距离约束
d_min = 5; % 时空关联解算阈值
DeTh = 10; % 迭代次数阈值
newcell = cell(1, numOfPlatForm);

for i = 1:length(angR)
    % 这里有问题
    newcell{i,1} = angR{i}(1, :); % 提取出来自一个目标的
    newcell{i,2} = angR{i}(5, :); % 提取出来自一个目标的
end
for iii = 1:length(t_obs)
    d = inf;
    if iii > 1
        inputX = outLoctionSPCX(:, iii-1);
        inputY = outLoctionSPCY(:, iii-1);
    else
        [inputX, inputY] = deal(nan);
    end
    %             Zt = cell2mat(arrayfun(@(ii) angR{ii}(:, iii), 1:size(node, 1), 'un', 0));
    % Zt是2*4double 2目标4平台
    % 单目标时是1*4double
    Zt = cell2mat(arrayfun(@(ii) newcell{ii}(:, iii), 1:size(platFormAll, 2), 'un', 0));
    [midOldCAX1, midOldCAY1, locs{iii}] = CA_Plus(Zt, node, Dth, inputX, inputY);
    %     [outLoctionSPCX(1:num,iii),outLoctionSPCY(1:num,iii),locs{iii}]     = CA_Plus(Zt,node,Dth,inputX,inputY);
    % end
    for i = 1:1 % 这里是粗关联纯方位定位的目标个数
        if isnan(midOldCAX1(i))
            outLoctionSPCX(i, iii) = midOldCAX1(i);
            outLoctionSPCY(i, iii) = midOldCAY1(i);
        else
            iikk = 1;
            while d > d_min && ~isnan(midOldCAX1(i))
                x_e = midOldCAX1(i) - node(:, 1); % 估计位置与观测站横轴距离
                y_e = midOldCAY1(i) - node(:, 2); % 估计位置与观测站纵轴距离
                r_e = sqrt(x_e.^2+y_e.^2); % 估计位置与观测站之间的距离
                t_e = r_e / c;
                t_ed = round(t_e-t_e(1), 1); % 时延差
                if ~all(t_ed < 0)
                    t_ed(t_ed > 0) = 0;
                end
                loc_ed = round(t_ed/T) + iii; % 位置
                a_ed = arrayfun(@(ii) locs{loc_ed(ii)}(ii, i), 1:size(node, 1)); % 对应位置的测量序号
                s = 1:4;

                midZt = arrayfun(@(ii) newcell{ii}(a_ed(ii), loc_ed(ii)), s(~isnan(a_ed) & a_ed ~= 0));
                if length(midZt) > 2
                    [midNewCAX, midNewCAY] = CA(midZt, node(~isnan(a_ed) & a_ed ~= 0, :), Dth);
                elseif length(midZt) == 2
                    [midNewCAX, midNewCAY] = LSM(midZt, node(~isnan(a_ed) & a_ed ~= 0, :));
                else
                    [midNewCAX, midNewCAY] = deal(nan);
                    %                     error('需要调试')
                end


                midOldCAX = midNewCAX;
                midOldCAY = midNewCAY;
                iikk = iikk + 1;
                if iikk >= DeTh
                    break;
                end
            end
            outLoctionSPCX(i, iii) = midOldCAX;
            outLoctionSPCY(i, iii) = midOldCAY;
        end

    end


end

%%
a = 10
% 显示目标信息矩阵
% for i = 1 : 10
%     i
%     disp(target_info_matrix{1, i, 1});
% end
% for i = 1 : 10
%     i
%     disp(angR{1}(i, 1));
% end
% for i = 1 : 4
%     i
%     disp(target_info_matrix{1, 9, i});
% end

%% 将传感器数据格式化为数组
% % 定义四个结构体
% sensorData(1).angle = 43.421747348254513;
% sensorData(1).type = 'CW';
% sensorData(1).fre = 2000;
%
% sensorData(2).angle = 36.235899172122004;
% sensorData(2).type = 'LFM';
% sensorData(2).fre = 0;
%
% sensorData(3).angle = 45.763144826916495;
% sensorData(3).type = 'CW';
% sensorData(3).fre = [1000 2000];
%
% sensorData(4).angle = 74.618253631672147;
% sensorData(4).type = 'CW';
% sensorData(4).fre = [1000 2000];
%
% % 输出格式化后的结构数组
% disp(sensorData);

%% 数据对齐
% 确保不同传感器的数据在时间和空间上是对齐的
% 假装是对齐的 ，不要给自己增加实时处理的麻烦！

%% 数据合并
% 将来自不同传感器的数据按照时间和空间坐标存储在一个统一的数据结构中，
% 以便稍后的分析决策
% 基于时间的融合或基于特征的融合

%% 基于特征的融合
% CW

% LFM

% NOISE

%% 特征提取和融合

%% 多参量融合，看看怎么多参量融合

%% 分层关联

%% 实现纯方位交汇定位 经典写法

%% 实现双曲面交汇定位

%% 实现时延差/方位联合定位

%% 粗关联定位
function [EstX, EstY, locs] = CA_Plus(Zt, node, Dth, inputX, inputY)
if length(find(all(isnan(Zt), 1))) == 0
    Zt = Zt(:, ~all(isnan(Zt)));
    node1 = node(~all(isnan(Zt)), :);
    S = size(node1, 1); % 节点数
    angM = mat2cell(Zt, size(Zt, 1), ones(1, S)); % 角度特征元胞组
    nl = cellfun(@length, angM(:)); % 各个平台测量数量
    subs = arrayfun(@(x)0:x, nl, 'un', 0); % 生成序号
    iter = cell(S, 1);
    [iter{end:-1:1}] = ndgrid(subs{end:-1:1}); % 生成网格
    temp = cellfun(@(x)x(:), iter, 'un', 0); % 中间变量
    iter = num2cell(cat(2, temp{:})); % 所有组合
    Niter = size(iter, 1);
    temp = arrayfun(@(x) find(cell2mat(iter(x, :))), 1:Niter, 'un', 0); % 中间变量
    iter1 = iter(cellfun(@length, temp) == 1, :); % 所有线的序数组
    iter2 = iter(cellfun(@length, temp) == 2, :); % 所有两两相交的组合
    K = cell(size(iter2, 1), 2); % 预分配
    for k = 1:size(iter2, 1) % 一次循环替代嵌套
        locs1 = find(cell2mat(iter2(k, :))); % 所选平台
        theta = cellfun(@(x, y)x(y), angM(locs1), iter2(k, locs1), 'un', 0); % 读取角度
        theta = cell2mat(theta)';
        x1 = node1(locs1, 1);
        y1 = node1(locs1, 2);
        A = [-tand(theta), ones(2, 1)];
        B = y1 - x1 .* tand(theta);
        X = (A' * A) \ A' * B; % 目标位置X=[x;y]
        K(k, :) = num2cell(X'); % 记录交点矩阵
    end
    iter22 = cat(2, iter2, K); % 所选平台测量序号+交点矩阵
    %%% Step2：计算与线L11 相交的所有交点
    iter41 = [];
    outIter33 = cell(1, nl(1));

    for ik = 1:nl(1)
        [seRow11, ~] = find(cell2mat(iter22(:, 1)) == ik); % 寻找第一个平台第一个测量参与的组合数
        iter32 = [];
        for jj = 1:nl(2)
            [seRow21, ~] = find(cell2mat(iter22(:, 1)) == ik & cell2mat(iter22(:, 2)) == jj); % 其中寻找第二个平台第jj个测量参与的组合
            [seRow20, ~] = find(cell2mat(iter22(:, 1)) == ik & cell2mat(iter22(:, 2)) == 0); % 其中寻找第二个平台不参与参与的组合
            point1 = cell2mat(iter22(seRow21, S+1:S+2)).';
            for ii = 1:length(seRow20)
                point2 = cell2mat(iter22(seRow20(ii), S+1:S+2)).';
                iter31(ii, 1:S) = [cell2mat(iter22(seRow21, 1:2)), cell2mat(iter22(seRow20(ii), 3:S))];
                iter31(ii, S+1) = norm(point2-point1);
            end
            iter32 = [iter32; iter31];
        end
        iter33 = iter32(iter32(:, S+1) < Dth, :); % 通过门限的组合
        % 没有通过门限的逐步放宽门限
        cirNum = 1;
        while isempty(iter33) && cirNum < 10
            iter33 = iter32(iter32(:, S+1) < Dth+50*cirNum, :);
            cirNum = cirNum + 1;
        end
        if ~isempty(iter33) && all(~(diff(iter33(:, 2)))) % 如果通过门限，且只有一个  第二个平台的测量关联
            iter34 = iter33; %
        elseif ~isempty(iter33) && ~all(~(diff(iter33(:, 2)))) % 如果通过门限，且只有多个  第二个平台的测量关联
            outIter33{ik} = iter33; % 保留竞争的筛选
            % 除了距离筛选 还要考虑是否竞争
            [~, minLocs] = min(iter33(:, S+1));
            iter34 = iter33(all(iter33(:, 1:2) == iter33(minLocs, 1:2), 2), :);
        end

        % 合并组合完成粗关联
        iter41(ik, 1:2) = iter34(1, 1:2);
        for jj = 3:S
            if length(find(iter34(:, jj))) == 1
                % 在平台jj上只关联一个目标的组合，即正常关联组合
                iter41(ik, jj) = iter34(find(iter34(:, jj)), jj);
            elseif length(find(iter34(:, jj))) == 0 % 没通过门限
                iter41(ik, jj) = 0;
            else % 关联两个需要细关联
                %                     error('关联需要细关联')
                outIter33{ik} = iter33; % 保留竞争的筛选
                [~, minLocs] = min(iter33(:, S+1));
                iter34 = iter33(all(iter33(:, 1:2) == iter33(minLocs, 1:2), 2), :);
                iter41(ik, jj) = iter34(find(iter34(:, jj)), jj);
            end
        end
    end
    iter42 = iter41;
    iter43 = arrayfun(@(s) all(~(diff(iter41(:, s)))), 1:S); % iter41出现相同测测量序号
    if find(all(iter41(:, iter43) ~= 0)) % 测量序号还不是0 即出现争抢测量的情况，需要处理
        for ik = 1:nl(1)
            if ~isempty(outIter33{ik})
                for i = 1:nl(1)
                    if find(all(outIter33{ik}(:, 1:2) == iter41(i, 1:2), 2)) % 找到相同的
                        iter333 = outIter33{ik}(~all(outIter33{ik}(:, 1:2) == iter41(i, 1:2), 2), :);
                        [~, minLocs] = min(iter333(:, S+1));
                        iter42(i, 1:2) = iter333(1, 1:2);
                        for jj = 3:S
                            if length(find(iter333(:, jj))) == 1
                                % 在平台jj上只关联一个目标的组合，即正常关联组合
                                iter42(ik, jj) = iter333(find(iter333(:, jj)), jj);
                            elseif length(find(iter333(:, jj))) == 0 % 没通过门限
                                iter42(ik, jj) = 0;
                            else % 关联两个需要细关联
                                error('关联需要细关联')
                            end
                        end
                    end
                end
            end
        end
    end
    [outLocX, outLocY] = deal(zeros(nl(1), 1));
    for i = 1:size(iter42, 1)
        inputZt = arrayfun(@(s) Zt(iter42(i, s), s), find(iter42(i, 1:S)));
        node2 = node1(find(iter42(i, 1:S)), :);
        [outLocX(i), outLocY(i)] = CA(inputZt, node2, Dth);
    end
    for i = 1:size(iter42, 1)
        for ii = 1:size(inputX, 1)
            d(i, ii) = norm([outLocX(i) - inputX, outLocY(i) - inputY]);
        end
    end
    if d(1, 1) <= Dth
        EstX = outLocX;
        EstY = outLocY;
        locs = iter42';
    else
        EstX = fliplr(outLocX);
        EstY = fliplr(outLocY);
        locs = fliplr(iter42');
    end

else
    [EstX, EstY] = deal(nan(size(Zt, 1), 1));
    locs = nan(size(Zt'));
end
end

%% TDOA/AOA

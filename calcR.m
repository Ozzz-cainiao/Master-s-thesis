%% 使用分治贪心关联获取线序号 然后获取时延关系解算
% 需要传入时延和方位信息

%% 传入参数：角度，平台数目，平台位置，目标数 当前时刻  观测周期
% arrR 5000*3 cell 时间 目标 平台数
function [outTimeM, choose] = calcR(timeR ,arrR, pNum, node, num, t_obs, T)

var2d = 1.5^2;
var2 = var2d * (pi / 180)^2;
c = 1500;

%% ==========================分治贪心关联===============================
dPonit = 10;
PD = 0.99; %0.9        	% 检测概率2~3个目标时是0.99，4~5个目标时是0.9
Fai = 2 * pi;
M = 3; % 选取最少线点数
Q = 10; % 列表最大长度
I = 3; % 并行次优条数
[outLoctionCAX, outLoctionCAY] = deal(nan(num, length(t_obs)));
outAngM = cell(num, length(t_obs));
outTimeM = cell(num, length(t_obs));
choose = cell(length(t_obs), 1);
for iii = 1:length(t_obs)
    disp(['正在处理', num2str(iii)])
    angM1 = arrR(iii, :);
    timeM1 = timeR(iii, :);
    % 这里angM1可能有nan值
    %     ns = cellfun(@(x)size(x, 1), angM1); % 每个平台的测向线个数
    ns = cellfun('prodofsize', angM1);

    % 这里根据目标个数和平台个数进行区分 两目标两平台无解
    if num == 2 && pNum == 2
        % todo 两目标两平台怎么做
        if all(ns ~= 1) % 无解只剩下两个平台有测量，不能进行关联，只能对一个目标进行处理
            [outLocX, outLocY] = deal(nan);

        else % 只有一个测量，直接进行定位
            Z = cell2mat(angM1(locs_notnan));
            [outLocX, outLocY] = LSM(Z, node);
        end
    else
        % 每个平台的测向线个数
%                 ns = arrayfun(@(x) size(angM1{x}(~isnan(angM1{x})), 1), 1:pNum);
        if all(ns == 1)
            %             disp("all(ns == 1)")
            %             Z = arrayfun(@(x) angM1{x}(~isnan(angM1{x})), 1:pNum, 'un');
            Z = cell2mat(angM1);
            %             Z = arrayfun(@(x) angM1{x}(~isnan(angM1{x})), 1:pNum, 'un');
            [outLocX, outLocY] = LSM(Z, node);
        else
            % Zt是1*numOfPlatform的cell数组，每个cell中是它的测向线
            fprintf("ns= %d\n", ns);
            if all(ns == 0)
                continue;
            end
            Z = arrayfun(@(x) {angM1{x}(~isnan(angM1{x}))}, 1:pNum, 'un', 0);
            outDCGT = dcgt(Z, node, [var2, PD, Fai], [M, Q, I]);
            outLocs = outDCGT(:, 1:length(Z)); % 输出的线序号结果
            choose{iii} = outLocs;
            outLocX = outDCGT(:, length(Z)+1);
            outLocY = outDCGT(:, length(Z)+2);
        end
    end

    %% 10个点以后，实现跟原有轨迹的跟踪
    % inputX:2*10double
    if iii > dPonit
        inputX = outLoctionCAX(:, iii-10:iii-1);
        inputY = outLoctionCAY(:, iii-10:iii-1);
    elseif iii > 1
        inputX = outLoctionCAX(:, 1:iii-1);
        inputY = outLoctionCAY(:, 1:iii-1);
    else
        [inputX, inputY] = deal(nan(num, 1));
    end

    %% 计算解算点与已有的轨迹之间的距离
    di = zeros(size(outLocX, 1), size(inputX, 1)); % 与既有轨迹的距离的预分配
    for i = 1:size(outLocX, 1)
        for ii = 1:size(inputX, 1)
            d1 = zeros(1, size(inputX, 2));
            for iiii = 1:size(inputX, 2)
                d1(iiii) = norm([outLocX(i) - inputX(ii, iiii), outLocY(i) - inputY(ii, iiii)]);
            end
            di(i, ii) = mean(d1(~isnan(d1)));
        end
    end


    di(isnan(di)) = 1e8;
    % 匈牙利算法，二分图匹配
    if size(di, 1) > size(di, 2) % HungarianAlgorithm.m只能对列数≥行数的正确关联
        [~, zeta] = HungarianAlgorithm(di');
        zeta = zeta';
    else
        [~, zeta] = HungarianAlgorithm(di);
    end

    %
    EstX = outLocX.' * zeta;
    EstY = outLocY.' * zeta;
    EstLocs = outLocs.' * zeta;
    EstX(EstX == 0) = nan;
    EstY(EstY == 0) = nan;
    EstLocs(:, all(EstLocs == 0, 1)) = nan;
    EstLocs = EstLocs.';
    if length(EstX) == 1
        outLoctionCAX(1, iii) = EstX(1)';
        outLoctionCAY(1, iii) = EstY(1)';
        outLoctionCAX(2:num, iii) = nan;
        outLoctionCAY(2:num, iii) = nan;
        %         if all(isnan(EstLocs(i, :)))
        %             outAngM{1, iii} = [];
        %             outTimeM{i, iii} = [];
        %         else
        %             for s = 1:pNum
        %                 if EstLocs(i, s) ~= 0
        %                     outAngM{i ,iii}(s) = angM1{s}(EstLocs(1, s)); % 提取出的定位角度组合
        %                     outTimeM{i ,iii}(s) = timeM1{s}(EstLocs(1, s)); %
        %                 else
        %                     outAngM{i ,iii}(s) = nan;
        %                     outTimeM{i ,iii}(s) = nan; %
        %                 end
        %             end
        %         end
        outAngM{1, iii} = arrayfun(@(s) angM1{s}(EstLocs(1, s)), 1:pNum);
        outTimeM{1, iii} = arrayfun(@(s) timeM1{s}(EstLocs(1, s)), 1:pNum);
    else
        outLoctionCAX(1:num, iii) = EstX';
        outLoctionCAY(1:num, iii) = EstY';
        for i = 1:num
            if all(isnan(EstLocs(i, :))) % 全为nan，即唯有测量
                outAngM{i ,iii} = [];
                outTimeM{i, iii} = [];
            else
                for s = 1:pNum
                    if EstLocs(i, s) ~= 0
                        outAngM{i ,iii}(s) = angM1{s}(EstLocs(i, s)); % 提取出的定位角度组合
                        outTimeM{i ,iii}(s) = timeM1{s}(EstLocs(i, s)); %
                    else
                        outAngM{i ,iii}(s) = nan;
                        outTimeM{i ,iii}(s) = nan; %
                    end
                end
            end
        end
    end
end
% save('midResult.mat','outLoctionCAX','outLoctionCAY','outAngM')
% load midResult.mat

%% ==========================分治贪心关联+时空关联===============================
d_min = 2; % 时空关联解算阈值
DeTh = 500; % 迭代次数阈值
loc = cell(length(t_obs), num);
[outLoctionSPCX, outLoctionSPCY] = deal(nan(num, length(t_obs)));
for iii = 1:length(t_obs)
    disp(['正在处理', num2str(iii)])
    % 进行时空关联
    for i = 1:num
        if isnan(outLoctionCAX(i, iii))
            outLoctionSPCX(i, iii) = outLoctionCAX(i, iii);
            outLoctionSPCY(i, iii) = outLoctionCAY(i, iii);
        else
            d = inf;
            iikk = 1;
            midOldCAX = outLoctionCAX(i, iii);
            midOldCAY = outLoctionCAY(i, iii);
            %             while d > d_min && ~isnan(outLoctionCAX(i))
            while d > d_min && iikk < DeTh
                x_e = midOldCAX - node(:, 1); % 估计位置与观测站横轴距离
                y_e = midOldCAY - node(:, 2); % 估计位置与观测站纵轴距离
                r_e = sqrt(x_e.^2+y_e.^2); % 估计位置与观测站之间的距离
                t_e = r_e / c;
                % 当前位置，针对的方位数据是在几个帧后获得的
                % 这里的问题是为什么选定平台1 它不是最早有数据的 会导致有帧号找不到
                t_ed = round(t_e-t_e(1), 1); % 时延差 四舍五入到小数点后最近的1位数
                % 参考王静飞论文  选取位置到平台最远的那个作为基准
%                 t_max = max(t_e);
%                 t_ed = t_max - t_e; % 时延差 四舍五入到小数点后最近的1位数
%                 loc_ed = iii - round(t_ed/T); % 位置 这是绝对的帧号，不是相对的

                %  t_min = min(t_e);
                %  t_ed = round(t_e - t_min, 1); % 时延差 四舍五入到小数点后最近的1位数
                loc_ed = iii + floor(t_ed/T); % 位置 这是绝对的帧号，不是相对的
                % 只找到时延差向后的 还应该往前找
                if all(loc_ed > 0 & loc_ed < length(t_obs)) % 时空关联能够进行
                    angM2 = zeros(1, pNum);
                    for s = 1:pNum
                        % 如果该找的这个帧数据不存在，就向后找
                        N = 30; % 确定向后寻找的范围
                        if isempty(outAngM{i, loc_ed(s)})
                            ij = 1;
                            while ij < N
                                if (loc_ed(s) + ij < size(outAngM, i)) && ~isempty(outAngM{i, loc_ed(s)+ij})
                                    angM2(s) = outAngM{i, loc_ed(s)+ij}(s); % 取到新的时延的角度值
                                    break
                                else
                                    ij = ij + 1;
                                end
                            end
                        else
                            angM2(s) = outAngM{i, loc_ed(s)}(s);
                        end
                    end

                    % 找到了之后开始计算新的位置
                    [midNewCAX, midNewCAY] = LSM(angM2, node);
                else
                    outLoctionSPCX(i, iii) = midOldCAX;
                    outLoctionSPCY(i, iii) = midOldCAY;
                    break;
                end
                d = sqrt((midOldCAX - midNewCAX)^2+(midOldCAY - midNewCAY)^2);
                midOldCAX = midNewCAX;
                midOldCAY = midNewCAY;
                iikk = iikk + 1;
            end
            % 这里的有问题
            if exist('loc_ed', 'var') == 1
                % 找到对应的索引
                loc{iii, i} = loc_ed; % 找这个附近2帧范围内的，提取时延，使用时延计算
            end
            clear loc_ed;
            outLoctionSPCX(i, iii) = midOldCAX;
            outLoctionSPCY(i, iii) = midOldCAY;
        end
    end
end

%% 加上时延数据时空关联  先在所有数据都能接收到的基础上进行 即第7秒，所有平台都接收到了目标的信息
% loc中的时延信息，是相对可靠的，基本只有0-3帧的误差，同时测得的绝对时延变化不大。
% 下面实现根据方位关联的反馈信息实现时延定位
% 时延信息先不加误差
% 计算loc中的帧号的±2的时间
% 使用什么算法？
[TA_resX, TA_resY] = deal(nan(length(t_obs), num));
TA_res = cell(length(t_obs), num);
[TDOA_resX, TDOA_resY] = deal(nan(length(t_obs), num));
TDOA_res = cell(length(t_obs), num);
% 一维时间
for iii = 1:length(t_obs)
    % 二维目标
    for i = 1:num
        % 1. 先提取出loc中的帧号
        frame = loc{iii, i};
        if isempty(frame)
            continue;
        end
        frame(frame <= 1) = 1;
        frame(frame >= length(t_obs)) = length(t_obs);

        c_angle = nan(size(frame));
        c_time = nan(size(frame));

        % 从outAngM和outTimeM中提取对应的方位和时延数据
        for j = 1:size(frame, 1)
            c_angle(j) = outAngM{i, frame(j)}(j);
            c_time(j) = outTimeM{i, frame(j)}(j);
        end

        % 提取平台位置

        node;

        % 使用TA进行计算?
        [res, ~] = TA1(c_time, c_angle, node);
        TA_res{iii, i} = res;
        TA_resX(iii, i) = res(1);
        TA_resY(iii, i) = res(2);
        % 使用纯方位计算的位置
        fprintf(" 使用TA计算的结果为：%f, %f\n", res);
        fprintf("使用AOA计算的结果为：%f, %f \n", [outLoctionSPCX(i, iii), outLoctionSPCY(i, iii)]);
        fprintf("这两个点之间的距离为： %f\n", pdist([res; [outLoctionSPCX(i, iii), outLoctionSPCY(i, iii)]]));

        % 使用TDOA进行计算
        nodeT = [node, zeros(size(node, 1), 1)];
        [res, ~] = TDOA(c_time', nodeT, 4);
        TDOA_res{iii, i} = res;
        TDOA_resX(iii, i) = res(1);
        TDOA_resY(iii, i) = res(2);

    end
end


fig1 = figure('Units', 'centimeters', 'Position', [10, 10, 20, 11.24 / 15 * 15]);
figure(fig1)
hold on

axis([0, 10e3, 0, 10e3])
title("分治贪心关联")
for ii = 1:num
    plot(outLoctionCAX(ii, :), outLoctionCAY(ii, :), '.');
end
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% legend('目标1', '目标2', '目标3', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)


fig2 = figure('Units', 'centimeters', 'Position', [10, 10, 20, 11.24 / 15 * 15]);
figure(fig2)
hold on
for ii = 1:num
    plot(outLoctionSPCX(ii, :), outLoctionSPCY(ii, :), '.')
end
axis([0, 10e3, 0, 10e3])
title("分治贪心关联+时空关联")
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% legend('目标1', '目标2', '目标3', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)


fig3 = figure('Units', 'centimeters', 'Position', [10, 10, 20, 11.24 / 15 * 15]);
figure(fig3)
hold on
for ii = 1:num
    plot(TA_resX(:, ii), TA_resY(:, ii), '.')
end
axis([0, 10e3, 0, 10e3])
title("TA算法，加上了帧号")
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% legend('目标1', '目标2', '目标3', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)


fig4 = figure('Units', 'centimeters', 'Position', [10, 10, 20, 11.24 / 15 * 15]);
figure(fig4)
hold on
for ii = 1:num
    plot(TDOA_resX(:, ii), TDOA_resY(:, ii), '.')
end
axis([0, 10e3, 0, 10e3])
title("TDOA算法")
scatter(node(:, 1), node(:, 2), 'b^', 'filled', 'LineWidth', 0.5, 'SizeData', 100);
legend('目标1', '目标2', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
% legend('目标1', '目标2', '目标3', '观测站', 'Location', 'eastoutside', 'FontSize', 12)
hold off
set(gca, 'Box', 'on')
xlabel('东向坐标/m', 'FontSize', 12)
ylabel('北向坐标/m', 'FontSize', 12)
end

%% ==========================子函数===============================
%dcgt 基于分治贪心思想的联合多站目标关联定位
function varargout = dcgt(varargin)

%   此处显示详细说明
% INPUTS:
%   varargin{1}   - 测量元胞
%   varargin{2}   - 节点位置
%   varargin{3}   - 分布参数
%   varargin{4}   - 处理参数
%
% OUTPUTS:
%   varargout{1}  - 关联组合+位置
% 判断Z是否为空

warning('off')
Z = varargin{1};
% % 去除空的内部 cell
% x = cellfun(@(x) ~isempty(x), Z);
% Z = Z(~cellfun('isempty', x));
node = varargin{2};
var2 = varargin{3}(1); % 角度测量方差单位rad
PD = varargin{3}(2); % 检测概率
Fai = varargin{3}(3);
M = varargin{4}(1); % 选取最少线点数
Q = varargin{4}(2); % 列表最大长度
I = varargin{4}(3); % 并行次优条数
pNum = size(node, 1); % 节点数
angM = arrayfun(@(x)cat(2, cell2mat(x{1, 1}(:, 1))), Z, 'un', 0); % 角度特征元胞组

%%% ==========================基于分治思想的最优交点集合选取===============================
%%% Step1：生成交点
% angM(:)：将矩阵angM展开成一个列向量，其中每个元素是原矩阵中的一个cell
% cellfun(@length, ...)：cellfun函数用于对cell数组的每个元素应用指定的函数。
nl = cellfun(@length, angM(:)); % 各个平台测量数量 2 2 0
% 计算非零行的个数
nonZeroRowCount = nnz(nl ~= 0);
disp(['非零行的个数为：', num2str(nonZeroRowCount)]);
if nonZeroRowCount < 2
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
subs = arrayfun(@(x)0:x, nl, 'un', 0); % 生成序号 从0开始是为了漏检的显示
iter = cell(pNum, 1);
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
    x1 = node(locs1, 1);
    y1 = node(locs1, 2);
    A = [-tand(theta), ones(2, 1)];
    B = y1 - x1 .* tand(theta);
    X = (A' * A) \ A' * B; % 目标位置X=[x;y]
    K(k, :) = num2cell(X'); % 记录交点矩阵
end
iter22 = cat(2, iter2, K); % 所选平台测量序号+交点矩阵

%%% Step2：对每条线选取最短的M条线段
W = {};
for k = 1:size(iter1, 1)
    locs21 = find(cell2mat(iter1(k, :))); % 所选平台序号
    locsM = iter1{k, locs21}; % 所选测量序号
    Num22 = find(cell2mat(iter22(:, locs21)) == iter1{k, locs21}); % 在iter22筛选出来的序号，选取的测向线
    point = iter22(Num22, :); % 筛选出来的测量序号+位置
    kk = 1;
    kknum = 1;
    if size(point, 1) > 1
        locs22 = cell(sum(1:size(point, 1)-1), 3); % 预分配
        while kk < size(point, 1)
            point1 = cell2mat(point(kk, pNum+1:pNum+2)).';
            for ii = kk + 1:size(point, 1)
                point2 = cell2mat(point(ii, pNum+1:pNum+2)).';
                locs22{kknum, 1} = Num22(kk); % iter22中行序号--对应点
                locs22{kknum, 2} = Num22(ii); % iter22中行序号--对应点
                locs22{kknum, 3} = norm(point2-point1); % 解算点之间的距离
                kknum = kknum + 1;
            end
            kk = kk + 1;
        end
        [~, locs23] = sort(cell2mat(locs22(:, 3))); % 按升序排列后的序数
        if length(locs23) > M
            W = cat(1, W, locs22(locs23(1:M), :)); % W中三列分为iter22中行序号；iter22中行序号；距离
        else
            W = cat(1, W, locs22(locs23(1:length(locs23)), :)); % iter22中的组合x和y中间的距离
        end
    else

    end

end
%%% Step3：定位

Num3 = size(W, 1);
if Num3 == 0
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
Lamdad = cell(Num3, 2);
for k = 1:Num3
    locs31 = iter22(cell2mat(W(k, 1:2)), :); % locs31所选测量序号，和交点位置
    [locs32, locs33] = find(cell2mat(locs31(:, 1:pNum))); % locs32行序数->无意义 locs33列序数->所选平台
    if length(unique(locs33)) < 3
        locs35 = arrayfun(@(x, y) locs31(x, y), locs32, locs33); % locs35测量序数，与locs33配合读取测量
        theta = cellfun(@(x, y) angM{x}(y), num2cell(locs33), locs35, 'un', 0);
        theta = cell2mat(theta);
        Lamdad{k, 1} = cat(2, locs33, cell2mat(locs35), theta); % 平台序号;测量序号;测量值
        Lamdad{k, 2} = nan; % 似然
    else
        [locs33, locs34] = unique(locs33); % locs34独一无二数的平台数的序数， locs33列序数更新
        locs32 = locs32(locs34); % locs32行序数更新
        locs35 = arrayfun(@(x, y) locs31(x, y), locs32, locs33); % locs35测量序数，与locs33配合读取测量
        theta = cellfun(@(x, y) angM{x}(y), num2cell(locs33), locs35, 'un', 0);
        theta = cell2mat(theta);
        x3 = node(locs33, 1);
        y3 = node(locs33, 2);
        A = [-tand(theta), ones(3, 1)];
        B = y3 - x3 .* tand(theta);
        Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
        thetaE = atan2d(Xd(2)-y3, Xd(1)-x3); % 估计目标位置到平台角度
        %%% 换成对数似然
        c = zeros(1, pNum);
        for ii = 1:pNum
            if ismember(ii, locs33)
                detla_l = 1;
                thetaii = theta(ismember(locs33, ii));
                thetaEii = thetaE(ismember(locs33, ii));
                esis = AngelDeal(thetaii, thetaEii) / 180 * pi;
                c(ii) = detla_l * (-log(PD*Fai/sqrt(2*pi*var2)) + 0.5 * (esis / sqrt(var2))^2);
            else
                u = 0;
                c(ii) = (u - 1) * log(1-PD);
            end
        end
        Lamdad{k, 1} = cat(2, locs33, cell2mat(locs35), theta); % 平台序号;测量序号;测量值
        Lamdad{k, 2} = sum(c); % 似然
    end
end

%% ==========================基于贪心思想的量测集合合并===============================
% 下面这一句 Lamdad_W可能为空，因为Lamdad可能都是nan值
Lamdad_W = Lamdad(~isnan(cell2mat(Lamdad(:, 2))), :); % 排除一个组合有多个同平台测量
if isempty(Lamdad_W)
    varargout{1} = [nan(1, pNum), nan, nan];
    return;
end
Num4 = size(Lamdad_W, 1);
%%% 对获得组合按平台-测量序数分
Set = cell(1, pNum);
for ii = 1:pNum
    for jj = 1:nl(ii)
        locs41 = arrayfun(@(x) sum(Lamdad_W{x, 1}(:, 1) == ii & Lamdad_W{x, 1}(:, 2) == jj), 1:Num4, 'un', 0);
        Set{ii} = cat(1, Set{ii}, Lamdad_W(logical(cell2mat(locs41')), :));
    end
end
s = 2;
Z = Set{1, s-1};
W_Plus = {};
while s <= pNum
    Z1 = {};
    z = Set{1, s};
    NumZ = size(Z, 1);
    Numz = size(z, 1);
    R = zeros(NumZ, Numz);
    for ii = 1:NumZ
        for jj = 1:Numz
            Mea = cat(1, Z{ii}, z{jj}); % 在下方拼接
            locs51 = arrayfun(@(x) find(Mea(:, 1) == x), 1:pNum, 'un', 0); % 平台1：S对应再矩阵的序号
            locs52 = cellfun(@length, locs51); % 有多余1个测量的序号
            Mea2 = cellfun(@(x) Mea(x, :), locs51(locs52 > 1), 'un', 0); % 有多余1个测量的Mea
            locs53 = arrayfun(@(x) unique(Mea2{1, x}(:, 2)), 1:length(Mea2), 'un', 0); % 有多余1个测量的平台中 重复测量数量
            locs54 = cellfun(@length, locs53);
            if isempty(find(locs54 > 1, 1))
                [locs55, locs56] = unique(Mea(:, 1));
                if length(locs55) == pNum % 遍历所有组合->输出
                    if isempty(W_Plus) % 第一个值
                        W_Plus = cat(1, W_Plus, {Mea(locs56, :)});
                        R(ii, jj) = 1; % 成功融合且没有输出过的标记为1
                    else % 已有输出集
                        for kk = 1:length(W_Plus) % 对输出集遍历
                            if sum(W_Plus{kk} == Mea(locs56, :), 'all') == numel(Mea(locs56, :)) % 如果和输出集相等
                                R(ii, jj) = 2; % 已成功融合但输出过的标记为2
                            end
                        end
                        if R(ii, jj) ~= 2
                            W_Plus = cat(1, W_Plus, {Mea(locs56, :)});
                            R(ii, jj) = 1; % 成功融合且没有输出过的标记为1
                        end
                    end
                else
                    if isempty(Z1) % 第一个值
                        Z1 = cat(1, Z1, {Mea(locs56, :)});
                        R(ii, jj) = 3; % 成功融合但未遍历所有平台的标记为3
                    else
                        for kk = 1:length(Z1) % 对输出集遍历
                            %                             if sum(Z1{kk}==Mea(locs56,:),'all') == numel(Mea(locs56,:))% 如果和输出集相等
                            %%% 以下判断条件20230919修改，此时s=5时出现错误
                            [Lia, ~] = ismember(Z1{kk}(:, 1:2), Mea(locs56, 1:2), 'rows');

                            if length(find(Lia)) == size(Mea(locs56, :), 1) % 如果Mea已经在输出集里面
                                R(ii, jj) = 4; % 已成功融合但未遍历所有平台的标记为4
                            end
                        end
                        if R(ii, jj) ~= 4
                            Z1 = cat(1, Z1, {Mea(locs56, :)});
                            R(ii, jj) = 3; % 成功融合但未遍历所有平台的标记为3
                        end
                    end

                end
            end
        end
    end
    %%%更新关联下一个平台
    [locs61, locs62] = find(R);
    numZ = 1:NumZ;
    if sum(ismember(numZ, locs61)) ~= NumZ % 未关联的行
        locs63 = numZ(~ismember(numZ, locs61)); % 未更新的行号
        Z1 = cat(1, Z1, Z{locs63, 1});
    end
    numz = 1:Numz;
    if sum(ismember(numz, locs62)) ~= Numz % 未关联的列
        locs64 = numz(~ismember(numz, locs62)); % 未关联的列号
        Z1 = cat(1, Z1, z{locs64, 1});
    end
    Z = Z1;
    s = s + 1;
end
W_Plus = cat(1, W_Plus, Z);

%%% 计算位置和关联似然概率并保留最大的Q个组合
Lamda7 = cell(length(W_Plus), 1);
for k = 1:length(W_Plus)
    Wk = W_Plus{k};
    theta = Wk(:, 3);
    x6 = node(Wk(:, 1), 1);
    y6 = node(Wk(:, 1), 2);
    A = [-tand(theta), ones(size(Wk, 1), 1)];
    B = y6 - x6 .* tand(theta);
    Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
    thetaE = atan2d(Xd(2)-y6, Xd(1)-x6); % 估计目标位置到平台角度
    p = arrayfun(@(x, y) DistributedDeal(x/180*pi, y/180*pi, var2), theta, thetaE);
    Lamda7{k} = prod(p);
    %%% 换成对数似然
    c = zeros(1, pNum);
    for ii = 1:pNum
        if ismember(ii, Wk(:, 1))
            detla_l = 1;
            thetaii = theta(ismember(Wk(:, 1), ii));
            thetaEii = thetaE(ismember(Wk(:, 1), ii));
            esis = AngelDeal(thetaii, thetaEii) / 180 * pi;
            c(ii) = detla_l * (-log(PD*Fai/sqrt(2*pi*var2)) + 0.5 * (esis / sqrt(var2))^2);
        else
            u = 0;
            c(ii) = (u - 1) * log(1-PD);
        end
    end
    Lamda7{k} = sum(c);
end
W_Plus = cat(2, W_Plus, Lamda7);
[~, locs71] = sort(cell2mat(Lamda7), 'descend');
[~, locs71] = sort(cell2mat(Lamda7));
if length(locs71) > Q
    W_Plus7 = W_Plus(locs71(1:Q), :);
else
    W_Plus7 = W_Plus(locs71, :);
end
%%% ==========================基于贪心思想的量测总集划分===============================
%%%寻找不互斥的输出测量组
R8 = zeros(size(W_Plus7, 1));
for ii = 1:size(W_Plus7, 1)
    W81 = W_Plus7{ii, 1}(:, 1:2);
    for jj = 1:size(W_Plus7, 1)
        W82 = W_Plus7{jj, 1}(:, 1:2);
        locs81 = cat(1, W81, W82); % 对两个集合拼接
        locs82 = arrayfun(@(x) locs81(locs81(:, 1) == x, 2), unique(locs81(:, 1)), 'un', 0); % 平台测量数
        locs83 = cellfun(@(x) length(unique(x)) == length(x), locs82, 'un', 0); % 平台测量互斥性，如果1不互斥；如果0则互斥
        if length(find(cell2mat(locs83))) == length(locs82) % 如果不互斥的平台数==有测量平台数
            R8(ii, jj) = 1; % 不互斥的标记为1
        end
    end
end
% 判断W_Plus7是否为空
q = 2;
Psi = {{}; W_Plus7{q-1, 1}}; % 索引容易超出数组边界：W_Plus7可能为空
L = [0; 1];
P = [0; W_Plus7{q-1, 2}];
while q <= size(W_Plus7, 1)
    NumPsi = size(Psi, 1);
    Psi = cat(2, repmat(Psi, 2, 1), cat(1, cell(NumPsi, 1), repmat(W_Plus7(q, 1), NumPsi, 1))); % 兼容组合
    L = cat(2, repmat(L, 2, 1), cat(1, zeros(NumPsi, 1), ones(NumPsi, 1))); % 兼容组合索引
    P = sum(cat(2, repmat(P, 2, 1), cat(1, zeros(NumPsi, 1), W_Plus7{q, 2}*ones(NumPsi, 1))), 2); % 总似然
    locsdet = [];
    for ii = NumPsi + 1:NumPsi * 2
        Lq = L(ii, :);
        locs91 = find(Lq == 1)';
        if sum(Lq == 1) == 2 % 存在两个关联量则检查是否能同时输出
            if R8(locs91(1), locs91(2)) ~= 1 % 不可以同时输出
                locsdet = cat(1, locsdet, ii);
            end
        elseif sum(Lq == 1) > 2
            locs92 = [locs91(1:end-1), locs91(end) * ones(sum(Lq == 1)-1, 1)]; % 待处理的兼容组合
            if sum(arrayfun(@(x, y) R8(x, y) == 1, locs92(:, 1), locs92(:, 2))) ~= sum(Lq == 1) - 1 % 没有和所有兼容
                locsdet = cat(1, locsdet, ii);
            end
        end
    end
    Psi = Psi(~ismember(1:end, locsdet), :);
    L = L(~ismember(1:end, locsdet), :);
    P = P(~ismember(1:end, locsdet), :);
    if length(P) > I
        [~, locs93] = sort(P, 'descend');
        [~, locs93] = sort(P);
        Psi = Psi(locs93(1:I), :);
        L = L(locs93(1:I), :);
        P = P(locs93(1:I), :);
    end
    q = q + 1;
end

%%% ==========================求解位置和关联组合===============================
%%% 以下判断条件20230919修改，此时出现Psi(1,:)={}情况
if isempty(Psi{1, 1}) && ~isempty(Psi{2, 1})
    Psi_opt = Psi(2, :);
else
    Psi_opt = Psi(1, :);
end
% Psi_opt = Psi(1,:);
Xout = [];
Pos = zeros(length(Psi_opt), pNum);
for ii = 1:length(Psi_opt)
    if ~isempty(Psi_opt{ii})
        %%% 求解位置
        theta = Psi_opt{ii}(:, 3);
        x9 = node(Psi_opt{ii}(:, 1), 1);
        y9 = node(Psi_opt{ii}(:, 1), 2);
        A = [-tand(theta), ones(size(Psi_opt{ii}, 1), 1)];
        B = y9 - x9 .* tand(theta);
        Xd = (A' * A) \ A' * B; % 估计目标位置Xd=[x;y]
        Xout = cat(2, Xout, Xd);
        %%% 求解组合数
        Pos(ii, Psi_opt{ii}(:, 1)) = Psi_opt{ii}(:, 2);
    else
        Pos(ii, :) = nan;
    end
end
%%% ==========================求解关联组合===============================
Pos1 = Pos(~isnan(Pos));
Pos2 = reshape(Pos1, [], pNum);

%%% ==========================输出===============================

varargout{1} = [Pos2, Xout'];

end

%% 最小二乘法定位
function [EstX, EstY] = LSM(Zt, node)
theta = Zt;
theta = theta(~isnan(Zt))';
x1 = node(~isnan(Zt), 1);
y1 = node(~isnan(Zt), 2);
A = [-tand(theta), ones(length(x1), 1)];
B = y1 - x1 .* tand(theta);
X = (A' * A) \ A' * B; % 目标位置X=[x;y]
if isempty(X)
    EstX = inf;
    EstY = inf;
else
    EstX = X(1);
    EstY = X(2);
end
end

%% 角度处理
function A_X_Y = AngelDeal(X, Y)
A_XY = mod(X-Y, 360);
A_YX = mod(Y-X, 360);
A_X_Y = min(A_XY, A_YX);
end

%% 子函数--角度变成概率函数
function P = DistributedDeal(theta1, theta2, thetas2)
if theta1 - theta2 < -pi
    P = normpdf(theta1, theta2-2*pi, thetas2);
elseif theta1 - theta2 > pi
    P = normpdf(theta1, theta2+2*pi, thetas2);
else
    P = normpdf(theta1, theta2, thetas2);
end
end

%%
function [cost, x] = HungarianAlgorithm(C)
% The implementation of the Hungarian algorithm.
% Reference:
% https://csclab.murraystate.edu/~bob.pilgrim/445/munkres_old.html
% C: cost matrix
[n, m] = size(C);
k = min(n, m);
starMatrix = zeros(n, m);
primedMatrix = zeros(n, m);
coveredRow = zeros(n, 1);
coveredColumn = zeros(1, m);
% Step1: For each row of the matrix, find the smallest element and subtract
% it from every element in its row. Go to Step 2.
C1 = zeros(n, m);
parfor i = 1:n
    tempV = min(C(i, :));
    C1(i, :) = C(i, :) - tempV;
end
% Step2: Find a zero (Z) in the resulting matrix. If there is no starred
% zero in its row or column, star (Z). Repeat for each element in the
% matrix. Go to Step 3.
for i = 1:n
    for j = 1:m
        if C1(i, j) == 0 && (sum(starMatrix(i, :)) == 0) && (sum(starMatrix(:, j)) == 0)
            starMatrix(i, j) = 1;
        end
    end
end
flag = zeros(1, 5);
flag(1) = 1;
while flag(5) == 0
    % Step3: Cover each column containing a starred zero. If K columns are
    % covered the starred zeros describe a complete set of unique
    % assignments. In this case, Go to DOWN, otherwise, Go to Step 4.
    if flag(1) == 1
        parfor j = 1:m
            temp = sum(starMatrix(:, j));
            if temp > 0
                coveredColumn(j) = 1;
            end
        end
        if sum(coveredColumn) == k
            flag = zeros(1, 5);
            flag(5) = 1;
        else
            flag = zeros(1, 5);
            flag(2) = 1;
        end
    end
    % Step4: Find a noncovered zero and prime it. If there is no starred
    % zero in the row containing this primed zero. Go to Step 5. Otherwise,
    % cover this row and uncover the column containing the starred zero.
    % Continue in this manner until there are no uncovered zeros left. Save
    % the smallest uncovered value and Go to Step 6.
    if flag(2) == 1
        tempC = C1 + ones(n, 1) * coveredColumn + coveredRow * ones(1, m);
        [idx1, idx2] = find(tempC == 0);
        for j = 1:length(idx1)
            primeIdx1 = idx1(j);
            primeIdx2 = idx2(j);
            primedMatrix(primeIdx1, primeIdx2) = 1;
            if sum(starMatrix(primeIdx1, :)) == 0
                % Go to Step 5:
                flag = zeros(1, 5);
                flag(3) = 1;
                break;
            else
                coveredRow(primeIdx1) = 1;
                idx = find(starMatrix(primeIdx1, :) == 1);
                coveredColumn(idx) = 0;
            end
        end
        if flag(3) == 0
            m1 = coveredRow * ones(1, m) * inf;
            m1(find(isnan(m1) == 1)) = 0;
            m2 = ones(n, 1) * coveredColumn * inf;
            m2(find(isnan(m2) == 1)) = 0;
            tempC = C1 + m1 + m2;
            minValue = min(min(tempC));
            [smallestIdx1, smallestIdx2] = find(tempC == minValue);
            smallestIdx1 = smallestIdx1(1);
            smallestIdx2 = smallestIdx2(1);
            % Go to Step 6:
            flag = zeros(1, 5);
            flag(4) = 1;
        end
    end
    if flag(3) == 1
        % Step5: Construct a series of alternating primed and starred zeros
        % as follows. Let Z0 represent the uncovered primed zero found in
        % Step 4. Let Z1 denote the starred zero in the column of Z0 (if
        % any). Let Z2 denote the primed zero in the row of Z1 (there will
        % always be one). Continue until the series terminates at a primed
        % zero that has no starred zero in its column. Unstar each starred
        % zero of the series, star each primed zero of the series, erase
        % all primes and uncover every line in the matrix. Return to Step
        % 3.
        tempFlag = true;
        Z = [primeIdx1, primeIdx2];
        while tempFlag
            starIdx1 = find(starMatrix(:, primeIdx2) == 1);
            starMatrix(primeIdx1, primeIdx2) = 1;
            if isempty(starIdx1)
                tempFlag = false;
            else
                tempZ = [starIdx1(1), primeIdx2];
                Z = [Z; tempZ];
                starMatrix(tempZ(1), tempZ(2)) = 0;
                primeIdx1 = tempZ(1);
                primeIdx2 = find(primedMatrix(primeIdx1, :) == 1);
                primeIdx2 = primeIdx2(1);
                tempZ = [primeIdx1, primeIdx2];
                Z = [Z; tempZ];
            end
        end
        primedMatrix = zeros(n, m);
        coveredRow = zeros(n, 1);
        coveredColumn = zeros(1, m);
        % Return to Step 3:
        flag = zeros(1, 5);
        flag(1) = 1;
    end
    if flag(4) == 1
        % Step6: Add the value found in Step 4 to every element of each
        % covered row, and subtract it from every element of each uncovered
        % column. Return to Step 4 without altering any stars, primes, or
        % covered lines.
        C1 = C1 + coveredRow * ones(1, m) * minValue;
        uncoveredColumn = ones(1, m) - coveredColumn;
        C1 = C1 - ones(n, 1) * uncoveredColumn * minValue;
        % Return to Step 4:
        flag = zeros(1, 5);
        flag(2) = 1;
    end
end
% DONE: Assignment pairs are indicated by the positions of the starred
% zeros in the cost matrix. If C(i,j) is a starred zeros, then the element
% associated with row i is assigned to the element associated with column
% j.
x = starMatrix;
cost = 0;
[idx1, idx2] = find(starMatrix == 1);
for j = 1:length(idx1)
    cost = cost + C(idx1(j), idx2(j));
end


end

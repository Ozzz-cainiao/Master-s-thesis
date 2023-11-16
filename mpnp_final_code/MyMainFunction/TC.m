%% 粗关联 + 时空关联
function [outLoctionSPCX, outLoctionSPCY] = TC(angR, node, t_obs, num, T)
c = 1500
Dth = 500; % 交点距离约束
% ==========================进行时空关联定位===============================
d_min = 5; % 时空关联解算阈值
DeTh = 10; % 迭代次数阈值
for iii = 1:length(t_obs)
    d = inf;
    if iii > 1
        inputX = outLoctionSPCX(:, iii-1); % 上一时刻的位置？
        inputY = outLoctionSPCY(:, iii-1);
    else
        [inputX, inputY] = deal(nan);
    end
    %{
        这段MATLAB代码的语法和作用如下：

        1. `cell2mat()` 函数是将一个单元格数组转换为普通的数值数组。
        2. `arrayfun()` 函数是将一个函数应用于数组的每个元素，并返回结果。
        3. `@(ii)` 是一个匿名函数，它接受一个参数 `ii`。
        4. `angR{ii}(:,iii)` 是一个索引操作，它从 `angR` 单元格数组中的第 `ii` 个元素中提取第 `iii` 列的数据。
        5. `1:size(node,1)` 创建一个从1到 `size(node,1)` 的整数数组，这里的 `size(node,1)` 是 `node` 数组的行数。
        6. `'un',0` 是 `arrayfun()` 函数的可选参数，表示输出的结果是一个普通的数组。
        7. 最后，`Zt` 是将 `arrayfun()` 函数返回的结果通过 `cell2mat()` 函数转换为普通的数值数组。

        因此，该代码的作用是将 `angR` 单元格数组中每个元素的第 `iii` 列提取出来，并将所有提取的列合并为一个数值数组，赋值给变量 `Zt`。最后，`Zt` 的维度是2行4列的双精度数值数组。
    %}
    Zt = cell2mat(arrayfun(@(ii) angR{ii}(:, iii), 1:size(node, 1), 'un', 0)); % Zt = 2*4 double 是角度信息
    [midOldCAX1, midOldCAY1, locs{iii}] = CA_Plus(Zt, node, Dth, inputX, inputY); % node是节点的位置
    %     [outLoctionSPCX(1:num,iii),outLoctionSPCY(1:num,iii),locs{iii}]     = CA_Plus(Zt,node,Dth,inputX,inputY);
    % end
    for i = 1:num
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

                midZt = arrayfun(@(ii) angR{ii}(a_ed(ii), loc_ed(ii)), s(~isnan(a_ed) & a_ed ~= 0));
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

end
%% 
%%===================粗关联函数
function [EstX, EstY, locs] = CA_Plus(Zt, node, Dth, inputX, inputY)
% 整个表达式 length(find(all(isnan(Zt),1))) == 0 的作用是判断矩阵 Zt 中是否存在全为 NaN 的列。
% 如果结果为 true，表示 Zt 中不存在全为 NaN 的列；如果结果为 false，表示 Zt 中至少存在一个全为 NaN 的列。


if isempty(find(all(isnan(Zt), 1)))  %% 如果Zt全为nan就不执行下方的内容
    Zt = Zt(:, ~all(isnan(Zt)));
    node1 = node(~all(isnan(Zt)), :);
    S = size(node1, 1); % 节点数
    angM = mat2cell(Zt, size(Zt, 1), ones(1, S)); % 角度特征元胞组
    nl = cellfun(@length, angM(:)); % 各个平台测量数量
    subs = arrayfun(@(x)0:x, nl, 'un', 0); % 生成序号
    iter = cell(S, 1);
    [iter{end:-1:1}] = ndgrid(subs{end:-1:1}); % 生成网格bvxcbxcvb
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
                %     error('关联需要细关联')

                %---------------2目标可用 lishuo原始版本-------%
             
%                 outIter33{ik} = iter33; % 保留竞争的筛选
%                 [~, minLocs] = min(iter33(:, S+1));
%                 iter34 = iter33(all(iter33(:, 1:2) == iter33(minLocs, 1:2), 2), :);
%                 iter41(ik, jj) = iter34(find(iter34(:, jj)), jj);

                % -----------------3目标可用， 兼容2目标， ZLM修改版本-------------%

                outIter33{ik} = iter33; % 保留竞争的筛选
                [~, minLocs] = min(iter33(:, S+1)); % 找到最小的一行
                % iter34 将包含 iter33 矩阵中那些前两列与 iter33(minLocs, 1:2) 的前两列完全相等的行。
                iter34 = iter33(all(iter33(:, 1:2) == iter33(minLocs, 1:2), 2), :); % zlm注释
                iter34 = iter34(find(iter34(:, jj)),:);
                [~, minLocs] = min(iter34(:, S+1)); % 找到最小的一行

                if (~isempty(minLocs))
                    iter41(ik, jj) = iter34(minLocs, jj);
                end

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
%% 
%%% 关联定位中的约束
function [EstX,EstY] = CA(Zt,node,Dth)
%%% 测量不能超过20°约束 + 交点距离约束
iter        = nchoosek (1:length(Zt ),2);                                    % 组合数
for ii = 1:size(iter,1)         % 计算交点
    inputTheta = Zt(iter(ii,:));
    [poi1X(ii),poi1Y(ii)] = LSM(Zt(iter(ii,:)),node(iter(ii,:),:));
    if ( abs(inputTheta(1)-inputTheta(2)) >= 180-20  && abs(inputTheta(1)-inputTheta(2)) <= 180+20)...
            ||  abs(inputTheta(1)-inputTheta(2)) <= 20      % 出现测角在20°以内
        [poiX(ii),poiY(ii)] = deal(nan);
    else
        [poiX(ii),poiY(ii)] = deal(poi1X(ii),poi1Y(ii));
    end
end
MutExciterAngle  = iter(isnan(poiX),:);                    % 角度互斥测量
% 加nchoosek的判断
if (length(Zt) >= 3)
    iter3       = nchoosek (1:length(Zt),3);

%%% 计算交点距离
% 预分配chooseD
chooseD = inf(1);
for ii3 = 1:size(iter3,1)
    chosePoint = nchoosek (iter3(ii3,:),2);             % 选择交点的测量平台序数
    temp = nchoosek (1:size(chosePoint,1),2);           % 选择交点序数
    for ii2 = 1:size(temp,1)
        locsTemp = chosePoint(temp(ii2,:),:);
        for ii1 = 1: 2
            choosePoi1X(ii1) = poi1X(all(ismember(iter,locsTemp(ii1,:)),2) );
            choosePoi1Y(ii1) = poi1Y(all(ismember(iter,locsTemp(ii1,:)),2)   );
        end
        chooseD(ii3,ii2) = sqrt( ( choosePoi1X(1) - choosePoi1X(2) )^2 + (choosePoi1Y(1) - choosePoi1Y(2) )^2 );    % 四个三元量测组的交点距离
    end
end

MutExciterDis = iter3( sum(chooseD <= Dth,2) < 2 , : )  ;       % 距离互斥组   

if ~isempty(MutExciterAngle) || ~isempty( MutExciterDis)       % 存在角度互斥 || 存在距离互斥  则四个量测不能同时使用
    iter3 = iter3 ( ~all( ismember(iter3,MutExciterDis) ,2) ,:);    % 删除距离互斥组合
    retainIter3 = ones(size(iter3,1),1);             % 保留的序数初始化为1
    for ii3 = 1:size(iter3,1)
        for ii = 1:size(MutExciterAngle,1)
            if length(find(ismember(iter3(ii3,:),MutExciterAngle(ii,:)))) >1
                retainIter3(ii3) = nan;              % 存在互斥组合标记为nan
            end
        end
    end
else        % 不存在互斥则直接进行定位
    retainIter3 = ones(size(iter3,1),1);             % 保留的序数初始化为1
    [EstX,EstY] = LSM(Zt,node);
end
iter2 = nchoosek (1:length(Zt ),2);
if  ~isempty(MutExciterAngle) || ~isempty( MutExciterDis)       % 存在互斥，则四个量测不能同时使用
    if isempty(iter3(~isnan(retainIter3),:))                    % 存在互斥，且三个量测不能同时用
        retainIter2 = ones(size(iter2,1),1);                    % 保留的序数初始化为1
        for ii2 = 1:size(iter2,1)
            for ii = 1:size(MutExciterAngle,1)
                if length(find(ismember(iter2(ii2,:),MutExciterAngle(ii,:)))) >1
                    retainIter2(ii2) = nan;              % 存在互斥组合标记为nan
                end
            end
        end
    else % 存在互斥，且三个量测能同时用
        retainIter2 = [];
        iter3 = iter3(~isnan(retainIter3),:);
        [outLoctionCAX3,outLoctionCAY3] = deal([]);
        for ii = 1:size(iter3,1)
            [outLoctionCAX3(ii),outLoctionCAY3(ii)] = LSM(Zt(iter3(ii,:)),node(iter3(ii,:),:));
        end
        EstX = mean(outLoctionCAX3);
        EstY = mean(outLoctionCAY3);
    end
end
iter3       = nchoosek (1:length(Zt ),3);
iter2       = nchoosek (1:length(Zt ),2);
if ~isempty(MutExciterAngle) || ~isempty( MutExciterDis) 	% 存在互斥，则四个量测不能同时使用
    if isempty(iter3(~isnan(retainIter3),:))                % 存在互斥，且三个量测不能同时用
        if isempty(iter2(~isnan(retainIter2),:))            % 存在互斥，两个量测不能同时用
            EstX = nan;
            EstY = nan;
        else                                    % 存在互斥，两个量测能同时用
            iter2 = iter2(~isnan(retainIter2),:);
            [outLoctionCAX2,outLoctionCAY2] = deal([]);
            for ii = 1:size(iter2,1)
                [outLoctionCAX2(ii),outLoctionCAY2(ii)] = LSM(Zt(iter2(ii,:)),node(iter2(ii,:),:));
            end
            EstX = mean(outLoctionCAX2);
            EstY = mean(outLoctionCAY2);
        end
    end
end
else 
    EstX = nan;
    EstY = nan;
end
end

%% 
%%% 最小二乘法定位
function [EstX,EstY] = LSM(Zt,node)
    theta   = Zt;
    theta   = theta(~isnan(Zt))';
    if length(theta) >1 
        x1    	= node(~isnan(Zt),1);
        y1     	= node(~isnan(Zt),2);
        A       = [-tand(theta),ones(length(x1),1)];
        B       = y1-x1.*tand(theta);
        if isempty(A) && isempty(B)
            EstX = nan;
            EstY = nan;
        else
            X       = (A'*A)\A'*B;              % 目标位置X=[x;y]
            EstX    = X(1);
            EstY    = X(2);
        end
    else
        EstX = nan;
        EstY = nan;
    end
end

%% 还需要一个细关联的程序
function [] = FA(iter34) 
%{
1. 最小二乘法计算目标位置

2. 计算目标方位
3. 计算lamuda


%}


end
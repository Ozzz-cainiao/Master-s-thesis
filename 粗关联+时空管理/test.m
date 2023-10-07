%% 作为一个测试函数，测试写的粗关联程序是否正确
node            = [0,0;10e3,0;10e3,10e3;0,10e3];			% 节点位置
Zt = [45, 135, -135, -45];
Dth = 500;
% 理论结果应该是[5000, 5000]
[EstX,EstY] = CA(Zt,node,Dth);

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

%%% 粗关联定位
function [EstX,EstY] = CA(Zt,node,Dth)
%%% 测量不能超过20°约束 + 交点距离约束
iter        = nchoosek (1:length(Zt ),2);                                    % 组合数
for ii = 1:size(iter,1)         % 计算交点
    inputTheta = Zt(iter(ii,:));
    [poi1X(ii),poi1Y(ii)] = LSM(Zt(iter(ii,:)),node(iter(ii,:),:));
    if ((abs(inputTheta(1)-inputTheta(2)) >= 180-20 && abs(inputTheta(1)-inputTheta(2)) <= 180+20) ||  abs(inputTheta(1)-inputTheta(2)) <= 20)      % 出现测角在20°以内
        [poiX(ii),poiY(ii)] = deal(nan);
    else
        [poiX(ii),poiY(ii)] = deal(poi1X(ii),poi1Y(ii));
    end
end
MutExciterAngle  = iter(isnan(poiX),:);                    % 角度互斥测量
iter3       = nchoosek (1:length(Zt),3);
%%% 计算交点距离
for ii3 = 1:size(iter3,1)
    chosePoint = nchoosek (iter3(ii3,:),2);             % 选择交点的测量平台序数
    temp = nchoosek (1:size(chosePoint,1),2);           % 选择交点序数
    for ii2 = 1:size(temp,1)
        locsTemp = chosePoint(temp(ii2,:),:);
        for ii1 = 1: 2
			t = ismember(iter,locsTemp(ii1,:));  % 返回有的平台   t是6*2矩阵
			ccc = all(t,2);  % 返回两个都有的平台  ccc(0) = 1
            choosePoi1X(ii1) = poi1X(ccc);% locsTemp是iter的成员
            choosePoi1Y(ii1) = poi1Y(ccc);
        end
        chooseD(ii3,ii2) = sqrt( ( choosePoi1X(1) - choosePoi1X(2) )^2 + (choosePoi1Y(1) - choosePoi1Y(2) )^2 );    % 四个三元量测组的交点距离
    end
end
MutExciterDis = iter3( sum(chooseD <= Dth,2) < 2 , : )  ;       % 距离互斥组   

if ~isempty(MutExciterAngle) || ~isempty( MutExciterDis)       % 存在角度互斥 || 存在距离互斥  则四个量测不能同时使用
    tt = ismember(iter3,MutExciterDis)
	iter3 = iter3 ( ~all( tt ,2) ,:);    % 删除距离互斥组合
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
end

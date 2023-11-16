%% 先进行粗关联再进行时空关联

% Author LI Shuo
% 2023年9月9日 21点32分
clc
clear
close all


%% ==========================布放参数===============================
rand('seed',1)
c       = 1500;
var2d   = 1.5^2 ;
% var2d   = 0^2 ;
var2    = var2d*(pi/180)^2; 
v0     	= [10,10];
range 	= [ 6000 8e3 ];                 % 目标产生位置的距离
bear    = [ 60 30];                   % 目标产生位置的角度
course  = [ 90 50];               	% 运动方向
num     = length(v0);
x0      = range.*cosd(bear);
y0      = range.*sind(bear);
vy      = v0.*cosd(course);
vx      = v0.*sind(course);
X0      = [x0;y0;vx;vy];
T       = 1e-3;                 %观测周期
T_all   = 5;                   %观测时间
T_num   = T_all/T;              %观测次数
dt      = T;
%匀速运动矩阵
F1 = [1,0,T,0;
    0,1,0,T;
    0,0,1,0;
    0,0,0,1];
%加速度矩阵
F2 = [0.5*T^2,0;
    0,0.5*T^2;
    T,0;
    0,T];
x       = zeros(num,T_num); % 目标x坐标
y       = zeros(num,T_num); % 目标y坐标
x_r     = zeros(num,T_num); % 目标相对观测者x坐标
y_r     = zeros(num,T_num); % 目标相对观测者y坐标
angle_r = zeros(num,T_num); % 目标相对观测者方位
a_max   = 0/1e3;         	% 目标最大加速度
X       = arrayfun(@(j) zeros(4,T_num),1:num,'un',0);
%% ==========================目标真实状态===============================
for t = 1:T_num
    for j = 1:length(x0)
        ax = a_max*(2*rand-1);
        ay = a_max*(2*rand-1);
        a  = [ax ay]';
        if t==1
            X{j}(:,t) = X0(:,j);
        else
            X{j}(:,t) = F1*X{j}(:,t-1)+F2*a;
        end
    end
end
for i = 1:num
    x(i,:) = X{i}(1,:);
    y(i,:) = X{i}(2,:);
end
for kk = 1:1
%% ==========================观测===============================
node            = [0,0;10e3,0;10e3,10e3;0,10e3];                        % 节点位置
t_obs           = T:T:T_num*T+20;
X               = repmat( node,1,1,T_num);
[x_obs,y_obs]   = deal(zeros(size(node,1),T_num));
for ii = 1:size(node,1)
    x_obs(ii,:) = X(ii,1,:);
    y_obs(ii,:) = X(ii,2,:);
    angR{ii}   	= nan(num,length(t_obs));
    for i=1: num
        x_r(i,:)    = x(i,:)-x_obs(ii,1:T_num);
        y_r(i,:)    = y(i,:)-y_obs(ii,1:T_num);
        r_r         = sqrt(x_r(i,:).^2+y_r(i,:).^2);
        angle_r(i,:)= atan2d(y_r(i,:), x_r(i,:));
        angle_r(angle_r<0) = angle_r(angle_r<0)+360;
        t_r         = r_r/c;
        t_delay{i,ii} 	= round(t_r, 1);
        for iii = 1:T_num
            tNum = round(t_delay{ii}(iii)/T) + iii;
            angR{ii}(i,tNum) = angle_r(i,iii) +  sqrt(var2d)* randn;
        end  
    end
    for iii = 1: size(angR{ii},2)
        angR{ii}(:,iii) = sort( angR{ii}(:,iii) );
    end   
end
%% ==========================不进行时空关联定位===============================
%%% 多目标粗关联
area = 0:1000:10e3;
Dth     = 500;     % 交点距离约束
% for iii = 1:length(t_obs)
%     %%% Step1：生成交点
%     disp (['当前处理观测点',num2str(iii)])
%     Zt      = cell2mat(arrayfun(@(ii) angR{ii}(:,iii) , 1:size(node,1),'un',0));
%     if iii > 1
%         inputX = outLoctionCAX(:,iii-1);
%         inputY = outLoctionCAY(:,iii-1);
%     else
%         [inputX, inputY]  = deal(nan);
%     end
%     [outLoctionCAX(1:num,iii),outLoctionCAY(1:num,iii),locs]     = CA_Plus(Zt,node,Dth,inputX,inputY);
% end
% ==========================进行时空关联定位===============================
d_min   = 5;  % 时空关联解算阈值
DeTh    = 10;    % 迭代次数阈值
angRR   = angR; 
for iii = 1:length(t_obs)
    d = inf;
    if iii > 1
        inputX = outLoctionSPCX(:,iii-1);
        inputY = outLoctionSPCY(:,iii-1);
    else
        [inputX, inputY]  = deal(nan);
    end
    Zt      = cell2mat(arrayfun(@(ii) angR{ii}(:,iii) , 1:size(node,1),'un',0));
    [midOldCAX1,midOldCAY1,locs{iii}] =  CA_Plus(Zt,node,Dth,inputX,inputY);
%     [outLoctionSPCX(1:num,iii),outLoctionSPCY(1:num,iii),locs{iii}]     = CA_Plus(Zt,node,Dth,inputX,inputY);
% end
    for i = 1:num
        if isnan(midOldCAX1(i))
            outLoctionSPCX(i,iii) = midOldCAX1(i);
            outLoctionSPCY(i,iii) = midOldCAY1(i);
        else
            iikk = 1;
            while d > d_min && ~isnan(midOldCAX1(i))
                x_e     = midOldCAX1(i) - node(:,1);        % 估计位置与观测站横轴距离
                y_e     = midOldCAY1(i) - node(:,2);        % 估计位置与观测站纵轴距离
                r_e     = sqrt(x_e.^2+y_e.^2);          % 估计位置与观测站之间的距离
                t_e     = r_e/c;
                t_ed    = round( t_e-t_e(1) , 1);       % 时延差
                if ~all(t_ed<0)
                    t_ed(t_ed>0) =0;
                end
                loc_ed  = round(t_ed/T)+iii;            % 位置
                a_ed    = arrayfun(@(ii) locs{loc_ed(ii)}(ii,i) , 1:size(node,1));% 对应位置的测量序号
                s = 1:4;
                
                midZt  	= arrayfun(@(ii) angR{ii}(a_ed(ii),loc_ed(ii)) , s(~isnan(a_ed) & a_ed~=0));
                if length(midZt)>2
                    [midNewCAX,midNewCAY] =  CA(midZt,node(~isnan(a_ed) & a_ed~=0,:),Dth);
                elseif length(midZt) == 2
                    [midNewCAX,midNewCAY] = LSM(midZt,node(~isnan(a_ed) & a_ed~=0,:));
                else
                    [midNewCAX,midNewCAY]  = deal(nan);
%                     error('需要调试')
                end
                
                
                midOldCAX   = midNewCAX;
                midOldCAY   = midNewCAY;
                iikk = iikk +1;
                if iikk >= DeTh
                    break;
                end
            end
            outLoctionSPCX(i,iii) = midOldCAX;
            outLoctionSPCY(i,iii) = midOldCAY;
        end
        
    end

end

end

% color = ['#059341';'#EFA90D';'#059341';'#DC2F1F';'#EFA90D';'#7E2F8E'];
% figure('Units', 'centimeters','Position', [20 5 20 11.24/15*15])
% for i = 1:num
%     if i<8
%         color_line = color(1,:);
%     else
%         color_line = [1.0000    0.5721    0.0349];
%     end
%     hold on
%     outLoc  = plot(outLoctionCAX(i,:),outLoctionCAY(i,:),'.','Color',color_line,'LineWidth',2);
%     aim_a   = scatter(x0(i),y0(i),10,'*','MarkerEdgeColor',color(4,:));
%     aim_f   = plot(x(i,:),y(i,:),'Color',color(4,:),'LineWidth',2);
% end
% obs_f=scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
% grid on
% legend([outLoc,aim_a obs_f],'定位结果','真实轨迹','观测站','Location','eastoutside','FontSize',12)
% set(gca,'Box','on')
% xlabel('东向坐标/m','FontSize',12)
% ylabel('北向坐标/m','FontSize',12)
% title('粗关联定位')

color = ['#059341';'#EFA90D';'#059341';'#DC2F1F';'#EFA90D';'#7E2F8E'];
figure('Units', 'centimeters','Position', [20 5 20 11.24/15*15])
for i = 1:num
    if i<8
        color_line = color(i,:);
    else
        color_line = [1.0000    0.5721    0.0349];
    end
    hold on
    outLoc  = plot(outLoctionSPCX(i,:),outLoctionSPCY(i,:),'.','Color',color_line,'LineWidth',2);
    aim_a   = scatter(x0(i),y0(i),10,'*','MarkerEdgeColor',color(4,:));
    aim_f   = plot(x(i,:),y(i,:),'Color',color(4,:),'LineWidth',2);
end
obs_f=scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
grid on
legend([outLoc,aim_a obs_f],'定位结果','真实轨迹','观测站','Location','eastoutside','FontSize',12)
set(gca,'Box','on')
xlabel('东向坐标/m','FontSize',12)
ylabel('北向坐标/m','FontSize',12)
title('粗关联+时空关联定位')
aaa = 1;



%% ==========================子函数===============================
%% 函数功能:最大似然定位
% 输入参数:Zt 为角度信息,node为节点位置，area为观测区域，Ginitial为初始分辨率， Gtarget为最终分辨率
% 输出参数:EstX  EstY 分为横标纵标
function [EstX,EstY] = Grid_Based(Zt,node,area,Ginitial,Gtarget)
%Grid_Based 最大似然定位
% 
% INPUTS:
%   varargin{1}   - 角度信息
%   varargin{2}   - 节点位置
%   varargin{3}   - 观测区域
%   varargin{4}   - 初始分辨率
%   varargin{5}   - 最终分辨率
%
% OUTPUTS:
%   EstX  - 位置横坐标
%   EstY  - 位置纵坐标
    S = size(node,1);
    % 生成网格
    r = 10;                             % 分辨率增加的因子
    G = Ginitial;                       % 分辨率
    % 进行计算
    while G > Gtarget
        % 构建网格
        if G == Ginitial
            Gxx = area{1};                     	% 横标范围
            Gyy = area{2};                     	% 纵标范围
        else 
            Gxx = estX-V/2:G:estX+V/2;      	% 横标范围
            Gyy = estY-V/2:G:estY+V/2;      	% 横标范围
        end
        Gx      = rectpulse(Gxx,length(Gyy));       % 网格横标
        Gy      = repmat(Gyy,1,length(Gxx));        % 网格纵标
        Num     = length(Gy);                       % 网格点数
        PSI     = atan2d(Gy-node(:,2),Gx-node(:,1));% 节点角度矩阵  
        % 先判断是否漏检，再进行计算
        if isempty(find(isnan(Zt), 1))          % 没有漏检
            n_out = zeros(size(Gy));
            for nn = 1:Num
                nii = zeros(S,1);
                for ii = 1 : S
                    nii(ii,1) = AngelDeal(Zt(ii),PSI(ii,nn))^2;
                end
                n_out(1,nn) = sum(nii);
            end
        else                                    % 有漏检
            posS = 1:S;
            pos_nan = posS(isnan(Zt));          % 漏检序号
            n_out   = zeros(size(Gy));
            for nn = 1:Num
                nii = zeros(S,1);
                for ii = posS(~ismember(posS,pos_nan))
                    nii(ii,1) = AngelDeal(Zt(ii),PSI(ii,nn))^2;
                end       
                n_out(1,nn) = sum(nii);
            end
        end
        [~,pos] = min(n_out);
        estX = Gx(pos);
        estY = Gy(pos);
        V = G;
        G = G/r;       
    end 
    if length(find(isnan(Zt))) == size(node,1)
        EstX = nan;
        EstY = nan;
    else  
        EstX = estX;
        EstY = estY;
    end
end

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
function [EstX,EstY,locs] = CA_Plus(Zt,node,Dth,inputX,inputY)
if length(find(all(isnan(Zt),1))) == 0  
        Zt      = Zt(:,~all(isnan(Zt)));
        node1   = node(~all(isnan(Zt)),:);
        S       = size(node1,1);                 % 节点数
        angM    = mat2cell(Zt,size(Zt,1),ones(1,S));          % 角度特征元胞组
        nl      = cellfun(@length,angM(:));             % 各个平台测量数量
        subs    = arrayfun(@(x)0:x,nl,'un',0);          % 生成序号
        iter    = cell(S,1);
        [iter{end:-1:1}] = ndgrid(subs{end:-1:1});      % 生成网格
        temp    = cellfun(@(x)x(:),iter,'un',0);        % 中间变量
        iter    = num2cell(cat(2,temp{:}));             % 所有组合
        Niter   = size(iter,1);
        temp    = arrayfun(@(x) find(cell2mat(iter(x,:))),1:Niter,'un',0);  % 中间变量
        iter1   = iter(cellfun(@length,temp)==1,:);                         % 所有线的序数组
        iter2   = iter(cellfun(@length,temp)==2,:);                         % 所有两两相交的组合
        K       = cell(size(iter2,1),2);                                    % 预分配
        for k = 1:size(iter2,1)	% 一次循环替代嵌套
            locs1  	= find(cell2mat(iter2(k,:)));                              % 所选平台
            theta   = cellfun(@(x,y)x(y),angM(locs1),iter2(k,locs1),'un',0);	% 读取角度
            theta   = cell2mat(theta)';
            x1    	= node1(locs1,1);
            y1     	= node1(locs1,2);
            A       = [-tand(theta),ones(2,1)];
            B       = y1-x1.*tand(theta);
            X       = (A'*A)\A'*B;              % 目标位置X=[x;y]
            K(k,:)    = num2cell(X');           % 记录交点矩阵
        end
        iter22   = cat(2,iter2,K);                 % 所选平台测量序号+交点矩阵
        %%% Step2：计算与线L11 相交的所有交点
        iter41 = [];
        outIter33 =cell(1,nl(1));

        for ik = 1:nl(1)
            [seRow11,~]	= find(cell2mat(iter22(:,1))==ik);          % 寻找第一个平台第一个测量参与的组合数
            iter32 = [];
            for jj = 1:nl(2)
                [seRow21,~]     = find(cell2mat(iter22(:,1))==ik & cell2mat(iter22(:,2))==jj);  	% 其中寻找第二个平台第jj个测量参与的组合
                [seRow20,~] 	= find(cell2mat(iter22(:,1))==ik & cell2mat(iter22(:,2))==0);  	% 其中寻找第二个平台不参与参与的组合
                point1          = cell2mat(iter22(seRow21,S+1:S+2)).';
                for ii = 1:length(seRow20)
                    point2          = cell2mat(iter22(seRow20(ii),S+1:S+2)).';
                    iter31(ii,1:S) = [cell2mat(iter22(seRow21,1:2)),cell2mat(iter22(seRow20(ii),3:S))];
                    iter31(ii,S+1) =  norm(point2-point1);
                end
                iter32 = [iter32;iter31];
            end
            iter33 = iter32(iter32(:,S+1)<Dth,:);       % 通过门限的组合
            % 没有通过门限的逐步放宽门限
            cirNum = 1;
            while isempty(iter33) && cirNum <10
                iter33 = iter32(iter32(:,S+1)<Dth+50*cirNum,:);
                cirNum = cirNum+1;
            end
            if ~isempty(iter33) && all (~(diff(iter33(:,2)))) % 如果通过门限，且只有一个  第二个平台的测量关联
                iter34 = iter33;            %
            elseif ~isempty(iter33) && ~all (~(diff(iter33(:,2))))% 如果通过门限，且只有多个  第二个平台的测量关联
                outIter33{ik} = iter33;          % 保留竞争的筛选
                % 除了距离筛选 还要考虑是否竞争
                [~,minLocs] = min(iter33(:,S+1));
                iter34 = iter33(all( iter33(:,1:2) == iter33(minLocs,1:2) ,2),:);        
            end

            % 合并组合完成粗关联
            iter41(ik,1:2) = iter34(1,1:2);
            for jj = 3:S
                if length(find(iter34(:,jj))) == 1
                    % 在平台jj上只关联一个目标的组合，即正常关联组合
                    iter41(ik,jj) =  iter34(find(iter34(:,jj)),jj);
                elseif length(find(iter34(:,jj))) == 0 % 没通过门限
                    iter41(ik,jj) = 0;
                else % 关联两个需要细关联
%                     error('关联需要细关联')
                    outIter33{ik} = iter33;          % 保留竞争的筛选
                    [~,minLocs] = min(iter33(:,S+1));
                    iter34 = iter33(all( iter33(:,1:2) == iter33(minLocs,1:2) ,2),:); 
                    iter41(ik,jj) =  iter34(find(iter34(:,jj)),jj);
                end
            end
        end
        iter42 = iter41;
        iter43 = arrayfun( @(s) all (~(diff(iter41(:,s)))) ,1:S) ; % iter41出现相同测测量序号
        if find (all(iter41(:,iter43) ~=0 ) ) % 测量序号还不是0 即出现争抢测量的情况，需要处理
            for ik = 1:nl(1)
                if ~isempty(outIter33{ik})
                    for i = 1:nl(1)
                        if find( all( outIter33{ik}(:,1:2) == iter41(i,1:2) ,2 )) % 找到相同的
                            iter333 = outIter33{ik}(~all( outIter33{ik}(:,1:2) == iter41(i,1:2) ,2 ),:);
                            [~,minLocs] = min(iter333(:,S+1));
                            iter42(i,1:2) = iter333(1,1:2);
                            for jj = 3:S
                                if length(find(iter333(:,jj))) == 1
                                    % 在平台jj上只关联一个目标的组合，即正常关联组合
                                    iter42(ik,jj) =  iter333(find(iter333(:,jj)),jj);
                                elseif length(find(iter333(:,jj))) == 0 % 没通过门限
                                    iter42(ik,jj) = 0;
                                else % 关联两个需要细关联
                                    error('关联需要细关联')
                                end
                            end
                        end
                    end
                end
            end
        end
        [outLocX,outLocY] = deal(zeros(nl(1),1));
        for i = 1 : size(iter42,1)
            inputZt = arrayfun(@(s) Zt(iter42(i,s),s) ,find(iter42(i,1:S)));
            node2   = node1(find(iter42(i,1:S)),:);
            [outLocX(i),outLocY(i)]     = CA(inputZt,node2,Dth); 
        end 
        for i = 1 : size(iter42,1)
            for ii = 1 : size(inputX,1) 
                d(i,ii) = norm([outLocX(i) - inputX ,outLocY(i) - inputY ]);
            end
        end 
        if d(1,1) <=Dth
            EstX = outLocX;
            EstY = outLocY;
            locs = iter42';
        else
            EstX = fliplr(outLocX);
            EstY = fliplr(outLocY);
            locs = fliplr(iter42');
        end
        
    else
        [EstX,EstY]     = deal(nan(size(Zt,1),1));
        locs            = nan(size(Zt'));
   end
end
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
iter3       = nchoosek (1:length(Zt ),3);
%%% 计算交点距离
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
end

%% 角度处理
function A_X_Y = AngelDeal(X,Y)
A_XY = mod(X-Y,360);
A_YX = mod(Y-X,360);
A_X_Y = min(A_XY,A_YX);
end
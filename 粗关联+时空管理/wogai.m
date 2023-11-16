%% 先进行粗关联再进行时空关联
% 1. 布放参数部分：设置一些常数和初始条件，包括随机数种子、速度、目标距离、目标角度、目标数量等。
% 
% 2. 目标真实状态部分：计算目标在每个时刻的实际位置和速度，这里引入了随机加速度来模拟目标的运动过程。
% 
% 3. 观测部分：设置观测周期、观测时间和观测节点位置，计算观测数据的相对位置和方位，并加入噪声模拟测量误差。
% 
% 4. 不进行时空关联定位部分：设置区域，并计算交点距离。
% 
% 这段代码的主要功能是模拟雷达的观测过程，包括目标的运动和观测数据的处理。其中，目标真实状态部分和观测部分是最关键的部分，通过计算目标的实际位置和观测数据的相对位置和方位，可以进行后续的目标定位和跟踪。
%
% Author LI Shuo
% 2023年6月28日 10点22分
clc
clear
close all


%% ==========================布放参数===============================
rand('seed',1)
c       = 1500;
var2d   = 1.5^2 ;
% var2d   = 0^2 ;
var2    = var2d*(pi/180)^2; 
v0       = 10;
range 	= [ 6000 ];             % 目标产生位置的距离
bear    = [ 60 ];           	% 目标产生位置的角度
course  = [ 90 ];               % 运动方向
num     = length(v0);			% 设置目标数量
x0      = range.*cosd(bear);	% 计算起始位置
y0      = range.*sind(bear);
vy      = v0.*cosd(course);		% 计算起始速度
vx      = v0.*sind(course);
X0      = [x0;y0;vx;vy];		% 计算起始状态
T       = 1e-3;             %观测周期
T_all   = 10;				%观测时间
T_num   = T_all/T;          %观测次数
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

% 这段程序中，`arrayfun` 函数用于创建一个 `num` 行、`T_num` 列的矩阵 `X`。其中，每个元素是一个 4 行 `T_num` 列的零矩阵，表示一个目标的状态信息，包括 `(x,y,vx,vy)` 四个分量。具体解释如下：
% 
% - `arrayfun` 函数是一个基于数组的函数，用于将一个函数应用到一个数组的每个元素上，并将结果组成一个数组返回。
% 
% - 函数 `@(j) zeros(4,T_num)` 定义了一个匿名函数，它的输入参数是 `j`，表示第 `j` 个目标，输出结果是一个 4 行 `T_num` 列的零矩阵，表示该目标的状态信息。
% 
% - `1:num` 表示数组的输入，它是一个由 `1` 到 `num` 的整数构成的数组。
% 
% - `'un',0` 表示将输出的数组类型设置为不规则的单元数组。这是因为每个元素的大小不同，需要使用单元数组来存储。
% 
% 因此，这句程序的作用是创建一个 `num` 行、`T_num` 列的矩阵 `X`，其中每个元素是一个 4 行 `T_num` 列的零矩阵，表示一个目标的状态信息。
% %
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
node            = [0,0;10e3,0;10e3,10e3;0,10e3];			% 节点位置
t_obs           = T:T:T_num*T+10;							% 观测时间				
X               = repmat(node,1,1,T_num);					% 设置观测数据
[x_obs,y_obs]   = deal(zeros(size(node,1),T_num));
for ii = 1:size(node,1)										% 计算相对位置和方位
    x_obs(ii,:) = X(ii,1,:);
    y_obs(ii,:) = X(ii,2,:);
    for i=1: num
        x_r(i,:)    = x(i,:)-x_obs(ii,1:T_num);
        y_r(i,:)    = y(i,:)-y_obs(ii,1:T_num);
        r_r         = sqrt(x_r(i,:).^2+y_r(i,:).^2);
        angle_r(i,:)= atan2d(y_r(i,:), x_r(i,:));			% 这里改了  正东夹角
%         angle_r(i,:)= atan2d(x_r(i,:), y_r(i,:));			% 这里改了  改成与正北夹角

        angle_r(angle_r<0) = angle_r(angle_r<0)+360;
        t_r         = r_r/c;
    end
    t_delay{ii}   	= round(t_r, 1);
    angR{ii}        = nan(1,length(t_obs));
    for iii = 1:T_num
        tNum = round(t_delay{ii}(iii)/T) + iii;
        angR{ii}(tNum) = angle_r(iii) +  sqrt(var2d)* randn;
    end   
end
% %% ==========================不进行时空关联定位===============================
area = 0:1000:10e3;
Dth  = 500;     % 交点距离约束
% for iii = 1:length(t_obs)
%     Zt = arrayfun(@(ii) angR{ii}(iii) , 1:size(node,1));
%     [outLoctionLMSX(iii),outLoctionLMSY(iii)] = LSM(Zt,node);
%     [outLoctionMLX(iii),outLoctionMLY(iii)] = Grid_Based(Zt,node,{area;area},1000,1);
%     [outLoctionCAX(iii),outLoctionCAY(iii)] = CA(Zt,node,Dth);
% end
% %%% 计算误差
% for i = 1:T_num
%     errorLMS(i) = sqrt( ( x(i) - outLoctionLMSX(i+round(t_delay{1}(i)/T)) )^2 + ( y(i) - outLoctionLMSY(i+round(t_delay{1}(i)/T)) )^2 );
%     errorML(i)  = sqrt( ( x(i) - outLoctionMLX(i+round(t_delay{1}(i)/T)) )^2 + ( y(i) - outLoctionMLY(i+round(t_delay{1}(i)/T)) )^2 );
%     errorCA(i)  = sqrt( ( x(i) - outLoctionCAX(i+round(t_delay{1}(i)/T)) )^2 + ( y(i) - outLoctionCAY(i+round(t_delay{1}(i)/T)) )^2 );
% end
% outErrorLMS = mean( errorLMS(~isnan(errorLMS)) );
% outErrorML  = mean( errorML(~isnan(errorML)) );
% outErrorCA 	= mean( errorCA(~isnan(errorCA)) );

%% ==========================进行时空关联定位===============================
% 这段程序进行了时空关联定位，主要是求解无线定位中的位置估计值。其中包含以下几个步骤：
% 1. 定义时空关联解算阈值 d_min 和迭代次数阈值 DeTh
% 2. 循环遍历观测站
% 3. 对于每个观测站，进行迭代，直到满足停止条件（d <= d_min 或者迭代次数 >= DeTh）
% 4. 在迭代过程中，根据观测站测得的角度和节点位置，估计位置与观测站之间的距离（r_e）和时延差（t_ed）
% 5. 根据时延差，计算出当前时刻各个节点的角度，并通过 CA（Circle Algorithm）算法估计位置
% 6. 计算当前估计位置与上一次估计位置的距离 d，如果 d > d_min，则继续迭代
% 7. 将最终的估计位置存储到 outLoctionSPCX 和 outLoctionSPCY 中
% 8. 计算估计误差 outErrorSPC，并将其存储到 outError 中
d_min   = 5;  % 时空关联解算阈值
DeTh    = 10;    % 迭代次数阈值
for iii = 1:length(t_obs)
    d = inf;
% 	xxx = 1:size(node,1);
%     midZt      = arrayfun(@(ii) angR{ii}(iii) , xxx);
%它的作用是将一个cell数组angR中第iii个元素的每个向量的第iii个值提取出来，存储到一个向量midZt中
midZt      = arrayfun(@(ii) angR{ii}(iii) , 1:size(node, 1));
    [midOldCAX,midOldCAY] = CA(midZt,node,Dth);
    iikk = 1;
    while d > d_min && ~isnan(midOldCAX)
        x_e     = midOldCAX - node(:,1);        % 估计位置与观测站横轴距离
        y_e     = midOldCAY - node(:,2);        % 估计位置与观测站纵轴距离
        r_e     = sqrt(x_e.^2+y_e.^2);          % 估计位置与观测站之间的距离
        t_e     = r_e/c;
        t_ed    = round( t_e-t_e(1) , 1);       % 时延差
        midZt  	= arrayfun(@(ii) angR{ii}(iii+ round(t_ed(ii)/T)) , 1:size(node,1));
        [midNewCAX,midNewCAY] = CA(midZt,node,Dth);
        d           =  sqrt( ( midOldCAX - midNewCAX )^2 + ( midOldCAY - midNewCAY )^2 );
        midOldCAX   = midNewCAX;
        midOldCAY   = midNewCAY;
        iikk = iikk +1;
        if iikk >= DeTh
            break;
        end

    end
    outLoctionSPCX(iii) = midOldCAX;
    outLoctionSPCY(iii) = midOldCAY;
end
for i = 1:T_num
    errorSPC(i)  = sqrt( ( x(i) - outLoctionSPCX(i+round(t_delay{1}(i)/T)) )^2 + ( y(i) - outLoctionSPCY(i+round(t_delay{1}(i)/T)) )^2 );
end
outErrorSPC      = mean( errorSPC(~isnan(errorSPC)) );
outError(kk,:) = [outErrorLMS, outErrorML, outErrorCA, outErrorSPC];
end
disp(['  ', num2str([outErrorLMS, outErrorML, outErrorCA, outErrorSPC]  ) ])
%% ==========================绘图===============================
color = ['#059341';'#EFA90D';'#059341';'#DC2F1F';'#EFA90D';'#7E2F8E'];
figure('Units', 'centimeters','Position', [20 5 20 11.24/15*15])
for i = 1%1:num
    if i<8
        color_line = color(i,:);
    else
        color_line = [1.0000    0.5721    0.0349];
    end
    aim_f(i) = scatter(x0(i),y0(i),10,'*','MarkerEdgeColor',color_line);
    hold on
    plot(x(i,:),y(i,:),'Color',color_line,'LineWidth',2)
end
obs_f=scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
grid on
legend([aim_f obs_f],'目标','观测站','Location','eastoutside','FontSize',12)
set(gca,'Box','on')
xlabel('东向坐标/m','FontSize',12)
ylabel('北向坐标/m','FontSize',12)
title('真实轨迹')


figure('Units', 'centimeters','Position', [20 5 20 11.24/15*15])
for i = 1%1:num
    if i<8
        color_line = color(i,:);
    else
        color_line = [1.0000    0.5721    0.0349];
    end
    hold on
    outLoc  = plot(outLoctionCAX(i,:),outLoctionCAY(i,:),'.','Color',color_line,'LineWidth',2);
    aim_a   = scatter(x0(i),y0(i),10,'*','MarkerEdgeColor',color(4,:));
    aim_f   = plot(x(i,:),y(i,:),'Color',color(4,:),'LineWidth',2);
end
obs_f=scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
grid on
legend([outLoc,aim_a obs_f],'定位结果','真实轨迹','观测站','Location','eastoutside','FontSize',12)
set(gca,'Box','on')
xlabel('东向坐标/m','FontSize',12)
ylabel('北向坐标/m','FontSize',12)
title('粗关联定位')

figure('Units', 'centimeters','Position', [20 5 20 11.24/15*15])
for i = 1%1:num
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



color = ['#059341';'#EFA90D';'#059341';'#DC2F1F';'#EFA90D';'#7E2F8E'];
for ii = 1:4
figure
hold on
for i=1
    if i<8
        color_line=color(i,:);
    else
        color_line=[1.0000    0.5721    0.0349];
    end
    hold on
    scatter(angR{ii}(i,:),t_obs,1,'filled','MarkerEdgeColor',color_line,'MarkerFaceColor',color_line,'LineWidth',2)
end

xlim([(ii-1)*90,ii*90])
set(gca,'Box','on')
xlabel('角度/°','FontSize',12)
ylabel('时间/s','FontSize',12)
legend('目标','Location','best','FontSize',12)
txtStr = ['观测站',num2str(ii)];
grid on
Hf = gcf;
Hf.Position = [-700   130  560   420];
end
for i = 1:num
angR{i} = angR{i}([1],:);
end
x = x([1],:);
y = y([1],:);
x0 = x0([1]);
y0 = y0([1]);




figure
plot(t_obs(1:T_num),errorCA,'LineWidth',1,'Color',color(1,:))
grid on
ylim([0,10])
xlabel('时间/s','FontSize',12)
ylabel('距离误差/m','FontSize',12)
title('粗关联误差')


figure
plot(t_obs(1:T_num),errorSPC,'LineWidth',1,'Color',color(1,:))
grid on
% ylim([0,10])
xlabel('时间/s','FontSize',12)
ylabel('距离误差/m','FontSize',12)
title('粗关联+时空关联误差')
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
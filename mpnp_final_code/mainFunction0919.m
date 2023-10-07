%% 利用分治贪心算法实现709项目关联
% Author LI Shuo
% 2023年9月19日 10点20分
clc
clear
close all

%% ==========================布放参数===============================
rand('seed',1)
c       = 1500;
var2d   = 1.5^2 ;
% var2d   = 0.5^2 ;
var2    = var2d*(pi/180)^2; 
v0     	= [10,10];
range 	= [ 6000 8e3 ];         % 目标产生位置的距离
bear    = [ 60 30];         	% 目标产生位置的角度
course  = [ 90 50];           	% 运动方向
num     = length(v0);
S     	=  3; % 目标数目
node   	= round( 5e3*sqrt(2)*[cosd(-135:360/S:224)',sind(-135:360/S:224)']+[5e3,5e3] ); % 节点位置
Dth     = 500; 
dPonit  = 10;
PD      = 0.99;%0.9        	% 检测概率2~3个目标时是0.99，4~5个目标时是0.9
Fai     = 2*pi;
M       = 3;                % 选取最少线点数
Q       = 10;                % 列表最大长度
I       = 3;                % 并行次优条数
%%
x0      = range.*cosd(bear);
y0      = range.*sind(bear);
vy      = v0.*cosd(course);
vx      = v0.*sind(course);
X0      = [x0;y0;vx;vy];
T       = 1e-3;                 % 观测周期
T_all   = 20;                	% 目标持续时间
T_num   = T_all/T;              % 观测次数
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
% figure
% hold on
% for i = 1:num
%     plot(x(i,:),y(i,:),'.')
% end
% scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
% for kk = 1:1
%% ==========================观测===============================
dT              = 5;
t_obs           = T+dT*2:T:dT*3;
X               = repmat( node,1,1,T_num);
[x_obs,y_obs]   = deal(zeros(size(node,1),T_num));
for ii = 1:S
    x_obs(ii,:) = X(ii,1,:);
    y_obs(ii,:) = X(ii,2,:);
    angR{ii}   	= nan(num,length(t_obs)*10);
    for i=1: num
        x_r(i,:)    = x(i,:)-x_obs(ii,1:T_num);
        y_r(i,:)    = y(i,:)-y_obs(ii,1:T_num);
        r_r         = sqrt(x_r(i,:).^2+y_r(i,:).^2);
        angle_r(i,:)= atan2d(y_r(i,:), x_r(i,:));
        angle_r(angle_r<0) = angle_r(angle_r<0)+360;
        t_r         = r_r/c;
        t_delay{i,ii} 	= round(t_r, 1);
        for iii = 1:T_num
            tNum = round(t_delay{i,ii}(iii)/T) + iii;
            angR{ii}(i,tNum) = angle_r(i,iii) +  sqrt(var2d)* randn;
        end  
    end 
end
angM = cell(length(t_obs),S);
for iii = 1:length(t_obs)
    angM(iii,:) = arrayfun(@(s) angR{s}( ~isnan( sort( angR{s}(:,t_obs(1)/T+iii-1) ) ) ,t_obs(1)/T+iii-1) ,1:S,'un',0);
end


%% ==========================分治贪心关联===============================
Dth     = 500; 
dPonit  = 10;
PD      = 0.99;%0.9        	% 检测概率2~3个目标时是0.99，4~5个目标时是0.9
Fai     = 2*pi;
M       = 3;                % 选取最少线点数
Q       = 10;                % 列表最大长度
I       = 3;                % 并行次优条数
[outLoctionCAX,outLoctionCAY] = deal(nan(num,length(t_obs)));
outAngM = cell(num,length(t_obs));
for iii = 1:length(t_obs)
    disp(['正在处理',num2str(iii)])
    angM1       = angM(iii,:);
    ns          =    cellfun(@(x)size(x,1),angM1);
    if num==2 && S==2
        if all( ns~=1 )             % 无解只剩下两个平台有测量，不能进行关联，只能对一个目标进行处理
            [outLocX,outLocY] = deal(nan);
        else                        % 只有一个测量，直接进行定位                    
            Z                   = cell2mat(angM1(locs_notnan));        
            [outLocX,outLocY]   = LSM(Z,node); 
        end       
    else
        ns      = arrayfun(@(x) size(angM1{x}(~isnan(angM1{x})),1),1:S);
        if all(ns==1)
            Z = arrayfun(@(x) angM1{x}(~isnan(angM1{x})),1:S);
            [outLocX,outLocY] = LSM(Z,node);
        else
            Z       = arrayfun(@(x) {angM1{x}(~isnan(angM1{x}))} ,1:S,'un',0);  
            outDCGT = dcgt(Z,node,[var2,PD,Fai],[M,Q,I]);
            outLocs = outDCGT(:,1:length(Z));
            outLocX = outDCGT(:,length(Z)+1);
            outLocY = outDCGT(:,length(Z)+2);
        end  
    end
    %%% 实现跟原有轨迹的跟踪
    if iii> dPonit
        inputX = outLoctionCAX(:,iii-10:iii-1);
        inputY = outLoctionCAY(:,iii-10:iii-1);
    elseif iii>1
        inputX = outLoctionCAX(:,1:iii-1);
        inputY = outLoctionCAY(:,1:iii-1);
    else
        [inputX,inputY] = deal(nan(num,1));
    end
    di = zeros(size(outLocX,1),size(inputX,1));         % 与既有轨迹的距离的预分配  
    for i = 1 : size(outLocX,1)
        for ii = 1 : size(inputX,1)
            d1 = zeros(1,size(inputX,2));
            for iiii =  1 : size(inputX,2)
                d1(iiii) = norm([outLocX(i) - inputX(ii,iiii) ,outLocY(i) - inputY(ii,iiii) ]);
            end
            di(i,ii) = mean(d1(~isnan(d1)));
        end
    end
    di(isnan(di)) = 1e8;
    if size(di,1) > size(di,2)                                % HungarianAlgorithm.m只能对列数≥行数的正确关联
        [~ ,zeta]   =  HungarianAlgorithm(di');
        zeta = zeta';
    else
        [~ ,zeta]   =  HungarianAlgorithm(di);
    end
    EstX    = outLocX.'*zeta;
    EstY    = outLocY.'*zeta;
    EstLocs = outLocs.'*zeta;
    EstX(EstX==0)=nan;
    EstY(EstY==0)=nan;
    EstLocs(:,all(EstLocs==0,1))=nan;
    EstLocs = EstLocs.';
    if length(EstX) == 1
        outLoctionCAX(1,iii) = EstX(1)';
        outLoctionCAY(1,iii) = EstY(1)';
        outLoctionCAX(2:num,iii) = nan;
        outLoctionCAY(2:num,iii) = nan;
        outAngM{1,iii} = arrayfun(@(s) angM1{s}(EstLocs(1,s)),1:S);
    else
        outLoctionCAX(1:num,iii) = EstX';
        outLoctionCAY(1:num,iii) = EstY';
        for i=1: num
            if all(isnan(EstLocs(i,:)))             % 全为nan，即唯有测量
                outAngM{i ,iii} = [];
            else
                for s = 1:S
                    if EstLocs(i,s) ~=0
                        outAngM{i ,iii}(s) = angM1{s}(EstLocs(i,s));
                    else
                        outAngM{i ,iii}(s) = nan;
                    end
                end
            end
        end
    end 
end
% save('midResult.mat','outLoctionCAX','outLoctionCAY','outAngM')
% load midResult.mat
%% ==========================分治贪心关联+时空关联===============================
d_min     = 5;        % 时空关联解算阈值
DeTh        = 10;       % 迭代次数阈值
[outLoctionSPCX,outLoctionSPCY] = deal(nan(num,length(t_obs)));
for iii = 1:length(t_obs)
    disp(['正在处理',num2str(iii)]) 
    % 进行时空关联
    for i = 1:num
        if isnan(outLoctionCAX(i))
            outLoctionSPCX(i,iii) = outLoctionCAX(i,iii);
            outLoctionSPCY(i,iii) = outLoctionCAY(i,iii);
        else
            d = inf;
            iikk = 1;
            midOldCAX = outLoctionCAX(i,iii);
            midOldCAY = outLoctionCAY(i,iii);
%             while d > d_min && ~isnan(outLoctionCAX(i))
            while d > d_min && iikk < DeTh
                x_e     = midOldCAX - node(:,1);        % 估计位置与观测站横轴距离
                y_e     = midOldCAY - node(:,2);        % 估计位置与观测站纵轴距离
                r_e     = sqrt(x_e.^2+y_e.^2);          % 估计位置与观测站之间的距离
                t_e     = r_e/c;
                t_ed    = round( t_e-t_e(1) , 1);       % 时延差
                loc_ed  = round(t_ed/T)+iii;            % 位置
                if all(loc_ed > 0 & loc_ed < length(t_obs))      % 时空关联能够进行
                    angM2 = zeros(1,S);
                    for s = 1:S
                        if isempty(outAngM{i,loc_ed(s)})
                            ij = 1;
                            while ij <10
                                if ~isempty(outAngM{i,loc_ed(s)+ij})
                                    angM2(s) = outAngM{i,loc_ed(s)+ij}(s);
                                    break
                                else
                                    ij = ij+1;
                                end
                            end
                        else
                            angM2(s) = outAngM{i,loc_ed(s)}(s);
                        end
                    end
                    [midNewCAX,midNewCAY] = LSM(angM2,node);
%                 elseif d ~= inf | iikk ~= 1
%                     midOldCAX = midNewCAX;
                else
                    outLoctionSPCX(i,iii) = midOldCAX;
                    outLoctionSPCY(i,iii) = midOldCAY;
                    break;
                end
                d       =  sqrt( ( midOldCAX - midNewCAX )^2 + ( midOldCAY - midNewCAY )^2 );
                midOldCAX   = midNewCAX;
                midOldCAY   = midNewCAY;
                iikk    = iikk +1;
            end
            outLoctionSPCX(i,iii) = midOldCAX;
            outLoctionSPCY(i,iii) = midOldCAY;
        end
    end  
end
%% ==========================分析误差===============================
%%% 只有样例部分存在
for iii = 1:length(t_obs)
    for ii = 1:num
        if ~isnan(outLoctionCAX(ii,iii))
            x_e     = outLoctionCAX(ii,iii) - node(:,1);        % 估计位置与观测站横轴距离
            y_e     = outLoctionCAY(ii,iii) - node(:,2);        % 估计位置与观测站纵轴距离
            r_e     = sqrt(x_e.^2+y_e.^2);          % 估计位置与观测站之间的距离
            t_e     = r_e/c;
            t_ed    = round( t_e(1)/T );       % 时延差
            errorCA(ii,iii) = sqrt( ( x(ii,t_obs(1)/T-t_ed) - outLoctionCAX(ii,iii))^2 + ( y(ii,t_obs(1)/T-t_ed) - outLoctionCAY(ii,iii))^2 );
            errorSPC(ii,iii) = sqrt( ( x(ii,t_obs(1)/T-t_ed) - outLoctionSPCX(ii,iii))^2 + ( y(ii,t_obs(1)/T-t_ed) - outLoctionSPCY(ii,iii))^2 );
        else
            errorCA(ii,iii) = nan;
            errorSPC(ii,iii) =nan;
        end
    end 
end
outErrorCA      = mean( errorCA(~isnan(errorCA)) );
outErrorSPC     = mean( errorSPC(~isnan(errorSPC)) );
%% ==========================绘图区===============================
figure
hold on
for ii = 1:num
plot(outLoctionCAX(ii,:),outLoctionCAY(ii,:),'.')
axis([0,10e3,0,10e3])
end


figure
hold on
for ii = 1:num
plot(outLoctionSPCX(ii,:),outLoctionSPCY(ii,:),'.')
axis([0,10e3,0,10e3])
end

% aa
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

warning('off')
Z       = varargin{1};
node    = varargin{2};
var2    = varargin{3}(1);               % 角度测量方差单位rad
PD      = varargin{3}(2);               % 检测概率
Fai     = varargin{3}(3);
M       = varargin{4}(1);               % 选取最少线点数
Q       = varargin{4}(2);               % 列表最大长度
I       = varargin{4}(3);               % 并行次优条数
S       = size(node,1);                 % 节点数
angM    = arrayfun(@(x)cat(2,cell2mat(x{1,1}(:,1))),Z,'un',0);          % 角度特征元胞组
%%% ==========================基于分治思想的最优交点集合选取===============================
%%% Step1：生成交点
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
    x1    	= node(locs1,1);
    y1     	= node(locs1,2);
    A       = [-tand(theta),ones(2,1)];
    B       = y1-x1.*tand(theta);
    X       = (A'*A)\A'*B;              % 目标位置X=[x;y]
    K(k,:)    = num2cell(X');           % 记录交点矩阵
end
iter22   = cat(2,iter2,K);                 % 所选平台测量序号+交点矩阵
%%% Step2：对每条线选取最短的M条线段
W = {};
for k = 1:size(iter1,1)
    locs21    = find(cell2mat(iter1(k,:)));               % 所选平台序号
    locsM   = iter1{k,locs21};                            % 所选测量序号
    Num22   = find(cell2mat(iter22(:,locs21))==iter1{k,locs21});    % 在iter22筛选出来的序号
    point   = iter22(Num22,:);                               % 筛选出来的测量序号+位置
    kk      = 1;
    kknum   = 1;
    locs22   = cell(sum(1:size(point,1)-1),3);           % 预分配
    while kk < size(point,1)
        point1 = cell2mat(point(kk,S+1:S+2)).';
        for ii = kk+1:size(point,1)
            point2      = cell2mat(point(ii,S+1:S+2)).';
            locs22{kknum,1} 	= Num22(kk);                            % iter22中行序号--对应点
            locs22{kknum,2} 	= Num22(ii);                            % iter22中行序号--对应点
            locs22{kknum,3} 	= norm(point2-point1);                  % 距离
            kknum       = kknum+1;
        end
        kk = kk+1;
    end
    [~,locs23] = sort(cell2mat(locs22(:,3)));           % 按升序排列后的序数
    if length(locs23)>M
        W = cat(1,W,locs22(locs23(1:M),:));                 % W中三列分为iter22中行序号；iter22中行序号；距离
    else
        W = cat(1,W,locs22(locs23(1:length(locs23)),:));
    end
    
end        
%%% Step3：定位
Num3        = size(W,1);
Lamdad      = cell(Num3,2);
for k = 1:Num3
    locs31          = iter22(cell2mat(W(k,1:2)),:);         % locs31所选测量序号，和交点位置
    [locs32,locs33] = find(cell2mat(locs31(:,1:S)));        % locs32行序数-》无意义 locs33列序数-》所选平台
    if length(unique(locs33))<3
        locs35	= arrayfun(@(x,y) locs31(x,y),locs32,locs33);   % locs35测量序数，与locs33配合读取测量
        theta   = cellfun(@(x,y) angM{x}(y),num2cell(locs33),locs35,'un',0);
        theta   = cell2mat(theta);
        Lamdad{k,1} = cat(2,locs33,cell2mat(locs35),theta);   % 平台序号;测量序号;测量值
        Lamdad{k,2} = nan;                % 似然
    else
        [locs33,locs34] = unique(locs33);                       % locs34独一无二数的平台数的序数， locs33列序数更新
        locs32  = locs32(locs34);                               % locs32行序数更新
        locs35	= arrayfun(@(x,y) locs31(x,y),locs32,locs33);   % locs35测量序数，与locs33配合读取测量
        theta   = cellfun(@(x,y) angM{x}(y),num2cell(locs33),locs35,'un',0);
        theta   = cell2mat(theta);
        x3    	= node(locs33,1);
        y3   	= node(locs33,2);
        A       = [-tand(theta),ones(3,1)];
        B       = y3-x3.*tand(theta);
        Xd     	= (A'*A)\A'*B;                  % 估计目标位置Xd=[x;y]
        thetaE  = atan2d(Xd(2)-y3,Xd(1)-x3);    % 估计目标位置到平台角度      
        %%% 换成对数似然
        c               = zeros(1,S); 
        for ii = 1 : S
            if ismember(ii,locs33)
                detla_l = 1;
                thetaii     = theta(ismember(locs33,ii));
                thetaEii    = thetaE(ismember(locs33,ii));
                esis        = AngelDeal(thetaii,thetaEii)/180*pi;
                c(ii)       = detla_l*(-log(PD*Fai/sqrt(2*pi*var2))+0.5*(esis/sqrt(var2))^2);
            else
                u = 0;
                c(ii) = (u-1)*log(1-PD);
            end
        end
        Lamdad{k,1} = cat(2,locs33,cell2mat(locs35),theta);   % 平台序号;测量序号;测量值
        Lamdad{k,2} = sum(c);                % 似然
    end   
end        
%%% ==========================基于贪心思想的量测集合合并===============================
Lamdad_W    = Lamdad(~isnan(cell2mat(Lamdad(:,2))),:);% 排除一个组合有多个同平台测量
Num4        = size(Lamdad_W,1);
%%% 对获得组合按平台-测量序数分
Set         = cell(1,S);
for ii = 1:S
    for jj = 1:nl(ii)
        locs41	= arrayfun(@(x) sum(Lamdad_W{x,1}(:,1)==ii & Lamdad_W{x,1}(:,2)==jj),1:Num4,'un',0) ;
        Set{ii} = cat(1,Set{ii},Lamdad_W(logical(cell2mat(locs41')),:));
    end
end
s   = 2;
Z  = Set{1,s-1};
W_Plus  = {};
while s <= S
    Z1      = {};
    z       = Set{1,s};
    NumZ    = size(Z,1);
    Numz    = size(z,1);
    R       = zeros(NumZ,Numz);
    for ii = 1:NumZ
        for jj = 1:Numz
            Mea     = cat(1,Z{ii},z{jj});
         	locs51  = arrayfun(@(x) find(Mea(:,1)==x),1:S ,'un',0);     % 平台1：S对应再矩阵的序号
            locs52  = cellfun(@length,locs51);                          % 有多余1个测量的序号
            Mea2    = cellfun(@(x) Mea(x,:),locs51(locs52>1),'un',0);   % 有多余1个测量的Mea
            locs53  = arrayfun(@(x) unique(Mea2{1,x}(:,2)),1:length(Mea2),'un',0);     % 有多余1个测量的平台中 重复测量数量
            locs54  = cellfun(@length,locs53);
            if isempty(find(locs54>1, 1))
                [locs55,locs56] = unique(Mea(:,1));
                if length(locs55)==S                % 遍历所有组合->输出
                    if isempty(W_Plus)              % 第一个值
                        W_Plus  = cat(1,W_Plus,{Mea(locs56,:)});
                        R(ii,jj)    = 1;                                            % 成功融合且没有输出过的标记为1
                    else                            % 已有输出集
                        for kk = 1:length(W_Plus)   % 对输出集遍历
                            if sum(W_Plus{kk}==Mea(locs56,:),'all')==numel(Mea(locs56,:)) % 如果和输出集相等
                                R(ii,jj)    = 2;                                    % 已成功融合但输出过的标记为2
                            end
                        end
                        if   R(ii,jj) ~= 2    
                             W_Plus  = cat(1,W_Plus,{Mea(locs56,:)});
                             R(ii,jj)    = 1;                                       % 成功融合且没有输出过的标记为1
                        end
                    end
                else
                    if isempty(Z1)              % 第一个值                   
                        Z1          = cat(1,Z1,{Mea(locs56,:)});
                        R(ii,jj)    = 3;                                                % 成功融合但未遍历所有平台的标记为3
                    else
                        for kk = 1:length(Z1)   % 对输出集遍历
%                             if sum(Z1{kk}==Mea(locs56,:),'all') == numel(Mea(locs56,:))% 如果和输出集相等
                            %%% 以下判断条件20230919修改，此时s=5时出现错误
                            [Lia, ~] =ismember(Z1{kk}(:,1:2),Mea(locs56,1:2), 'rows');
                            
                            if length( find(Lia)) == size( Mea(locs56,:) ,1)            % 如果Mea已经在输出集里面
                                R(ii,jj)    = 4;                                        % 已成功融合但未遍历所有平台的标记为4
                            end
                        end
                        if   R(ii,jj) ~= 4    
                             Z1         = cat(1,Z1,{Mea(locs56,:)});
                             R(ii,jj) 	= 3;                                       % 成功融合但未遍历所有平台的标记为3
                        end
                    end
                        
                end   
            end
        end
    end
    %%%更新关联下一个平台
    [locs61,locs62] = find(R);
    numZ = 1:NumZ;
    if sum(ismember(numZ,locs61)) ~= NumZ               % 未关联的行
        locs63 = numZ(~ismember(numZ,locs61));          % 未更新的行号
        Z1 = cat(1,Z1,Z{locs63,1});
    end
    numz = 1:Numz;
    if sum(ismember(numz,locs62)) ~= Numz           % 未关联的列
        locs64 = numz(~ismember(numz,locs62));      % 未关联的列号
        Z1 = cat(1,Z1,z{locs64,1});
    end
    Z = Z1;
    s = s+1;
end
W_Plus = cat(1,W_Plus,Z);
            
%%% 计算位置和关联似然概率并保留最大的Q个组合
Lamda7 = cell(length(W_Plus),1);
for k = 1:length(W_Plus)
    Wk      = W_Plus{k};
    theta   = Wk(:,3);
    x6    	= node(Wk(:,1),1);
    y6   	= node(Wk(:,1),2);
    A       = [-tand(theta),ones(size(Wk,1),1)];
    B       = y6-x6.*tand(theta);
    Xd     	= (A'*A)\A'*B;                  % 估计目标位置Xd=[x;y]
    thetaE  = atan2d(Xd(2)-y6,Xd(1)-x6);    % 估计目标位置到平台角度
    p       = arrayfun(@(x,y) DistributedDeal(x/180*pi,y/180*pi,var2),theta,thetaE);
    Lamda7{k}  = prod(p);
    %%% 换成对数似然
    c   = zeros(1,S);
    for ii = 1 : S
        if ismember(ii, Wk(:,1))
            detla_l = 1;
            thetaii     = theta(ismember(Wk(:,1),ii));
            thetaEii    = thetaE(ismember(Wk(:,1),ii));
            esis        = AngelDeal(thetaii,thetaEii)/180*pi;
            c(ii)       = detla_l*(-log(PD*Fai/sqrt(2*pi*var2))+0.5*(esis/sqrt(var2))^2);
        else
            u = 0;
            c(ii) = (u-1)*log(1-PD);  
        end
    end
    Lamda7{k}  = sum(c);
end
W_Plus      = cat(2,W_Plus,Lamda7);
[~,locs71]	= sort(cell2mat(Lamda7),'descend');
[~,locs71]	= sort(cell2mat(Lamda7));
if length(locs71)>Q
    W_Plus7     = W_Plus(locs71(1:Q),:);
else
    W_Plus7     = W_Plus(locs71,:);
end 
%%% ==========================基于贪心思想的量测总集划分===============================
%%%寻找不互斥的输出测量组
R8  = zeros(size(W_Plus7,1));
for ii = 1:size(W_Plus7,1)
    W81 = W_Plus7{ii,1}(:,1:2);
    for jj = 1:size(W_Plus7,1)
        W82 =  W_Plus7{jj,1}(:,1:2);
        locs81 = cat(1,W81,W82);                    % 对两个集合拼接
        locs82 = arrayfun(@(x) locs81(locs81(:,1)==x,2),unique(locs81(:,1)),'un',0);    % 平台测量数
        locs83 = cellfun(@(x) length(unique(x))==length(x),locs82,'un',0);              % 平台测量互斥性，如果1不互斥；如果0则互斥
        if length(find(cell2mat(locs83))) == length(locs82)                          	% 如果不互斥的平台数==有测量平台数
            R8(ii,jj) = 1;                                                              % 不互斥的标记为1
        end
    end
end
q   = 2;
Psi = {{};W_Plus7{q-1,1}};
L   = [0;1];
P   = [0;W_Plus7{q-1,2}];
while q <= size(W_Plus7,1)
    NumPsi = size(Psi,1);
    Psi = cat(2,repmat(Psi,2,1),cat(1,cell(NumPsi,1),repmat(W_Plus7(q,1),NumPsi,1)));       % 兼容组合
    L   = cat(2,repmat(L,2,1),cat(1,zeros(NumPsi,1),ones(NumPsi,1)));                       % 兼容组合索引
    P   = sum(cat(2,repmat(P,2,1),cat(1,zeros(NumPsi,1),W_Plus7{q,2}*ones(NumPsi,1))),2);   % 总似然
    locsdet = [];
    for ii = NumPsi+1:NumPsi*2
        Lq  = L(ii,:);
        locs91 = find(Lq==1)';
        if sum(Lq==1) == 2                  % 存在两个关联量则检查是否能同时输出
            if R8(locs91(1),locs91(2))~=1   % 不可以同时输出
                locsdet = cat(1,locsdet,ii); 
            end
        elseif sum(Lq==1) > 2
            locs92 = [locs91(1:end-1),locs91(end)*ones(sum(Lq==1)-1,1)];                % 待处理的兼容组合
            if sum(arrayfun(@(x,y) R8(x,y)==1,locs92(:,1),locs92(:,2)))~=sum(Lq==1)-1   % 没有和所有兼容
                locsdet = cat(1,locsdet,ii);
            end
        end
    end
    Psi = Psi(~ismember(1:end,locsdet),:);
    L   = L(~ismember(1:end,locsdet),:);
    P   = P(~ismember(1:end,locsdet),:);
    if length(P)>I
        [~,locs93] = sort(P,'descend');
        [~,locs93] = sort(P);
        Psi = Psi(locs93(1:I),:);
        L   = L(locs93(1:I),:);
        P   = P(locs93(1:I),:);
    end
    q = q+1;
end

%%% ==========================求解位置和关联组合=============================== 
%%% 以下判断条件20230919修改，此时出现Psi(1,:)={}情况
if isempty(Psi{1,1}) && ~isempty(Psi{2,1})
    Psi_opt = Psi(2,:);
else
    Psi_opt = Psi(1,:);
end
% Psi_opt = Psi(1,:);
Xout   	= [];
Pos     = zeros(length(Psi_opt),S);
for ii = 1:length(Psi_opt)
    if ~isempty(Psi_opt{ii})
        %%% 求解位置
        theta   = Psi_opt{ii}(:,3);
        x9    	= node(Psi_opt{ii}(:,1),1);
        y9   	= node(Psi_opt{ii}(:,1),2);
        A       = [-tand(theta),ones(size(Psi_opt{ii},1),1)];
        B       = y9-x9.*tand(theta);
        Xd     	= (A'*A)\A'*B;                  % 估计目标位置Xd=[x;y]
        Xout   = cat(2,Xout,Xd);
        %%% 求解组合数
        Pos(ii,Psi_opt{ii}(:,1)) = Psi_opt{ii}(:,2);  
    else
        Pos(ii,:) = nan;
    end
end
%%% ==========================求解关联组合=============================== 
Pos1 = Pos(~isnan(Pos));
Pos2 = reshape(Pos1,[],S);

%%% ==========================输出=============================== 

varargout{1} = [Pos2,Xout'];

end



%%% 最小二乘法定位
function [EstX,EstY] = LSM(Zt,node)
    theta   = Zt;
    theta   = theta(~isnan(Zt))';
    x1    	= node(~isnan(Zt),1);
    y1     	= node(~isnan(Zt),2);
    A       = [-tand(theta),ones(length(x1),1)];
    B       = y1-x1.*tand(theta);
    X       = (A'*A)\A'*B;              % 目标位置X=[x;y]
    if isempty(X)
        EstX = inf;
        EstY = inf;
    else
        EstX    = X(1);
        EstY    = X(2);
    end
end

%%% 角度处理
function A_X_Y = AngelDeal(X,Y)
    A_XY = mod(X-Y,360);
    A_YX = mod(Y-X,360);
    A_X_Y = min(A_XY,A_YX);
end
%% 子函数--角度变成概率函数
function P = DistributedDeal(theta1,theta2,thetas2)
    if theta1 - theta2 < -pi
        P = normpdf(theta1,theta2-2*pi,thetas2);
    elseif theta1 - theta2 > pi
        P = normpdf(theta1,theta2+2*pi,thetas2);
    else
        P = normpdf(theta1,theta2,thetas2);
    end
end

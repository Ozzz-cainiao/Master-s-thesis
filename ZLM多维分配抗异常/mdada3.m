% 验证算法可行性专用，TH1相关参数有变

function varargout = mdada3(varargin)
%mdada 基于角度的SD关联
% ZLM自用版本
%   《Indoor Multiple Sound Source Localization via Multi-Dimensional Assignment Data Association》
% INPUTS:
%   varargin{1}   - 测量
%   varargin{2}   - 节点位置
%   varargin{3}   - 定位区域
%   varargin{4}   - 分布参数等
%
% OUTPUTS:
%   varargout{1}  - 输出
Z       = varargin{1};
node    = varargin{2};
area    = varargin{3};
var2    = varargin{4}(1); % 均方根误差
PD      = varargin{4}(2);
Fai     = varargin{4}(3);
ns      = cellfun(@(x)size(x,1),Z);
S       = size(node,1);                 % 节点数
zs      = arrayfun(@(x)cat(2,cell2mat(x{1,1}(:,1))),Z,'un',0);          % 角度特征元胞组
% len = numel(Z); % Z 数组的长度
% combinations = generate_combinations(Z, len, cell(1, len), 1, {}); % 生成所有可能的组合
% iter1 = vertcat(combinations{:});

%%% 获取所有组合的定位结果和代价
varRange    = arrayfun(@(x)0:x,ns,'un',0);                              % 循环变量集合
maxIter     = ns+1;
subs        = arrayfun(@(x)1:x,maxIter,'un',0);                         % 数目元组
iter        = cell(S,1);
[iter{end:-1:1}] = ndgrid(subs{end:-1:1});                              % 构建重复网格
temp        = cellfun(@(x)x(:),iter,'un',0);                            % 整形
iter        = num2cell(cat(2,temp{:}));                                 % 循环变量索引,cat拼接
zk          = cellfun(@(x)cat(1,nan,x),zs,'un',0);                      % 将序数0对于的nan与测量拼接，形成广义测量
outK1       = zeros(size(iter,1),S+3);
for k = 1:size(iter,1)	% 一次循环替代嵌套
    vari            = cellfun(@(x,y)x(y),varRange,iter(k,:),'un',0);   % 当前循环序数
    zkk             = cellfun(@(x,y)x(y),zk,iter(k,:),'un',0);          % 当前循环测量
    zkk             = cell2mat(zkk);
%     [x_est,y_est]   = Grid_Based(zkk,node,{area;area});                 % 用网格法估计位置(最大似然)
    [x_est,y_est]   = LSM(zkk,node);                                    % 用最小二乘
    angE            = atan2d(y_est-node(:,2).', x_est-node(:,1).');   	% 计算估计位置到节点角度
    c               = zeros(1,S);   
    for ii = 1 : S
        if isnan(zkk(ii))
            u       = 0;
            c(ii)   = (u-1)*log(1-PD);
        else
            u       = 1;
            esis    = AngelDeal(zkk(ii),angE(ii))/180*pi;
            c(ii)   = u*(-log(PD*Fai/sqrt(2*pi*var2))+0.5*(esis/sqrt(var2))^2);
        end
    end
    outK1(k,:) = [cell2mat(vari),x_est,y_est,sum(c)]; % 生成关联序数、位置、空间关联代价
end
%% ==========================拉格朗日松弛===============================
%%% 利用门控技术初筛组合
Q = S*max(ns)*4;
LocsK1  = sort(outK1(:,S+3));
if isinf(LocsK1(Q))
    TH1     = max(LocsK1(~isinf(LocsK1)));
else
    TH1     = LocsK1(Q);
end
% 存在虚警的情况
if max(ns) > 3
    TH1 = outK1(1,S+3);
    temp    = arrayfun(@(x) find(outK1(x,1:4)),1:size(outK1,1),'un',0);   % 删除只有一个平台的测量
    TH1 = min(outK1(cellfun(@length,temp)==1,S+3));
end

inputK1 = outK1(outK1(:,S+3)<=TH1,:); % 只留下代价小于门限的
if isempty(inputK1)
    varargout{1} = [];
    return;
else
%%% step1 初始化
epsilon = 0.1;      	% 预设阈值
K       = 100;        	% 最大迭代次数
r       = 3:S;          % rth
u_r_ir  = cell(1,S);    % Lagrangian multipliers
for rr = r
    u_r_ir{1,rr} = zeros(ns(rr)+1,1);
end
f_dual      = -inf;     % dual func
f_primal    = inf;      % primal func
iter        = 0;        % 
gap         = inf;      % gap
while iter <= K && gap >= epsilon
    R       = 2;
    N_est   = ns(1);
%%% step2 计算降低的成本
%   逐步降低约束条件，将原始的S-D问题转化为一系列较小规模的子问题
	r       = S-1:-1:2; % 
    clear d
    d{S}    = inputK1;
    for rr = r
        % 检查是否存在上一唯独的可行解d{rr+1}
        if ~isempty(d{rr+1})
            % 将上一维度可行解d{rr+1}中的第1到第rr列提取出来，并去重，得到集合CNum
            CNum = unique(d{rr+1}(:,1:rr),'rows'); 
            % 对于集合CNum中的每个元素，找到在上一维度可行解d{rr+1}中对应的行，并将其作为新的一组可行解d_up。
            % 如果该组可行解只有一行，则直接将其添加到本维度的可行解集合d{rr}中
            % 如果该组可行解有多行，则计算它们的成本，并选择成本最小的一组添加到本维度的可行解集合d{rr}中。
            for i1_ir = 1:size(CNum,1)
                locs21      = sum(d{rr+1}(:,1:rr)==CNum(i1_ir,1:rr),2) == rr;   % 在inputK2中的位置
                if length(d{rr+1}(locs21,1))==1
                    % d_up作为新的可能的可行解
                    d_up    = cat(2,d{rr+1}(locs21,1:S+2),d{rr+1}(locs21,S+3)-u_r_ir{rr+1}(d{rr+1}(locs21,rr+1)+1));
                    d{rr}   = cat(1,d{rr},d_up);
                else
                    [~,locs22] = min(d{rr+1}(locs21,S+3)-u_r_ir{rr+1}(d{rr+1}(locs21,rr+1)+1));       % 代价最小的组合
                    d_up    = cat(2,d{rr+1}(locs21,1:S+2),d{rr+1}(locs21,S+3)-u_r_ir{rr+1}(d{rr+1}(locs21,rr+1)+1));
                    d{rr}   = cat(1,d{rr},d_up(locs22,:));
                end
            end
        end
    end
%%% step3 强制约束集 在这段代码中，是对每个维度r进行强制约束集的处理，
% 目的是根据前一步计算得到的可行解，确定每个维度r的关联矩阵relatedM，
% 并使用匈牙利算法（Hungarian Algorithm）求解二维关联，以满足约束条件。
% 具体步骤如下： 
    while R <= S
        % 初始化r为当前维度R，设置lamdaSetR为第r维的可行解。
        r = R;  
        % 构建关联矩阵relatedM
        % 如果r为2，表示当前维度为最底层，直接将第2维的可行解作为lamdaSetR，
        %  并设置ass{r}为(1:N_est)'，即已关联组合测量的索引序列。
        if r == 2
            lamdaSetR   = d{1,r} ;
            ass{r}    	= (1:N_est)';                                 % 已关联组合测量
        % 如果r不为2，根据上一级成功关联的测量的索引locs32，在lamdaSetR中选择相应的行，
        % 作为新的lamdaSetR，同时更新ass{r}为上一级成功关联的组合测量。
        else
            locs32      = sort(locsM(logical(locsM.*omega)));       % 上一级lamdaSetR中成功关联的测量
            locs34      = arrayfun(@(x) sum(d{1,r}(:,1:r-1)==lamdaSetR(locs32(x),1:r-1),2) == r-1,1:length(locs32),'un',0);
            locs34      = logical(sum(cell2mat(locs34),2));
            ass{r}    	= lamdaSetR(locs32,1:r-1);                  % 已关联组合测量
            lamdaSetR   = d{1,r}(locs34,:);
        end
        % 根据新的lamdaSetR和已关联组合测量ass{r}，
        % 构建关联矩阵relatedM和位置矩阵locsM，用于后续的匈牙利算法求解。
        NumM        = 1:size(lamdaSetR,1);
        relatedM    = 10e3*ones(N_est+1,ns(r)+1);
        locsM       = zeros(N_est+1,ns(r)+1);
        for lamdaNum = 1:size(ass{r},1)
            for rr = 0:ns(r)
                if ~isempty(lamdaSetR)
                    locs31 = sum(lamdaSetR(:,1:r-1) == ass{r}(lamdaNum,:) & lamdaSetR(:,r)== rr,2) == r-1;
                    if ~isempty(lamdaSetR(locs31,S+3))
                        relatedM(lamdaNum+1,rr+1)   = lamdaSetR(locs31,S+3);
                        locsM(lamdaNum+1,rr+1)      = NumM(locs31);
                    end
                end
            end
        end
        % 利用匈牙利算法HungarianAlgorithm求解二维关联，得到二维关联矩阵omega，
        % 表示r维度上各个测量与已关联组合测量的关系。
        omega = zeros(size(relatedM));
        if size(ass{r},1) > ns(r)+1         % HungarianAlgorithm.m只能对列数≥行数的正确关联
            [J(r) ,omegaTemp]  = HungarianAlgorithm(relatedM(2:end,2:end)');
            omega(2:end,2:end) = omegaTemp';
        else
            [J(r) ,omega(2:end,2:end)] =  HungarianAlgorithm(relatedM(2:end,2:end));
        end  
        omega(logical((relatedM==10e3).*omega)) = 0;     % 关联到10e3代价的关联指数 置0，因为没有此数值，故没有关联
        
        % 检查是否满足约束 
        % 如果存在未关联的已关联集合或者未关联的待关联变量，则强制进行关联。
        check1 = omega * ones(ns(r)+1,1);       % 检查是否有已关联集合没有成功关联 已关联集合序号0:N_est 索引1:N_est+1
        check2 = ones(1,N_est+1) * omega;       % 检查是否有待关联变量没有成功关联 已关联集合序号0:ns(r) 索引1:ns(r)+1
        if sum(check1(2:N_est+1)) ~= N_est      % 在左部将未关联已关联集合 与 测量0 关联
            omega(~check1 & [0;ones(N_est,1)],1) = 1;
        end
        if sum(check2(2:ns(r)+1)) ~= ns(r)      % 在上部将未关联测量和已测量集合0关联
            omega(1,~check2 & [0,ones(1,ns(r))]) = 1;
        end
        J(r) = sum(sum(relatedM.*omega)); % 根据关联矩阵omega和相关成本计算J(r)，即第r维的代价
        N_est   = length(find(omega)); % 更新N_est为已关联的测量数目，以便在下一次迭代中使用。
%%% step4 更新拉格朗日乘数u_r_ir 
        % 首先，检查当前维度r是否小于最大维度S，如果是则继续进行更新操作，
        % 否则不进行更新（因为已经到达最高维度，无需再更新）。
        if R < S
            % 如果r等于2，则将当前维度的成本J(r)赋值给J2_star，用于后续计算
            if r == 2
                J2_star = J(r);
            end
            % 初始化一个长度为ns(r+1)+1的数组g{r+1}，并设置所有元素为1。
            % 该数组用于存储更新的步长
            g{r+1}  = ones(1,ns(r+1)+1);
            k       = 1:N_est;
            ir      = 1:ns(r);
            mu      = zeros(1,size(omega,2));
            % 遍历二维关联矩阵omega，对于每个关联的测量，更新g{r+1}数组中对应位置的值。
            % 同时，根据关联测量的索引，从lamdaSetR中获取对应的代价，将这些代价存储在数组mu中。
            for lamdaNum = 0:size(omega,1)-1
                for rr = 0:size(omega,2)-1
                    if omega(lamdaNum+1,rr+1) == 1 && locsM(lamdaNum+1,rr+1)~=0
                        j           = lamdaSetR(locsM(lamdaNum+1,rr+1),r+1);
                        g{r+1}(j+1) = g{r+1}(j+1)-1;
                        mu(rr+1)    = lamdaSetR(locsM(lamdaNum+1,rr+1),S+3);
                    end
                end
            end
            % adapt = (J(r)-f_dual)/norm(g{r+1})^2*(ns(r)*mu)/sum(mu);
            % 计算更新步长adapt，这里暂时将其设置为1。
            adapt = 1;
            % 将更新步长adapt乘以数组g{r+1}，
            % 然后加到当前维度的拉格朗日乘数u_r_ir{r+1}中，完成一轮乘数的更新。
            u_r_ir{r+1} = u_r_ir{r+1} + (adapt.*g{r+1})';
        end
        % 最后，将R加1，准备进行下一轮的更新操作。
        R = R+1;
%%% step5 递归：确定 R-D 问题的分配
% 首先判断当前维度R是否等于总维度S，如果是则表示已经到达最高维度，需要确定最终的分配方案。
% 如果当前维度R等于总维度S，说明已经完成了对所有维度的约束放松和分配操作，
% 需要确定最终的分配方案。这里通过计算omega矩阵中非零元素的个数，来确定已成功关联的测量数目N_est。
        if R==S
            N_est   = length(find(omega)); % 
        end
    end
%%% step6 迭代：提高解决方案质量   
    f_dual      = max(f_dual,J2_star);
    f_primal    = min(f_primal,J(S));
    gap         =(f_primal-f_dual)/abs(f_primal);
    iter        = iter+1;
end

out1 = lamdaSetR(sort(locsM(logical(locsM.*omega))),:);
temp = arrayfun(@(x) find(out1(x,1:4)),1:size(out1,1),'un',0);   % 删除只有一个和零个平台的测量
out2 = out1(cellfun(@length,temp)>1,:);

out3 = out2(out2(:,S+3)<TH1,:);
varargout{1} = out3;         % 1:S关联结果 S+1:S+2位置 S+3 代价
% index = out3(:, 1: S);
% for i = 1:size(index, 1)
%     extracted_data{i,:} = cellfun(@(idx, x) Z{x}{idx}, num2cell(index(i, :)), num2cell(1:S));
% end
% varargout{2} = cell2mat(extracted_data);
end
end

%% ==========================子函数===============================
%%% 最大似然网格法求解
%{
`Grid_Based` 函数是一个通过最大似然网格法求解纯方位定位问题的函数。下面重新解释一下这个函数的主要部分：

1. `Grid_Based` 函数接收观测值 `Zt`、节点位置 `node` 和搜索区域 `area` 作为输入参数。
2. 函数首先定义了一些参数，如节点数量 `S`、初始网格分辨率 `Ginitial`、目标网格分辨率 `Gtarget` 
    和分辨率增加因子 `r` 等。
3. 在主循环中，函数通过不断调整网格分辨率 `G`，在给定的搜索区域内生成网格，
    并计算每个网格点的似然度。具体地，函数根据当前网格分辨率生成网格，并计算每个网格点到节点位置的角度差，
    然后利用 `AngelDeal` 函数处理角度差，得到角度差的最小值，作为该网格点的似然度。
4. 如果观测值中存在缺失值 `NaN`，则将漏检的节点排除在计算之外。
5. 在每次循环结束时，选择似然度最小的网格点作为当前估计位置，并更新估计的位置坐标 `estX` 和 `estY`。
6. 每次循环结束后，将当前的网格分辨率 `G` 缩小一定比例 `r`，直到网格分辨率达到目标分辨率 `Gtarget`。
7. 最终，返回最终估计的位置坐标 `EstX` 和 `EstY`，即纯方位定位的估计结果。
这个函数的核心思想是通过网格搜索的方式，在给定的搜索区域内寻找最有可能的目标位置，以实现纯方位定位的目标。
%}
function [EstX,EstY] = Grid_Based(Zt,node,area)
    S = size(node,1);
    % 生成网格
    Ginitial    = 0.5;                  % 初始分辨率
    Gtarget     = 5e-4;                 % 最终分辨率
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
        Gx  = rectpulse(Gxx,length(Gyy));       % 网格横标
        Gy  = repmat(Gyy,1,length(Gxx));        % 网格纵标
        Num   = length(Gy);                     % 网格点数
        PSI = atan2d(Gy-node(:,2),Gx-node(:,1));% 节点角度矩阵  
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
    EstX = estX;
    EstY = estY;
end

%%% 最小二乘法定位
function [EstX,EstY] = LSM(Zt,node)
    theta = Zt;
    theta = theta(~isnan(Zt))';
    x1    = node(~isnan(Zt),1);
    y1    = node(~isnan(Zt),2);
    A     = [-tand(theta),ones(length(x1),1)];
    B     = y1-x1.*tand(theta);
    X     = (A'*A)\A'*B;              % 目标位置X=[x;y]
    if isempty(X)
        EstX = inf;
        EstY = inf;
    else
        EstX = X(1);
        EstY = X(2);
    end
end

%%% 角度处理
function A_X_Y = AngelDeal(X,Y)
    A_XY = mod(X-Y,360);
    A_YX = mod(Y-X,360);
    A_X_Y = min(A_XY,A_YX);
end

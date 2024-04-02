%最近邻关联(NNDA)方法对稀疏目标关联功能函数说明：
%输入：Zk(targetnum*2,:)当前帧量测值，2表示xy坐标，targetnum目标个数
%      L轨迹起始判断L个点，Lth表示判断是否收敛的阈值,
%      disth目标相邻两点间距最大值，T采样间隔
%      Z_NNDAforward(targetnum*2,L)，上一帧的关联结果
%      Z_kalmanforward(targetnum*2,L)，上一帧的滤波结果
%      forwardNum，上一帧的帧数
%      RealNumforward(targetnum,:)，到上一帧为止有效点数，每个目标均有两个有效点之后才进行滤波
%      v_last(targetnum,2)，上一帧数计算得到的目标速度，2表示vx和vy
%      X_Preforward(4,targetnum)，P_Preforward，上一帧滤波预测结果，用于当前帧NNDA
%      bisendforward(targetnum,L)上一帧轨迹终止标志
%      endZkforward(targetnum,L)上一帧轨迹终止对应存储量测值
%输出：Z_NNDA(targetnum*2,L)关联后结果，Z_kalman(targetnum*2,L)滤波后结果,
%      RealNum到当前帧为止有效点数，
%      v_update(targetnum,2)，当前帧数计算更新后的目标速度，2表示vx和vy
%      X_Pre(4,targetnum)，P_Pre，上一帧滤波预测结果，用于当前帧NNDA
%      bisconvergence(targetnum,2)表示前L个点是否收敛（值为1表示收敛，值为0表示未收敛，NaN表示不是第L帧），2表示vx和vy
%      bisend(targetnum,L)当前帧轨迹终止标志
%      endZk(targetnum,L)当前帧轨迹终止对应存储量测值
%整体流程：
%       0.参数设置
%       1.航迹起始：当前帧数forwardNum+1<=L帧 简单关联inNNDAeasy+三点平滑滤波Mean
%           此时传出关联和滤波结果点数为当前帧数forwardNum+1个
%       2.0 当前帧数forwardNum+1>L帧：判断是否轨迹终止，是的话进行轨迹起始inBegin；
%       2.1 关联：分为简单关联和最近邻关联
%       2.2 更新轨迹终止标志（若成功进行轨迹起始，将当前帧之前的轨迹终止对应存储量测值置空）
%       2.3 关联滤波值输出：传出关联和滤波结果，点数为L个，多余L帧之后每次都传出L个点，绘图L个点
function [Z_NNDA, Z_mean, RealNum, v_update, X_Pre, P_Pre, bisconvergence, bisend, endZk] = myNNDAQZ(Zk, targetnum, L, Lth, disth, T, Z_NNDAforward, Z_meanforward, forwardNum, RealNumforward, v_last, X_Preforward, P_Preforward, bisendforward, endZkforward)
%*****************************************************************
%          0.参数设置
%*****************************************************************
A_Model = [1, T, 0, 0; ...
    0, 1, 0, 0; ...
    0, 0, 1, T; ...
    0, 0, 0, 1]; %建立目标运动模型--CV模型
H = [1, 0, 0, 0; ...
    0, 0, 1, 0]; %测量模型
Q_Model = 1; %建立模型的过程噪声
r = 0.1;
R = [r, 0; ...
    0, r]; %量测噪声,越小关联越好
R11 = r;
R22 = r;
R12 = 0;
R21 = 0;
P_NNSF = [R11, R11 / T, R12, R12 / T; ...
    R11 / T, 2 * R11 / T^2, R12 / T, 2 * R12 / T^2; ...
    R21, R21 / T, R22, R22 / T; ...
    R21 / T, 2 * R21 / T^2, R22 / T, 2 * R22 / T^2]; %初始量测协方差
G = [T^2 / 2, 0; ...
    T, 0; ...
    0, T^2 / 2; ...
    0, T]; %噪声加权矩阵

%*****************************************************************
%          1.航迹起始前L帧
%*****************************************************************
Z_NNDA = zeros(targetnum*2, 1);
Z_NNDAbeg = zeros(targetnum*2, 1);
Z_mean = zeros(targetnum*2, 1);
Z_meanbeg = zeros(targetnum*2, 1);
RealNum = zeros(targetnum, 1);
v_update = zeros(targetnum, 2);
X_Pre = zeros(4, targetnum); %卡尔曼滤波预测的目标状态，用于最近邻关联
P_Pre = cat(3);
Pkk = cat(3);
bisconvergence = zeros(targetnum, 2);
bisend = zeros(targetnum, L);
bisend1 = zeros(targetnum, 1);
endZk = zeros(targetnum*2, L);
Xkk0 = zeros(4, targetnum);
Xkk1 = zeros(4, targetnum);
Zkk = zeros(4, targetnum);
Xkk = zeros(4, targetnum); %当前时刻目标状态（滤波后）
Zkxy = zeros(targetnum*2, 1); %航迹起始targetnum个目标L个点的x,y坐标
%1.1 第一帧，直接输出
if forwardNum == 0
    Z_NNDA = Zk;
    Z_mean = Zk;
    bisend = bisendforward;
    endZk = endZkforward;
    for iv = 1:targetnum
        RealNum(iv, 1) = length(find(~isnan(Zk(iv*2, 1))));
        v_update(iv, 1) = NaN;
        v_update(iv, 2) = NaN;
        bisconvergence(iv, 1) = NaN;
        bisconvergence(iv, 2) = NaN;
        X_Pre(:, iv) = [NaN; NaN; NaN; NaN];
        P_Pre = cat(3, P_Pre, P_NNSF);
        Pkk = cat(3, Pkk, P_NNSF);
        %     bisend(iv,1:L) = 0;
        %     endZk(:,1:L) = [NaN;NaN;NaN;NaN];
    end
    %1.2 第二帧到第八帧，进行简单关联和滤波
elseif 1 <= forwardNum && forwardNum < L
    bisend = bisendforward;
    endZk = endZkforward;
    %%%1.2.1 简单关联
    [Zkxy] = inNNDAeasy(Z_NNDAforward(:, end), Zk, targetnum, disth);
    %%%1.2.2 对传出变量（除去滤波相关）赋值
    Z_NNDA(:, 1:forwardNum) = Z_NNDAforward;
    Z_NNDA(:, forwardNum+1) = Zkxy;
    for iv = 1:targetnum
        RealNum(iv, 1) = length(find(~isnan(Zkxy(iv*2, 1)))) + RealNumforward(iv, 1);
        bisconvergence(iv, 1) = NaN;
        bisconvergence(iv, 2) = NaN;
    end
    %%%1.2.3 判断是否进行滤波
    for itar = 1:targetnum
        islvbo = 1;
        if RealNum(itar, 1) < 2
            islvbo = 0;
        end
        if islvbo == 0 %不够2个点，无法计算vxvy，不进行滤波
            Z_mean(1+(itar - 1)*2:itar*2, 1:forwardNum+1) = Z_NNDA(1+(itar - 1)*2:itar*2, 1:forwardNum+1);
            v_update(itar, 1) = NaN;
            v_update(itar, 2) = NaN;
            X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
            P_Pre = cat(3, P_Pre, P_NNSF);
        else %够2个点，计算vxvy
            Z_mean(1+(itar - 1)*2, 1:forwardNum) = Z_meanforward(1+(itar - 1)*2, 1:forwardNum);
            Z_mean(itar*2, 1:forwardNum) = Z_meanforward(itar*2, 1:forwardNum);
            ind_vx = find(~isnan(Z_NNDA(1+(itar - 1)*2, :)));
            ind_vy = find(~isnan(Z_NNDA(itar*2, :)));
            v_update(itar, 1) = Z_NNDA(1+(itar - 1)*2, ind_vx(end)) - Z_NNDA(1+(itar - 1)*2, ind_vx(end-1));
            v_update(itar, 2) = Z_NNDA(itar*2, ind_vx(end)) - Z_NNDA(itar*2, ind_vx(end-1));
            if RealNum(itar, 1) == 2 %第一次够2个点，不滤波
                Z_mean(1+(itar - 1)*2, forwardNum+1) = Z_NNDA(1+(itar - 1)*2, ind_vx(end)) + (forwardNum + 1 - ind_vx(end)) * v_update(itar, 1) * T;
                Z_mean(itar*2, forwardNum+1) = Z_NNDA(itar*2, ind_vy(end)) + (forwardNum + 1 - ind_vy(end)) * v_update(itar, 2) * T;
                Xkk(1, itar) = Z_mean(1+(itar - 1)*2, forwardNum+1);
                Xkk(3, itar) = Z_mean(itar*2, forwardNum+1);
                Xkk(2, itar) = v_update(itar, 1);
                Xkk(4, itar) = v_update(itar, 1);

            else %前几帧至少有3个有效点，进行滤波
                Xkk0(:, itar) = [Z_meanforward(1+(itar - 1)*2, forwardNum); v_update(itar, 1); Z_meanforward(itar*2, forwardNum); v_update(itar, 2)];
                Xkk1(:, itar) = [Z_meanforward(1+(itar - 1)*2, forwardNum) + v_update(itar, 1) * 2 * T; v_update(itar, 1); Z_meanforward(itar*2, forwardNum) + v_update(itar, 2) * 2 * T; v_update(itar, 2)];
                Zkk(:, itar) = [Z_NNDA(1+(itar - 1)*2, forwardNum+1); v_update(itar, 1); Z_NNDA(itar*2, forwardNum+1); v_update(itar, 2)];
                [Xk] = Mean(Xkk0(:, itar), Xkk1(:, itar), Zkk(:, itar), A_Model);
                Xkk(:, itar) = Xk;
                Z_mean(1+(itar - 1)*2, forwardNum+1) = Xkk(1, itar);
                Z_mean(itar*2, forwardNum+1) = Xkk(3, itar);
            end
            X_Pre(:, itar) = A_Model * Xkk(:, itar); %预测下一时刻的目标状态
            %                 Pkk = cat(3,Pkk,P_NNSF);
            P_Pre(:, :, itar) = A_Model * P_NNSF * A_Model' + G * Q_Model * G';
        end
    end
    %1.2.4 到第L帧时，计算vx和vy，用速度众数求平均作为初始速度，判断前L帧是否收敛
    if forwardNum == L - 1
        %%%计算速度
        bisconvergence = zeros(targetnum, 2); %速度收敛标志
        for itar = 1:targetnum
            Vx = zeros(1); %前L个点计算得到的L*(L-1)/2个速度
            Vy = zeros(1);
            Zkxx = Z_NNDAforward(1+(itar - 1)*2, :);
            Zkyy = Z_NNDAforward(itar*2, :);
            iv = 1;
            %通过前L个点计算L*(L-1)/2个速度
            for il = 1:size(Zkxx, 2)
                for jl = il + 1:1:size(Zkxx, 2)
                    vx = (Zkxx(1, jl) - Zkxx(1, il)) / (T * (jl - il));
                    vy = (Zkyy(1, jl) - Zkyy(1, il)) / (T * (jl - il));
                    if isnan(vx)
                        continue;
                    end
                    Vx(iv) = vx;
                    Vy(iv) = vy;
                    iv = iv + 1;
                end
            end
            %找到速度众数作为速度初始值
            [v_update(itar, 1), modenum1] = modemeanv(Vx);
            [v_update(itar, 2), modenum2] = modemeanv(Vy);
            if modenum1 < Lth
                bisconvergence(itar, 1) = 0; %该条轨迹前L个点不收敛
            else
                bisconvergence(itar, 1) = 1;
            end
            if modenum2 < Lth
                bisconvergence(itar, 2) = 0; %该条轨迹前L个点不收敛
            else
                bisconvergence(itar, 2) = 1;
            end
        end
    end

    %*****************************************************************
    %          2.量测值>=L个点，进行最近邻关联并滤波，传出后L个NNDA点
    %*****************************************************************
elseif forwardNum >= L %第九帧及以后,直接进行NNDA和卡尔曼滤波
    %     if forwardNum==37
    %         ssss = 1;
    %     end
    for itar = 1:targetnum
        RealNum(itar, 1) = length(find(~isnan(Zkxy(itar*2, 1)))) + RealNumforward(itar, 1);
        bisconvergence(itar, 1) = NaN;
        bisconvergence(itar, 2) = NaN;
    end
    %2.0 判断是否轨迹终止，若轨迹终止标志L个均为1，则进行轨迹起始
    bisbegin = zeros(targetnum, 1) + 1;
    bisbeginnan = zeros(targetnum, 1);
    for itar = 1:targetnum
        for il = 1:L
            if bisendforward(itar, il) == 0
                bisbegin(itar, 1) = 0;
            end
            if isnan(endZkforward(1+(itar - 1)*2:itar*2, il)) == 0
                bisbeginnan(itar, 1) = 1;
            end
        end
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %终止标志L个均为1且终止存储量测值不为空,进行轨迹起始
            [Z_NNDAbeg(1+(itar - 1)*2:itar*2, 1:L), Z_meanbeg(1+(itar - 1)*2:itar*2, 1:L), RealNum(itar, 1), v_update(itar, :), bisconvergence(itar, :)] = inBegin(L, Lth, T, endZkforward(1+(itar - 1)*2:itar*2, :), A_Model);
            X_Preforward(:, itar) = [Z_meanbeg(1+(itar - 1)*2, L) + v_update(itar, 1) * T; v_update(itar, 1); Z_meanbeg(itar*2, L) + v_update(itar, 2) * T; v_update(itar, 2)];
        end
    end
    %2.1 关联
    %2.1.1 判断前一帧是否两个目标都满足轨迹终止，是的话用简单关联
    isnndaeasy = 0; %判断是否用简单关联
    for itar = 1:targetnum
        if isnan(X_Preforward(1, itar))
            isnndaeasy = 1;
        end
    end
    if isnndaeasy == 1
        [Z] = inNNDAeasy(Z_NNDAforward(:, end), Zk, targetnum, disth);
    else
        %2.1.2  最近邻关联
        Z_Matrix = zeros(2, targetnum); %这一时刻量测值，用于最近邻关联
        Z_Predict = zeros(2, targetnum); %预测的这一时刻量测值，用于最近邻关联
        PZ_Predict = cat(3);
        for itar = 1:targetnum
            Z_Matrix(:, itar) = Zk(1+(itar - 1)*2:itar*2, 1);
            Z_Predict(:, itar) = H * X_Preforward(:, itar); %量测预测
            PZ_Predict = cat(3, PZ_Predict, H*P_Preforward(:, :, itar)*H'+R); %新息协方差
        end
        PZ_Matrix = R;
        [Z, ~, ~] = inNNDA(Z_Matrix, PZ_Matrix, Z_Predict, PZ_Predict);
    end
    %2.2 轨迹终止标志和存储量测值更新
    for itar = 1:targetnum
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %进行了轨迹起始，判断该帧是否满足终止条件时用轨迹起始后的滤波值
            [bisend1(itar, 1), endZk1] = inEnd(Z(1+(itar - 1)*2:itar*2, 1), Z_meanbeg(1+(itar - 1)*2:itar*2, L), disth);
        else %没进行轨迹起始，判断该帧是否满足终止条件时用上一帧滤波值
            [bisend1(itar, 1), endZk1] = inEnd(Z(1+(itar - 1)*2:itar*2, 1), Z_meanforward(1+(itar - 1)*2:itar*2, L), disth);
        end
        bisend(itar, 1:L-1) = bisendforward(itar, 2:L);
        bisend(itar, L) = bisend1(itar, 1);
        endZk(1+(itar - 1)*2:itar*2, 1:L-1) = endZkforward(1+(itar - 1)*2:itar*2, 2:L);
        endZk(1+(itar - 1)*2:itar*2, L) = endZk1;
    end
    %2.3 关联和滤波值输出
    for itar = 1:targetnum
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %终止标志L个均为1,进行了轨迹起始
            Z_NNDA(1+(itar - 1)*2:itar*2, 1:L) = Z_NNDAbeg(1+(itar - 1)*2:itar*2, :);
            Z_mean(1+(itar - 1)*2:itar*2, 1:L) = Z_meanbeg(1+(itar - 1)*2:itar*2, :);
            if (bisconvergence(itar, 1) == 1) && (bisconvergence(itar, 2) == 1) %轨迹起始成功(速度收敛)，将前L-1帧终止标志置空
                for il = 1:L - 1
                    bisend(itar, il) = 0;
                    endZk(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
                end
                Xkk(:, itar) = [Z_meanbeg(1+(itar - 1)*2, end); v_update(itar, 1); Z_meanbeg(itar*2, end); v_update(itar, 2)];
                X_Pre(:, itar) = A_Model * Xkk(:, itar); %预测下一时刻的目标状态
                Pkk = cat(3, Pkk, P_NNSF);
                P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
            else %轨迹起始未成功(速度不收敛)，预测下一时刻目标状态为空
                X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
                Pkk = cat(3, Pkk, P_NNSF);
                P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
            end
        elseif bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 0 %终止标志L个均为1,但量测值全为空
            for il = 1:L
                Z_NNDA(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
                Z_mean(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
            end
            X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
            Pkk = cat(3, Pkk, P_NNSF);
            P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
        else %未进行轨迹起始
            %关联值输出
            Z_NNDA(1+(itar - 1)*2:itar*2, 1:L-1) = Z_NNDAforward(1+(itar - 1)*2:itar*2, 2:L);
            if bisend1(itar, 1) == 1
                Z_NNDA(1+(itar - 1)*2:itar*2, L) = [NaN; NaN];
            else
                Z_NNDA(1+(itar - 1)*2:itar*2, L) = Z(1+(itar - 1)*2:itar*2, 1);
            end
            %滤波
            Xkk0(:, itar) = [Z_meanforward(1+(itar - 1)*2, L); v_last(itar, 1); Z_meanforward(itar*2, L); v_last(itar, 2)];
            Xkk1(:, itar) = [Z_meanforward(1+(itar - 1)*2, L) + v_last(itar, 1) * 2 * T; v_last(itar, 1); Z_meanforward(itar*2, L) + v_last(itar, 2) * 2 * T; v_last(itar, 2)];
            Zkk(:, itar) = [Z_NNDA(1+(itar - 1)*2, L); v_last(itar, 1); Z_NNDA(itar*2, L); v_last(itar, 2)];
            [Xk] = Mean(Xkk0(:, itar), Xkk1(:, itar), Zkk(:, itar), A_Model);
            Xkk(:, itar) = Xk;
            Z_mean(1+(itar - 1)*2:itar*2, 1:L-1) = Z_meanforward(1+(itar - 1)*2:itar*2, 2:L);
            Z_mean(1+(itar - 1)*2, L) = Xkk(1, itar);
            Z_mean(itar*2, L) = Xkk(3, itar);
            %预测,更新vx和vy（卡尔曼滤波需要vx和vy，在有两个点关联后求vx和vy，不断更新）
            v_update(itar, 1) = (Z_mean(1+(itar - 1)*2, L) - Z_mean(1+(itar - 1)*2, L-1)) / T;
            v_update(itar, 2) = (Z_mean(itar*2, L) - Z_mean(itar*2, L-1)) / T;
            if abs(v_update(itar, 1)-v_last(itar, 1)) > v_last(itar, 1) / 2 %如果计算得到速度与历史速度相差过大，则认为出现野值点，不更新速度
                Xkk(2, itar) = v_last(itar, 1);
                Xkk(4, itar) = v_last(itar, 2);
            else
                Xkk(2, itar) = 0.7 * v_update(itar, 1) + 0.3 * v_last(itar, 1);
                Xkk(4, itar) = 0.7 * v_update(itar, 2) + 0.3 * v_last(itar, 2);
            end
            v_update(itar, 1) = Xkk(2, itar);
            v_update(itar, 2) = Xkk(4, itar);
            X_Pre(:, itar) = A_Model * Xkk(:, itar); %预测下一时刻的目标状态
            Pkk = cat(3, Pkk, P_NNSF);
            P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
        end
    end
end
end

%%%该功能函数用到的内部功能函数
function [Z] = inNNDAeasy(Z_FORWARD, Zk, targetnum, disth)
%%%简单关联,通过计算当前帧与前一帧的距离来判断是否属于同一目标
%输入：Z_FORWARD(targetnum*2,1)，前一帧关联结果
%       Zk(targetnum*2,1)，当前帧量测结果
%       targetnum目标个数
%       disth同一目标两点间间距阈值
%输出：Z(targetnum*2,1)，当前帧关联结果
%
Z = zeros(targetnum*2, 1);
d = zeros(targetnum, 1);
Z_FORWARDnotnan = zeros(targetnum, 2);
Zknotnan = zeros(targetnum, 2);
ind_nan0 = find(~isnan(Z_FORWARD)); %前一帧非NaN的点数所在位置
ind_nan1 = find(~isnan(Zk)); %当前帧非NaN的点数所在位置
if isempty(ind_nan0) || isempty(ind_nan1) %若前一帧或当前帧全是NaN,直接输出
    Z = Zk;
else %前一帧和当前帧都有非NaN值
    for izf = 1:length(ind_nan0) / 2
        Z_FORWARDnotnan(izf, :) = [Z_FORWARD(ind_nan0(1+(izf - 1)*2)), Z_FORWARD(ind_nan0(izf*2))]; %存前一帧非NaN的点，2表示xy坐标
    end
    for izk = 1:length(ind_nan1) / 2
        Zknotnan(izk, :) = [Zk(ind_nan1(1+(izk - 1)*2)), Zk(ind_nan1(izk*2))]; %存当前帧非NaN的点，2表示xy坐标
    end
    if length(ind_nan0) / 2 == 1 && length(ind_nan1) / 2 == 1 %前一帧和当前帧都只有一个非NaN点
        dis = (Zknotnan(1, 1) - Z_FORWARDnotnan(1, 1))^2 + (Zknotnan(1, 2) - Z_FORWARDnotnan(1, 2))^2;
        if dis < disth^2 %前一帧和当前帧是同一目标
            Z(ind_nan0(1), 1) = Zknotnan(1, 1);
            Z(ind_nan0(2), 1) = Zknotnan(1, 2);
        else %前一帧和当前帧不是同一目标，将当前帧放到不是前一帧目标所在行
            if (ind_nan0(1) + 2) <= targetnum * 2
                Z(ind_nan0(1)+2, 1) = Zknotnan(1, 1);
                Z(ind_nan0(2)+2, 1) = Zknotnan(1, 2);
            else
                Z(ind_nan0(1)-2, 1) = Zknotnan(1, 1);
                Z(ind_nan0(2)-2, 1) = Zknotnan(1, 2);
            end
        end
    else %前一帧和当前帧有多余一个的非NaN点
        if length(ind_nan0) > length(ind_nan1) %前一帧非NaN点比当前帧多，先循环当前帧，再循环前一帧
            for j = 1:length(ind_nan1) / 2
                for i = 1:length(ind_nan0) / 2
                    d(i, 1) = (Zknotnan(j, 1) - Z_FORWARDnotnan(i, 1))^2 + (Zknotnan(j, 2) - Z_FORWARDnotnan(i, 2))^2;
                end
                [xm, ~] = find(d == min(d));
                Z(ind_nan0(1+(xm - 1)*2), 1) = Zknotnan(j, 1);
                Z(ind_nan0(xm*2), 1) = Zknotnan(j, 2);
            end
        else %前一帧非NaN点比当前帧少，先循环前一帧，再循环当前帧
            for i = 1:length(ind_nan0) / 2
                for j = 1:length(ind_nan1) / 2
                    d(j, 1) = (Zknotnan(j, 1) - Z_FORWARDnotnan(i, 1))^2 + (Zknotnan(j, 2) - Z_FORWARDnotnan(i, 2))^2;
                end
                [xm, ~] = find(d == min(d));
                Z(ind_nan0(1+(i - 1)*2), 1) = Zknotnan(xm, 1);
                Z(ind_nan0(i*2), 1) = Zknotnan(xm, 2);
            end
        end
        if length(ind_nan0) < length(ind_nan1) %前一帧非NaN点比当前帧少，将当前帧未匹配的值放到不是前一帧非NaN点所在位置
            ii2 = 0;
            for j = 1:ind_nan1(end) / 2
                if j ~= xm
                    ii2 = ii2 + 1;
                    if (ind_nan0(1) + 2 * ii2) <= targetnum * 2
                        Z(ind_nan0(1)+2*ii2, 1) = Zknotnan(j, 1);
                        Z(ind_nan0(2)+2*ii2, 1) = Zknotnan(j, 2);
                    else
                        Z(ind_nan0(1)-2*ii2, 1) = Zknotnan(j, 1);
                        Z(ind_nan0(2)-2*ii2, 1) = Zknotnan(j, 2);
                    end
                end
            end
        end
    end
    for itar = 1:targetnum %匹配结束后找到未匹配的前一帧目标，将当前帧关联值置NaN
        if Z(itar*2, 1) == 0
            Z(1+(itar - 1)*2, 1) = NaN;
            Z(itar*2, 1) = NaN;
        end
    end
end
end
function [Z_NNDA, Z_mean, RealNum, v_update, bisconvergence] = inBegin(L, Lth, T, endZkforward, A_Model)
%%判断轨迹是否起始
%输入:itarnum轨迹终止的目标号
%      L轨迹起始判断L个点，Lth表示判断是否收敛的阈值,
%      disth目标相邻两点间距最大值，T采样间隔
%      endZkforward(2,1)，前L个轨迹终止量测值
%       A_Model运动状态模型
%输出：Z_mean(2,1),滤波后结果
%      v_update(1,2)，新目标计算更新后的目标速度，2表示vx和vy
%      bisconvergence(1,2)表示新目标前L个点是否收敛（值为1表示收敛，值为0表示未收敛，NaN表示不是第L帧），2表示vx和vy
%

%计算vx和vy，用速度众数求平均作为初始速度，判断前L帧是否收敛
%%%计算速度
bisconvergence = zeros(1, 2); %速度收敛标志
Vx = zeros(1); %前L个点计算得到的L*(L-1)/2个速度
Vy = zeros(1);
Zkxx = endZkforward(1, :);
Zkyy = endZkforward(2, :);
iv = 1;
%通过前L个点计算L*(L-1)/2个速度
for il = 1:size(Zkxx, 2)
    for jl = il + 1:1:size(Zkxx, 2)
        vx = (Zkxx(1, jl) - Zkxx(1, il)) / (T * (jl - il));
        vy = (Zkyy(1, jl) - Zkyy(1, il)) / (T * (jl - il));
        if isnan(vx)
            continue;
        end
        Vx(iv) = vx;
        Vy(iv) = vy;
        iv = iv + 1;
    end
end
%找到速度众数作为速度初始值
[v_update(1, 1), modenum1] = modemeanv(Vx);
[v_update(1, 2), modenum2] = modemeanv(Vy);
if modenum1 < Lth
    bisconvergence(1, 1) = 0; %该条轨迹前L个点不收敛
else
    bisconvergence(1, 1) = 1;
end
if modenum2 < Lth
    bisconvergence(1, 2) = 0; %该条轨迹前L个点不收敛
else
    bisconvergence(1, 2) = 1;
end
%%%滤波
Z_NNDA = zeros(2, 1);
Z_mean = zeros(2, 1);
RealNum = zeros(1, 1);
Xkk0 = zeros(4, 1);
Xkk1 = zeros(4, 1);
Zkk = zeros(4, 1);
Xkk = zeros(4, 1); %当前时刻目标状态（滤波后）
if (bisconvergence(1, 1) == 1) && (bisconvergence(1, 2) == 1) %该条轨迹前L个点收敛,进行滤波
    for iend = 1:L
        Zk = endZkforward(1:2, iend);
        if iend == 1
            Z_mean = Zk;
            RealNum(1, 1) = length(find(~isnan(Zk(1, 1))));
            %1.2 第二帧到第八帧，进行简单关联和滤波
        elseif 1 < iend
            forwardNum = iend - 1;
            RealNum(1, 1) = length(find(~isnan(Zk(1, 1)))) + RealNumforward(1, 1);
            %%%1.2.3 判断是否进行滤波
            islvbo = 1;
            if RealNum(1, 1) < 2
                islvbo = 0;
            end
            if islvbo == 0 %不够2个点，无法计算vxvy，不进行滤波
                Z_mean(1:2, 1:forwardNum) = Z_meanforward(1:2, 1:forwardNum);
                Z_mean(1:2, forwardNum+1) = Zk;
            else %够2个点，计算vxvy
                Z_mean(1, 1:forwardNum) = Z_meanforward(1, 1:forwardNum);
                Z_mean(2, 1:forwardNum) = Z_meanforward(2, 1:forwardNum);
                ind_vx = find(~isnan(Z_meanforward(1, :)));
                ind_vy = find(~isnan(Z_meanforward(2, :)));
                if RealNum(1, 1) == 2 %第一次够2个点，不滤波
                    Z_mean(1, forwardNum+1) = Z_meanforward(1, ind_vx(end)) + (forwardNum + 1 - ind_vx(end)) * v_update(1, 1) * T;
                    Z_mean(2, forwardNum+1) = Z_meanforward(2, ind_vy(end)) + (forwardNum + 1 - ind_vy(end)) * v_update(1, 2) * T;
                    Xkk(1, 1) = Z_mean(1, forwardNum+1);
                    Xkk(3, 1) = Z_mean(2, forwardNum+1);
                    Xkk(2, 1) = v_update(1, 1);
                    Xkk(4, 1) = v_update(1, 1);
                else %前几帧至少有3个有效点，进行滤波
                    Xkk0(:, 1) = [Z_meanforward(1, forwardNum); v_update(1, 1); Z_meanforward(2, forwardNum); v_update(1, 2)];
                    Xkk1(:, 1) = [Z_meanforward(1, forwardNum) + v_update(1, 1) * 2 * T; v_update(1, 1); Z_meanforward(2, forwardNum) + v_update(1, 2) * 2 * T; v_update(1, 2)];
                    Zkk(:, 1) = [endZkforward(1, forwardNum+1); v_update(1, 1); endZkforward(2, forwardNum+1); v_update(1, 2)];
                    [Xk] = Mean(Xkk0(:, 1), Xkk1(:, 1), Zkk(:, 1), A_Model);
                    Xkk(:, 1) = Xk;
                    Z_mean(1, forwardNum+1) = Xkk(1, 1);
                    Z_mean(2, forwardNum+1) = Xkk(3, 1);
                end
            end
        end
        Z_meanforward = Z_mean;
        RealNumforward = RealNum;
    end
    Z_NNDA = endZkforward(1:2, :);
else
    for iend = 1:L
        Z_NNDA(:, iend) = [NaN; NaN];
        Z_mean(:, iend) = [NaN; NaN];
        RealNum(1, 1) = 0;
    end
end
end
function [bisend, endZk] = inEnd(Z_NNDA, Zk, disth)
%%判断轨迹是否终止
%输入：Z_NNDA(2,1)，当前帧关联结果
%      Zk(2,1)，上一帧关联结果
%       disth同一目标两点间间距阈值
%输出：bisend(1,1),判断该点是否终止
%       endZk(2,1),存判断为终止的量测点
%
bisend = zeros(1, 1);
d = (Z_NNDA(1, 1) - Zk(1, 1))^2 + (Z_NNDA(2, 1) - Zk(2, 1))^2;
if d > disth^2 || isnan(d)
    bisend(1, 1) = 1;
    endZk(1:2, 1) = Z_NNDA(1:2, 1);
else
    bisend(1, 1) = 0;
    endZk(1:2, 1) = [NaN; NaN];
end
end

function [Z, P, ischange] = inNNDA(Z_Matrix, PZ_Matrix, Z_Predict, PZ_Predict)
% 最邻近数据关联函数 输入： Z_Matrix：波门内的有效量测值（包括targetnum列） PZ_Matrix：有效量测值的误差方差阵（1个目标）
% Z_Predict：量测预测值（包括targetnum列） PZ_Predict：量测预测值的误差方差阵（targetnum个目标） 输出：
% Z：按照统计距离最近原则关联上的量测值（targetnum*2行）
% P：关联上的量测值对应的协方差
P = cat(3);
ischange = zeros(1, size(Z_Predict, 2)); %判断关联位置有无变化
d = zeros(1, size(Z_Predict, 2));
Z = zeros(size(Z_Predict, 2)*2, 1);
D = zeros(1);
%传入量测值NaN,判断非NaN的
if sum(sum(1*(isnan(Z_Matrix(1, :))))) ~= 0
    %找到不是nan的数
    ind_nan = find(~isnan(Z_Matrix));
    if isempty(ind_nan) %全为NaN,直接输出NaN
        for i = 1:size(Z_Predict, 2)
            Z(1+(i - 1)*2:i*2, 1) = [NaN; NaN];
            P = cat(3, P, PZ_Matrix);
            ischange(1, 1:size(Z_Predict, 2)) = 1:size(Z_Predict, 2);
        end
    else %不全为NaN,对不为NaN的点进行关联
        for itar = 1:size(Z_Predict, 2)
            e = Z_Matrix(ind_nan) - Z_Predict(:, itar);
            S = PZ_Predict(:, :, itar) + PZ_Matrix; %对应协方差（X、R、Q互不相关条件下）
            D(:, itar) = e' / S * e;
            P = cat(3, P, PZ_Matrix);
        end
        Z(1:2, 1) = Z_Matrix(ind_nan)';
        ischange(1, itar) = 1;
        d = D(:, 1);
        for i = 2:1:size(Z_Predict, 2)
            if D(:, i) < d
                d = D(:, i);
                Z(1+(i - 1)*2:i*2, 1) = Z_Matrix(ind_nan)';
                ischange(1, itar) = i;
                Z(1:2, 1) = [NaN; NaN];
            else
                Z(1+(i - 1)*2:i*2, 1) = [NaN; NaN];
            end
        end
    end
else
    for itar = 1:size(Z_Predict, 2)
        for i = 1:size(Z_Matrix, 2)
            e = Z_Matrix(:, i) - Z_Predict(:, itar); %每个量测与预测值的距离
            S = PZ_Predict(:, :, itar) + PZ_Matrix; %对应协方差（X、R、Q互不相关条件下）
            D(:, i) = e' / S * e;
        end
        Z(1+(itar - 1)*2:itar*2, 1) = Z_Matrix(:, 1);
        ischange(:, itar) = 1;
        PP = PZ_Matrix;
        d(:, itar) = D(:, 1);
        for i = 2:1:size(Z_Matrix, 2)
            if D(:, i) < d(:, itar)
                d(:, itar) = D(:, i);
                Z(1+(itar - 1)*2:itar*2, 1) = Z_Matrix(:, i);
                ischange(:, itar) = i;
                PP = PZ_Matrix;
            end
        end
        P = cat(3, P, PP);
    end
    if ischange(1, 1) == ischange(1, 2)
        for itar = 1:size(Z_Predict, 2)
            if itar ~= ischange(1, 1)
                if d(1, 1) < d(1, 2)
                    Z(1+(itar - 1)*2:itar*2, 1) = Z_Matrix(:, 2);
                    Z(1+(ischange(1, 1) - 1)*2:ischange(1, 1)*2, 1) = Z_Matrix(:, 1);
                else
                    Z(1+(itar - 1)*2:itar*2, 1) = Z_Matrix(:, 1);
                    Z(1+(ischange(1, 1) - 1)*2:ischange(1, 1)*2, 1) = Z_Matrix(:, 2);
                end
            end
        end
    end
end
end

function [v_update, modenum] = modemeanv(v_forward)
%找到整数部分的众数后，将整数部分等于众数的小数求均值得到最终众数
%输入：v_forward是1*n列的小数
%输出：v_update众数的均值，modenum众数的个数
Mv = mode(floor(v_forward));
Sv = zeros(1, 1);
isv = 1;
for ivforward = 1:size(v_forward, 2)
    if floor(v_forward(1, ivforward)) == Mv
        Sv(1, isv) = v_forward(1, ivforward);
        isv = isv + 1;
    end
end
modenum = isv - 1;
v_update = mean(Sv);
end
function [X] = Mean(X_Forward, X_Predict, Z, A)
%三点均值滤波
%       Z--观测数据矢量k时刻
%       A--系统模型状态矩阵
%       X_Forward0--前一次k-1时刻估计状态矢量 X_Predict--由k-1时刻预测的k+1时刻估计状态矢量
%       X--输出估计状态矢量 P--输出估计状态协方差矩阵
if isnan(Z(1, 1))
    X = A * X_Forward;
else
    X = (Z + X_Forward + X_Predict) / 3;
end
end

%����ڹ���(NNDA)������ϡ��Ŀ��������ܺ���˵����
%���룺Zk(targetnum*2,:)��ǰ֡����ֵ��2��ʾxy���꣬targetnumĿ�����
%      L�켣��ʼ�ж�L���㣬Lth��ʾ�ж��Ƿ���������ֵ,
%      disthĿ���������������ֵ��T�������
%      Z_NNDAforward(targetnum*2,L)����һ֡�Ĺ������
%      Z_kalmanforward(targetnum*2,L)����һ֡���˲����
%      forwardNum����һ֡��֡��
%      RealNumforward(targetnum,:)������һ֡Ϊֹ��Ч������ÿ��Ŀ�����������Ч��֮��Ž����˲�
%      v_last(targetnum,2)����һ֡������õ���Ŀ���ٶȣ�2��ʾvx��vy
%      X_Preforward(4,targetnum)��P_Preforward����һ֡�˲�Ԥ���������ڵ�ǰ֡NNDA
%      bisendforward(targetnum,L)��һ֡�켣��ֹ��־
%      endZkforward(targetnum,L)��һ֡�켣��ֹ��Ӧ�洢����ֵ
%�����Z_NNDA(targetnum*2,L)����������Z_kalman(targetnum*2,L)�˲�����,
%      RealNum����ǰ֡Ϊֹ��Ч������
%      v_update(targetnum,2)����ǰ֡��������º��Ŀ���ٶȣ�2��ʾvx��vy
%      X_Pre(4,targetnum)��P_Pre����һ֡�˲�Ԥ���������ڵ�ǰ֡NNDA
%      bisconvergence(targetnum,2)��ʾǰL�����Ƿ�������ֵΪ1��ʾ������ֵΪ0��ʾδ������NaN��ʾ���ǵ�L֡����2��ʾvx��vy
%      bisend(targetnum,L)��ǰ֡�켣��ֹ��־
%      endZk(targetnum,L)��ǰ֡�켣��ֹ��Ӧ�洢����ֵ
%�������̣�
%       0.��������
%       1.������ʼ����ǰ֡��forwardNum+1<=L֡ �򵥹���inNNDAeasy+����ƽ���˲�Mean
%           ��ʱ�����������˲��������Ϊ��ǰ֡��forwardNum+1��
%       2.0 ��ǰ֡��forwardNum+1>L֡���ж��Ƿ�켣��ֹ���ǵĻ����й켣��ʼinBegin��
%       2.1 ��������Ϊ�򵥹���������ڹ���
%       2.2 ���¹켣��ֹ��־�����ɹ����й켣��ʼ������ǰ֮֡ǰ�Ĺ켣��ֹ��Ӧ�洢����ֵ�ÿգ�
%       2.3 �����˲�ֵ����������������˲����������ΪL��������L֮֡��ÿ�ζ�����L���㣬��ͼL����
function [Z_NNDA, Z_mean, RealNum, v_update, X_Pre, P_Pre, bisconvergence, bisend, endZk] = myNNDAQZ(Zk, targetnum, L, Lth, disth, T, Z_NNDAforward, Z_meanforward, forwardNum, RealNumforward, v_last, X_Preforward, P_Preforward, bisendforward, endZkforward)
%*****************************************************************
%          0.��������
%*****************************************************************
A_Model = [1, T, 0, 0; ...
    0, 1, 0, 0; ...
    0, 0, 1, T; ...
    0, 0, 0, 1]; %����Ŀ���˶�ģ��--CVģ��
H = [1, 0, 0, 0; ...
    0, 0, 1, 0]; %����ģ��
Q_Model = 1; %����ģ�͵Ĺ�������
r = 0.1;
R = [r, 0; ...
    0, r]; %��������,ԽС����Խ��
R11 = r;
R22 = r;
R12 = 0;
R21 = 0;
P_NNSF = [R11, R11 / T, R12, R12 / T; ...
    R11 / T, 2 * R11 / T^2, R12 / T, 2 * R12 / T^2; ...
    R21, R21 / T, R22, R22 / T; ...
    R21 / T, 2 * R21 / T^2, R22 / T, 2 * R22 / T^2]; %��ʼ����Э����
G = [T^2 / 2, 0; ...
    T, 0; ...
    0, T^2 / 2; ...
    0, T]; %������Ȩ����

%*****************************************************************
%          1.������ʼǰL֡
%*****************************************************************
Z_NNDA = zeros(targetnum*2, 1);
Z_NNDAbeg = zeros(targetnum*2, 1);
Z_mean = zeros(targetnum*2, 1);
Z_meanbeg = zeros(targetnum*2, 1);
RealNum = zeros(targetnum, 1);
v_update = zeros(targetnum, 2);
X_Pre = zeros(4, targetnum); %�������˲�Ԥ���Ŀ��״̬����������ڹ���
P_Pre = cat(3);
Pkk = cat(3);
bisconvergence = zeros(targetnum, 2);
bisend = zeros(targetnum, L);
bisend1 = zeros(targetnum, 1);
endZk = zeros(targetnum*2, L);
Xkk0 = zeros(4, targetnum);
Xkk1 = zeros(4, targetnum);
Zkk = zeros(4, targetnum);
Xkk = zeros(4, targetnum); %��ǰʱ��Ŀ��״̬���˲���
Zkxy = zeros(targetnum*2, 1); %������ʼtargetnum��Ŀ��L�����x,y����
%1.1 ��һ֡��ֱ�����
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
    %1.2 �ڶ�֡���ڰ�֡�����м򵥹������˲�
elseif 1 <= forwardNum && forwardNum < L
    bisend = bisendforward;
    endZk = endZkforward;
    %%%1.2.1 �򵥹���
    [Zkxy] = inNNDAeasy(Z_NNDAforward(:, end), Zk, targetnum, disth);
    %%%1.2.2 �Դ�����������ȥ�˲���أ���ֵ
    Z_NNDA(:, 1:forwardNum) = Z_NNDAforward;
    Z_NNDA(:, forwardNum+1) = Zkxy;
    for iv = 1:targetnum
        RealNum(iv, 1) = length(find(~isnan(Zkxy(iv*2, 1)))) + RealNumforward(iv, 1);
        bisconvergence(iv, 1) = NaN;
        bisconvergence(iv, 2) = NaN;
    end
    %%%1.2.3 �ж��Ƿ�����˲�
    for itar = 1:targetnum
        islvbo = 1;
        if RealNum(itar, 1) < 2
            islvbo = 0;
        end
        if islvbo == 0 %����2���㣬�޷�����vxvy���������˲�
            Z_mean(1+(itar - 1)*2:itar*2, 1:forwardNum+1) = Z_NNDA(1+(itar - 1)*2:itar*2, 1:forwardNum+1);
            v_update(itar, 1) = NaN;
            v_update(itar, 2) = NaN;
            X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
            P_Pre = cat(3, P_Pre, P_NNSF);
        else %��2���㣬����vxvy
            Z_mean(1+(itar - 1)*2, 1:forwardNum) = Z_meanforward(1+(itar - 1)*2, 1:forwardNum);
            Z_mean(itar*2, 1:forwardNum) = Z_meanforward(itar*2, 1:forwardNum);
            ind_vx = find(~isnan(Z_NNDA(1+(itar - 1)*2, :)));
            ind_vy = find(~isnan(Z_NNDA(itar*2, :)));
            v_update(itar, 1) = Z_NNDA(1+(itar - 1)*2, ind_vx(end)) - Z_NNDA(1+(itar - 1)*2, ind_vx(end-1));
            v_update(itar, 2) = Z_NNDA(itar*2, ind_vx(end)) - Z_NNDA(itar*2, ind_vx(end-1));
            if RealNum(itar, 1) == 2 %��һ�ι�2���㣬���˲�
                Z_mean(1+(itar - 1)*2, forwardNum+1) = Z_NNDA(1+(itar - 1)*2, ind_vx(end)) + (forwardNum + 1 - ind_vx(end)) * v_update(itar, 1) * T;
                Z_mean(itar*2, forwardNum+1) = Z_NNDA(itar*2, ind_vy(end)) + (forwardNum + 1 - ind_vy(end)) * v_update(itar, 2) * T;
                Xkk(1, itar) = Z_mean(1+(itar - 1)*2, forwardNum+1);
                Xkk(3, itar) = Z_mean(itar*2, forwardNum+1);
                Xkk(2, itar) = v_update(itar, 1);
                Xkk(4, itar) = v_update(itar, 1);

            else %ǰ��֡������3����Ч�㣬�����˲�
                Xkk0(:, itar) = [Z_meanforward(1+(itar - 1)*2, forwardNum); v_update(itar, 1); Z_meanforward(itar*2, forwardNum); v_update(itar, 2)];
                Xkk1(:, itar) = [Z_meanforward(1+(itar - 1)*2, forwardNum) + v_update(itar, 1) * 2 * T; v_update(itar, 1); Z_meanforward(itar*2, forwardNum) + v_update(itar, 2) * 2 * T; v_update(itar, 2)];
                Zkk(:, itar) = [Z_NNDA(1+(itar - 1)*2, forwardNum+1); v_update(itar, 1); Z_NNDA(itar*2, forwardNum+1); v_update(itar, 2)];
                [Xk] = Mean(Xkk0(:, itar), Xkk1(:, itar), Zkk(:, itar), A_Model);
                Xkk(:, itar) = Xk;
                Z_mean(1+(itar - 1)*2, forwardNum+1) = Xkk(1, itar);
                Z_mean(itar*2, forwardNum+1) = Xkk(3, itar);
            end
            X_Pre(:, itar) = A_Model * Xkk(:, itar); %Ԥ����һʱ�̵�Ŀ��״̬
            %                 Pkk = cat(3,Pkk,P_NNSF);
            P_Pre(:, :, itar) = A_Model * P_NNSF * A_Model' + G * Q_Model * G';
        end
    end
    %1.2.4 ����L֡ʱ������vx��vy�����ٶ�������ƽ����Ϊ��ʼ�ٶȣ��ж�ǰL֡�Ƿ�����
    if forwardNum == L - 1
        %%%�����ٶ�
        bisconvergence = zeros(targetnum, 2); %�ٶ�������־
        for itar = 1:targetnum
            Vx = zeros(1); %ǰL�������õ���L*(L-1)/2���ٶ�
            Vy = zeros(1);
            Zkxx = Z_NNDAforward(1+(itar - 1)*2, :);
            Zkyy = Z_NNDAforward(itar*2, :);
            iv = 1;
            %ͨ��ǰL�������L*(L-1)/2���ٶ�
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
            %�ҵ��ٶ�������Ϊ�ٶȳ�ʼֵ
            [v_update(itar, 1), modenum1] = modemeanv(Vx);
            [v_update(itar, 2), modenum2] = modemeanv(Vy);
            if modenum1 < Lth
                bisconvergence(itar, 1) = 0; %�����켣ǰL���㲻����
            else
                bisconvergence(itar, 1) = 1;
            end
            if modenum2 < Lth
                bisconvergence(itar, 2) = 0; %�����켣ǰL���㲻����
            else
                bisconvergence(itar, 2) = 1;
            end
        end
    end

    %*****************************************************************
    %          2.����ֵ>=L���㣬��������ڹ������˲���������L��NNDA��
    %*****************************************************************
elseif forwardNum >= L %�ھ�֡���Ժ�,ֱ�ӽ���NNDA�Ϳ������˲�
    %     if forwardNum==37
    %         ssss = 1;
    %     end
    for itar = 1:targetnum
        RealNum(itar, 1) = length(find(~isnan(Zkxy(itar*2, 1)))) + RealNumforward(itar, 1);
        bisconvergence(itar, 1) = NaN;
        bisconvergence(itar, 2) = NaN;
    end
    %2.0 �ж��Ƿ�켣��ֹ�����켣��ֹ��־L����Ϊ1������й켣��ʼ
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
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %��ֹ��־L����Ϊ1����ֹ�洢����ֵ��Ϊ��,���й켣��ʼ
            [Z_NNDAbeg(1+(itar - 1)*2:itar*2, 1:L), Z_meanbeg(1+(itar - 1)*2:itar*2, 1:L), RealNum(itar, 1), v_update(itar, :), bisconvergence(itar, :)] = inBegin(L, Lth, T, endZkforward(1+(itar - 1)*2:itar*2, :), A_Model);
            X_Preforward(:, itar) = [Z_meanbeg(1+(itar - 1)*2, L) + v_update(itar, 1) * T; v_update(itar, 1); Z_meanbeg(itar*2, L) + v_update(itar, 2) * T; v_update(itar, 2)];
        end
    end
    %2.1 ����
    %2.1.1 �ж�ǰһ֡�Ƿ�����Ŀ�궼����켣��ֹ���ǵĻ��ü򵥹���
    isnndaeasy = 0; %�ж��Ƿ��ü򵥹���
    for itar = 1:targetnum
        if isnan(X_Preforward(1, itar))
            isnndaeasy = 1;
        end
    end
    if isnndaeasy == 1
        [Z] = inNNDAeasy(Z_NNDAforward(:, end), Zk, targetnum, disth);
    else
        %2.1.2  ����ڹ���
        Z_Matrix = zeros(2, targetnum); %��һʱ������ֵ����������ڹ���
        Z_Predict = zeros(2, targetnum); %Ԥ�����һʱ������ֵ����������ڹ���
        PZ_Predict = cat(3);
        for itar = 1:targetnum
            Z_Matrix(:, itar) = Zk(1+(itar - 1)*2:itar*2, 1);
            Z_Predict(:, itar) = H * X_Preforward(:, itar); %����Ԥ��
            PZ_Predict = cat(3, PZ_Predict, H*P_Preforward(:, :, itar)*H'+R); %��ϢЭ����
        end
        PZ_Matrix = R;
        [Z, ~, ~] = inNNDA(Z_Matrix, PZ_Matrix, Z_Predict, PZ_Predict);
    end
    %2.2 �켣��ֹ��־�ʹ洢����ֵ����
    for itar = 1:targetnum
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %�����˹켣��ʼ���жϸ�֡�Ƿ�������ֹ����ʱ�ù켣��ʼ����˲�ֵ
            [bisend1(itar, 1), endZk1] = inEnd(Z(1+(itar - 1)*2:itar*2, 1), Z_meanbeg(1+(itar - 1)*2:itar*2, L), disth);
        else %û���й켣��ʼ���жϸ�֡�Ƿ�������ֹ����ʱ����һ֡�˲�ֵ
            [bisend1(itar, 1), endZk1] = inEnd(Z(1+(itar - 1)*2:itar*2, 1), Z_meanforward(1+(itar - 1)*2:itar*2, L), disth);
        end
        bisend(itar, 1:L-1) = bisendforward(itar, 2:L);
        bisend(itar, L) = bisend1(itar, 1);
        endZk(1+(itar - 1)*2:itar*2, 1:L-1) = endZkforward(1+(itar - 1)*2:itar*2, 2:L);
        endZk(1+(itar - 1)*2:itar*2, L) = endZk1;
    end
    %2.3 �������˲�ֵ���
    for itar = 1:targetnum
        if bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 1 %��ֹ��־L����Ϊ1,�����˹켣��ʼ
            Z_NNDA(1+(itar - 1)*2:itar*2, 1:L) = Z_NNDAbeg(1+(itar - 1)*2:itar*2, :);
            Z_mean(1+(itar - 1)*2:itar*2, 1:L) = Z_meanbeg(1+(itar - 1)*2:itar*2, :);
            if (bisconvergence(itar, 1) == 1) && (bisconvergence(itar, 2) == 1) %�켣��ʼ�ɹ�(�ٶ�����)����ǰL-1֡��ֹ��־�ÿ�
                for il = 1:L - 1
                    bisend(itar, il) = 0;
                    endZk(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
                end
                Xkk(:, itar) = [Z_meanbeg(1+(itar - 1)*2, end); v_update(itar, 1); Z_meanbeg(itar*2, end); v_update(itar, 2)];
                X_Pre(:, itar) = A_Model * Xkk(:, itar); %Ԥ����һʱ�̵�Ŀ��״̬
                Pkk = cat(3, Pkk, P_NNSF);
                P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
            else %�켣��ʼδ�ɹ�(�ٶȲ�����)��Ԥ����һʱ��Ŀ��״̬Ϊ��
                X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
                Pkk = cat(3, Pkk, P_NNSF);
                P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
            end
        elseif bisbegin(itar, 1) == 1 && bisbeginnan(itar, 1) == 0 %��ֹ��־L����Ϊ1,������ֵȫΪ��
            for il = 1:L
                Z_NNDA(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
                Z_mean(1+(itar - 1)*2:itar*2, il) = [NaN; NaN];
            end
            X_Pre(:, itar) = [NaN; NaN; NaN; NaN];
            Pkk = cat(3, Pkk, P_NNSF);
            P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
        else %δ���й켣��ʼ
            %����ֵ���
            Z_NNDA(1+(itar - 1)*2:itar*2, 1:L-1) = Z_NNDAforward(1+(itar - 1)*2:itar*2, 2:L);
            if bisend1(itar, 1) == 1
                Z_NNDA(1+(itar - 1)*2:itar*2, L) = [NaN; NaN];
            else
                Z_NNDA(1+(itar - 1)*2:itar*2, L) = Z(1+(itar - 1)*2:itar*2, 1);
            end
            %�˲�
            Xkk0(:, itar) = [Z_meanforward(1+(itar - 1)*2, L); v_last(itar, 1); Z_meanforward(itar*2, L); v_last(itar, 2)];
            Xkk1(:, itar) = [Z_meanforward(1+(itar - 1)*2, L) + v_last(itar, 1) * 2 * T; v_last(itar, 1); Z_meanforward(itar*2, L) + v_last(itar, 2) * 2 * T; v_last(itar, 2)];
            Zkk(:, itar) = [Z_NNDA(1+(itar - 1)*2, L); v_last(itar, 1); Z_NNDA(itar*2, L); v_last(itar, 2)];
            [Xk] = Mean(Xkk0(:, itar), Xkk1(:, itar), Zkk(:, itar), A_Model);
            Xkk(:, itar) = Xk;
            Z_mean(1+(itar - 1)*2:itar*2, 1:L-1) = Z_meanforward(1+(itar - 1)*2:itar*2, 2:L);
            Z_mean(1+(itar - 1)*2, L) = Xkk(1, itar);
            Z_mean(itar*2, L) = Xkk(3, itar);
            %Ԥ��,����vx��vy���������˲���Ҫvx��vy�������������������vx��vy�����ϸ��£�
            v_update(itar, 1) = (Z_mean(1+(itar - 1)*2, L) - Z_mean(1+(itar - 1)*2, L-1)) / T;
            v_update(itar, 2) = (Z_mean(itar*2, L) - Z_mean(itar*2, L-1)) / T;
            if abs(v_update(itar, 1)-v_last(itar, 1)) > v_last(itar, 1) / 2 %�������õ��ٶ�����ʷ�ٶ�����������Ϊ����Ұֵ�㣬�������ٶ�
                Xkk(2, itar) = v_last(itar, 1);
                Xkk(4, itar) = v_last(itar, 2);
            else
                Xkk(2, itar) = 0.7 * v_update(itar, 1) + 0.3 * v_last(itar, 1);
                Xkk(4, itar) = 0.7 * v_update(itar, 2) + 0.3 * v_last(itar, 2);
            end
            v_update(itar, 1) = Xkk(2, itar);
            v_update(itar, 2) = Xkk(4, itar);
            X_Pre(:, itar) = A_Model * Xkk(:, itar); %Ԥ����һʱ�̵�Ŀ��״̬
            Pkk = cat(3, Pkk, P_NNSF);
            P_Pre(:, :, itar) = A_Model * Pkk(:, :, itar) * A_Model' + G * Q_Model * G';
        end
    end
end
end

%%%�ù��ܺ����õ����ڲ����ܺ���
function [Z] = inNNDAeasy(Z_FORWARD, Zk, targetnum, disth)
%%%�򵥹���,ͨ�����㵱ǰ֡��ǰһ֡�ľ������ж��Ƿ�����ͬһĿ��
%���룺Z_FORWARD(targetnum*2,1)��ǰһ֡�������
%       Zk(targetnum*2,1)����ǰ֡������
%       targetnumĿ�����
%       disthͬһĿ�����������ֵ
%�����Z(targetnum*2,1)����ǰ֡�������
%
Z = zeros(targetnum*2, 1);
d = zeros(targetnum, 1);
Z_FORWARDnotnan = zeros(targetnum, 2);
Zknotnan = zeros(targetnum, 2);
ind_nan0 = find(~isnan(Z_FORWARD)); %ǰһ֡��NaN�ĵ�������λ��
ind_nan1 = find(~isnan(Zk)); %��ǰ֡��NaN�ĵ�������λ��
if isempty(ind_nan0) || isempty(ind_nan1) %��ǰһ֡��ǰ֡ȫ��NaN,ֱ�����
    Z = Zk;
else %ǰһ֡�͵�ǰ֡���з�NaNֵ
    for izf = 1:length(ind_nan0) / 2
        Z_FORWARDnotnan(izf, :) = [Z_FORWARD(ind_nan0(1+(izf - 1)*2)), Z_FORWARD(ind_nan0(izf*2))]; %��ǰһ֡��NaN�ĵ㣬2��ʾxy����
    end
    for izk = 1:length(ind_nan1) / 2
        Zknotnan(izk, :) = [Zk(ind_nan1(1+(izk - 1)*2)), Zk(ind_nan1(izk*2))]; %�浱ǰ֡��NaN�ĵ㣬2��ʾxy����
    end
    if length(ind_nan0) / 2 == 1 && length(ind_nan1) / 2 == 1 %ǰһ֡�͵�ǰ֡��ֻ��һ����NaN��
        dis = (Zknotnan(1, 1) - Z_FORWARDnotnan(1, 1))^2 + (Zknotnan(1, 2) - Z_FORWARDnotnan(1, 2))^2;
        if dis < disth^2 %ǰһ֡�͵�ǰ֡��ͬһĿ��
            Z(ind_nan0(1), 1) = Zknotnan(1, 1);
            Z(ind_nan0(2), 1) = Zknotnan(1, 2);
        else %ǰһ֡�͵�ǰ֡����ͬһĿ�꣬����ǰ֡�ŵ�����ǰһ֡Ŀ��������
            if (ind_nan0(1) + 2) <= targetnum * 2
                Z(ind_nan0(1)+2, 1) = Zknotnan(1, 1);
                Z(ind_nan0(2)+2, 1) = Zknotnan(1, 2);
            else
                Z(ind_nan0(1)-2, 1) = Zknotnan(1, 1);
                Z(ind_nan0(2)-2, 1) = Zknotnan(1, 2);
            end
        end
    else %ǰһ֡�͵�ǰ֡�ж���һ���ķ�NaN��
        if length(ind_nan0) > length(ind_nan1) %ǰһ֡��NaN��ȵ�ǰ֡�࣬��ѭ����ǰ֡����ѭ��ǰһ֡
            for j = 1:length(ind_nan1) / 2
                for i = 1:length(ind_nan0) / 2
                    d(i, 1) = (Zknotnan(j, 1) - Z_FORWARDnotnan(i, 1))^2 + (Zknotnan(j, 2) - Z_FORWARDnotnan(i, 2))^2;
                end
                [xm, ~] = find(d == min(d));
                Z(ind_nan0(1+(xm - 1)*2), 1) = Zknotnan(j, 1);
                Z(ind_nan0(xm*2), 1) = Zknotnan(j, 2);
            end
        else %ǰһ֡��NaN��ȵ�ǰ֡�٣���ѭ��ǰһ֡����ѭ����ǰ֡
            for i = 1:length(ind_nan0) / 2
                for j = 1:length(ind_nan1) / 2
                    d(j, 1) = (Zknotnan(j, 1) - Z_FORWARDnotnan(i, 1))^2 + (Zknotnan(j, 2) - Z_FORWARDnotnan(i, 2))^2;
                end
                [xm, ~] = find(d == min(d));
                Z(ind_nan0(1+(i - 1)*2), 1) = Zknotnan(xm, 1);
                Z(ind_nan0(i*2), 1) = Zknotnan(xm, 2);
            end
        end
        if length(ind_nan0) < length(ind_nan1) %ǰһ֡��NaN��ȵ�ǰ֡�٣�����ǰ֡δƥ���ֵ�ŵ�����ǰһ֡��NaN������λ��
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
    for itar = 1:targetnum %ƥ��������ҵ�δƥ���ǰһ֡Ŀ�꣬����ǰ֡����ֵ��NaN
        if Z(itar*2, 1) == 0
            Z(1+(itar - 1)*2, 1) = NaN;
            Z(itar*2, 1) = NaN;
        end
    end
end
end
function [Z_NNDA, Z_mean, RealNum, v_update, bisconvergence] = inBegin(L, Lth, T, endZkforward, A_Model)
%%�жϹ켣�Ƿ���ʼ
%����:itarnum�켣��ֹ��Ŀ���
%      L�켣��ʼ�ж�L���㣬Lth��ʾ�ж��Ƿ���������ֵ,
%      disthĿ���������������ֵ��T�������
%      endZkforward(2,1)��ǰL���켣��ֹ����ֵ
%       A_Model�˶�״̬ģ��
%�����Z_mean(2,1),�˲�����
%      v_update(1,2)����Ŀ�������º��Ŀ���ٶȣ�2��ʾvx��vy
%      bisconvergence(1,2)��ʾ��Ŀ��ǰL�����Ƿ�������ֵΪ1��ʾ������ֵΪ0��ʾδ������NaN��ʾ���ǵ�L֡����2��ʾvx��vy
%

%����vx��vy�����ٶ�������ƽ����Ϊ��ʼ�ٶȣ��ж�ǰL֡�Ƿ�����
%%%�����ٶ�
bisconvergence = zeros(1, 2); %�ٶ�������־
Vx = zeros(1); %ǰL�������õ���L*(L-1)/2���ٶ�
Vy = zeros(1);
Zkxx = endZkforward(1, :);
Zkyy = endZkforward(2, :);
iv = 1;
%ͨ��ǰL�������L*(L-1)/2���ٶ�
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
%�ҵ��ٶ�������Ϊ�ٶȳ�ʼֵ
[v_update(1, 1), modenum1] = modemeanv(Vx);
[v_update(1, 2), modenum2] = modemeanv(Vy);
if modenum1 < Lth
    bisconvergence(1, 1) = 0; %�����켣ǰL���㲻����
else
    bisconvergence(1, 1) = 1;
end
if modenum2 < Lth
    bisconvergence(1, 2) = 0; %�����켣ǰL���㲻����
else
    bisconvergence(1, 2) = 1;
end
%%%�˲�
Z_NNDA = zeros(2, 1);
Z_mean = zeros(2, 1);
RealNum = zeros(1, 1);
Xkk0 = zeros(4, 1);
Xkk1 = zeros(4, 1);
Zkk = zeros(4, 1);
Xkk = zeros(4, 1); %��ǰʱ��Ŀ��״̬���˲���
if (bisconvergence(1, 1) == 1) && (bisconvergence(1, 2) == 1) %�����켣ǰL��������,�����˲�
    for iend = 1:L
        Zk = endZkforward(1:2, iend);
        if iend == 1
            Z_mean = Zk;
            RealNum(1, 1) = length(find(~isnan(Zk(1, 1))));
            %1.2 �ڶ�֡���ڰ�֡�����м򵥹������˲�
        elseif 1 < iend
            forwardNum = iend - 1;
            RealNum(1, 1) = length(find(~isnan(Zk(1, 1)))) + RealNumforward(1, 1);
            %%%1.2.3 �ж��Ƿ�����˲�
            islvbo = 1;
            if RealNum(1, 1) < 2
                islvbo = 0;
            end
            if islvbo == 0 %����2���㣬�޷�����vxvy���������˲�
                Z_mean(1:2, 1:forwardNum) = Z_meanforward(1:2, 1:forwardNum);
                Z_mean(1:2, forwardNum+1) = Zk;
            else %��2���㣬����vxvy
                Z_mean(1, 1:forwardNum) = Z_meanforward(1, 1:forwardNum);
                Z_mean(2, 1:forwardNum) = Z_meanforward(2, 1:forwardNum);
                ind_vx = find(~isnan(Z_meanforward(1, :)));
                ind_vy = find(~isnan(Z_meanforward(2, :)));
                if RealNum(1, 1) == 2 %��һ�ι�2���㣬���˲�
                    Z_mean(1, forwardNum+1) = Z_meanforward(1, ind_vx(end)) + (forwardNum + 1 - ind_vx(end)) * v_update(1, 1) * T;
                    Z_mean(2, forwardNum+1) = Z_meanforward(2, ind_vy(end)) + (forwardNum + 1 - ind_vy(end)) * v_update(1, 2) * T;
                    Xkk(1, 1) = Z_mean(1, forwardNum+1);
                    Xkk(3, 1) = Z_mean(2, forwardNum+1);
                    Xkk(2, 1) = v_update(1, 1);
                    Xkk(4, 1) = v_update(1, 1);
                else %ǰ��֡������3����Ч�㣬�����˲�
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
%%�жϹ켣�Ƿ���ֹ
%���룺Z_NNDA(2,1)����ǰ֡�������
%      Zk(2,1)����һ֡�������
%       disthͬһĿ�����������ֵ
%�����bisend(1,1),�жϸõ��Ƿ���ֹ
%       endZk(2,1),���ж�Ϊ��ֹ�������
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
% ���ڽ����ݹ������� ���룺 Z_Matrix�������ڵ���Ч����ֵ������targetnum�У� PZ_Matrix����Ч����ֵ��������1��Ŀ�꣩
% Z_Predict������Ԥ��ֵ������targetnum�У� PZ_Predict������Ԥ��ֵ��������targetnum��Ŀ�꣩ �����
% Z������ͳ�ƾ������ԭ������ϵ�����ֵ��targetnum*2�У�
% P�������ϵ�����ֵ��Ӧ��Э����
P = cat(3);
ischange = zeros(1, size(Z_Predict, 2)); %�жϹ���λ�����ޱ仯
d = zeros(1, size(Z_Predict, 2));
Z = zeros(size(Z_Predict, 2)*2, 1);
D = zeros(1);
%��������ֵNaN,�жϷ�NaN��
if sum(sum(1*(isnan(Z_Matrix(1, :))))) ~= 0
    %�ҵ�����nan����
    ind_nan = find(~isnan(Z_Matrix));
    if isempty(ind_nan) %ȫΪNaN,ֱ�����NaN
        for i = 1:size(Z_Predict, 2)
            Z(1+(i - 1)*2:i*2, 1) = [NaN; NaN];
            P = cat(3, P, PZ_Matrix);
            ischange(1, 1:size(Z_Predict, 2)) = 1:size(Z_Predict, 2);
        end
    else %��ȫΪNaN,�Բ�ΪNaN�ĵ���й���
        for itar = 1:size(Z_Predict, 2)
            e = Z_Matrix(ind_nan) - Z_Predict(:, itar);
            S = PZ_Predict(:, :, itar) + PZ_Matrix; %��ӦЭ���X��R��Q������������£�
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
            e = Z_Matrix(:, i) - Z_Predict(:, itar); %ÿ��������Ԥ��ֵ�ľ���
            S = PZ_Predict(:, :, itar) + PZ_Matrix; %��ӦЭ���X��R��Q������������£�
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
%�ҵ��������ֵ������󣬽��������ֵ���������С�����ֵ�õ���������
%���룺v_forward��1*n�е�С��
%�����v_update�����ľ�ֵ��modenum�����ĸ���
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
%�����ֵ�˲�
%       Z--�۲�����ʸ��kʱ��
%       A--ϵͳģ��״̬����
%       X_Forward0--ǰһ��k-1ʱ�̹���״̬ʸ�� X_Predict--��k-1ʱ��Ԥ���k+1ʱ�̹���״̬ʸ��
%       X--�������״̬ʸ�� P--�������״̬Э�������
if isnan(Z(1, 1))
    X = A * X_Forward;
else
    X = (Z + X_Forward + X_Predict) / 3;
end
end

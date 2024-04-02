% 使用本地保存的目标轨迹测试滤波函数


load("ini_pos.mat")
ini_pos = ini_pos';
targetnum = 1; %目标个数
L = 8; %轨迹起始判断L个点
Lth = 3; %判断是否收敛的阈值
disth = 100; %目标相邻两点间距最大值，为了避免第一次量测值有NaN时第二个点关联错误
Z_NNDAforward = zeros(targetnum*2, 1); %上一帧的关联结果
Z_kalmanforward = zeros(targetnum*2, 1); %上一帧的滤波结果
v_last = zeros(targetnum, 2); %上一帧数计算得到的目标速度，2表示vx和vy
bisconvergence = zeros(targetnum, 2); %表示前L个点是否收敛，值为1或0或NaN
isconvergence = zeros(1, targetnum); %记录前L个点是否收敛,值为1或0
X_Preforward = zeros(4, targetnum); %上一帧滤波预测结果，用于当前帧NNDA
P_Preforward = cat(3); %上一帧滤波预测结果，用于当前帧NNDA
bisendforward = zeros(targetnum, L); %上一帧轨迹终止标志
endZkforward = zeros(targetnum*2, L); %上一帧轨迹终止对应存储量测值
figure
plot(0, 0, 'k*');
hold on;
for i = 1:size(ini_pos, 2)
    if i == 1
        forwardNum = 0;
        RealNumforward = 0;
        for itar = 1:targetnum
            Z_NNDAforward(:, 1) = NaN;
            Z_kalmanforward(:, 1) = NaN;
            v_last(itar, 1) = NaN;
            v_last(itar, 2) = NaN;
            bisconvergence(itar, 1) = NaN;
            bisconvergence(itar, 2) = NaN;
            X_Preforward(:, itar) = [NaN; NaN; NaN; NaN];
            P_Preforward = cat(3, P_Preforward, NaN);
        end
    end
    [Z_NNDA, Z_kalman, RealNum, v_update, X_Pre, P_Pre, bisconvergence, bisend, endZk] ...
        = myNNDAQZ(ini_pos(:, i), targetnum, L, Lth, disth, T, Z_NNDAforward, Z_kalmanforward, ...
        forwardNum, RealNumforward, v_last, X_Preforward, P_Preforward, bisendforward, endZkforward);
    Z_NNDAforward = Z_NNDA;
    Z_kalmanforward = Z_kalman;
    forwardNum = i;
    RealNumforward = RealNum;
    v_last = v_update;
    X_Preforward = X_Pre;
    P_Preforward = P_Pre;
    bisendforward = bisend;
    endZkforward = endZk;

    %绘图
    ii = 1:size(Z_NNDA, 2);
    if size(Z_NNDA, 2) < L
        plot(Zk(1, ii), Zk(2, ii), 'b.'); %实际测量值
    else
        plot(Zk(1, i-L+ii), Zk(2, i-L+ii), 'b.'); %实际测量值
    end
    plot(Z_NNDA(1, ii), Z_NNDA(2, ii), 'go'); %目标1关联的测量值
    plot(Z_kalman(1, ii), Z_kalman(2, ii), 'g-', 'LineWidth', 2); %目标1滤波值

end
legend('观测点', '量测值', '量测值', '目标1关联量测', '目标1滤波值', '目标2关联量测', '目标2滤波值');
% title('目标运动轨迹');
xlabel('x/m');
ylabel('y/m');
xlim([-200, 1600]);
ylim([-200, 1600]);
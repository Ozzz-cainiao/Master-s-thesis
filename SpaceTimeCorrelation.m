%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\SpaceTimeCorrelation.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-09
% 描述: 时空关联部分的函数
% 输入: 当前周期号，采样周期，解算出的位置，平台位置，两次迭代之间的门限（用于终止）, index 用于第一次解算
% 输出: 最后时空关联完解算出的位置
%**************************************************************************

function res = SpaceTimeCorrelation(start, T, posIn, posPlatForm, D, index)
c = 1500;
len = length(posPlatForm);
dis = zeros(1, len);
tao = zeros(1, len);
nowIndex = zeros(1, len); % 更新之后的周期号
tt = zeros(1, len);  % 保存当前解算结果到各平台的时延
while index == 1 || abs(posIn - res) > D
    % 计算当前解算结果到各个平台的距离
    for i = 1 : len
        dis(i) =  sqrt((posIn(1) - posPlatForm(i, 1))^2 + (posIn(2) - posPlatForm(i, 2))^2);
        tt(i) = dis(i) / c;
    end
    [maxDis, maxPos] = max(dis); % 获取距离最大的值和平台序号
    % 开始计算几个接收阵元之间的时延差
    for i = 1 : len
        tao(i) = abs(tt(maxPos) - tt(i)); % 计算解算位置到平台距离最远的平台和其他平台之间的时延差
        nowIndex(i) = floor(start - tao(i) / T); % 当前周期号减去所差的周期数
    end
    % 更新帧之后重新解算位置  角度序列作为全局变量存在
    % 这里整合一下解算作为一个函数来调用
    res = AOARaw()
end

end
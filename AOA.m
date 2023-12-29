%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\AOA.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-29
% 描述: 纯方位，两两组合+LSM解算目标点
% 输入: 角度数组，平台位置数组
% 输出: 两两组合的定位结果，用到的参量组合
%**************************************************************************

%% 两两组合加LSM解算目标点，针对某一时刻
function [res, loc] = AOA(Zt, node)
len = size(node, 1);
m = 0; % 记录解的个数

for i = 1:len
    for j = i + 1:len
        pos1 = node(i, 1: 2);
        pos2 = node(j, 1: 2);
        alpha1 = Zt(i);
        alpha2 = Zt(j);

        %         % 加上判角条件 和解算直接融合到一起
        %         beta1 = atand((pos_2(2) - pos1(2))/(pos2(1) - pos1(1))); %基线与正东方向夹角
        %         beta2 = atand((pos_1(2) - pos2(2))/(pos1(1) - pos2(1))); %基线与正东方向夹角
        %         if alpha1 - beta1 < lamda && alpha2 - beta2 < lamda
        %             % 直接将该点标记为异常情况
        %
        %
        %             continue;
        %         else
        %             k = k + 1;
        %             res(k, : ) = LSM([alpha1, alpha2], [pos1;pos2]); % 计算结果
        %             loc(k, : ) = [i, j]; % 记录目标位置与所利用的参量的对应关系
        %         end
        % 不带角度预先判决
        [EstX, EstY] = LSM([alpha1, alpha2], [pos1; pos2]); % 计算结果
        if ~isnan(EstY) &&  ~isnan(EstX)
            m = m + 1;
            [res(m, 1), res(m, 2)] = deal(EstX, EstY); % 计算结果
            loc{m, :} = ["theta"+ num2str(i), "theta"+ num2str(j)]; % 记录目标位置与所利用的参量的对应关系        
        end
    end
end
end
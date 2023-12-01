%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\Platform.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-22
% 描述: 定义一个平台类，在这个类中实现平台的一些观测功能，同时便于维护
% 输入:  
% 输出:  
%**************************************************************************


%% 定义平台类
classdef Platform
    properties
        position
        c = 1500
    end

    methods
        % 构造函数
        function obj = Platform(position)
            obj.position = position;
        end

        % 获取目标信息的方法  在这里加上传播时延
        function [angle, distance, t_delay, type, fre] = getTargetInfo(obj, target)
            relative_position = target.Position - obj.position;
%             disp(relative_position);
            distance = norm(relative_position);
            angle = atan2d(relative_position(2), relative_position(1)); % 与x正的夹角
            angle(angle < 0) = angle(angle  < 0) + 360;
            t_delay = distance / obj.c;
            type = target.Type; % 获得信号类型
            if strcmp(type, 'CW')
                fre = target.Frequency; % 获得信号频率
            else
                fre = 0; % 其它信号频率先设置为0
            end

        end
    end
end
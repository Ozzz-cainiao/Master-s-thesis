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
        t = 0 % 作为累计观测时间
    end

    methods (Access = public)
        % 构造函数
        function obj = Platform(position)
            obj.position = position;
        end

        % 获取目标信息的方法  在这里加上传播时延
        function [angle, distance, t_delay, type, fre, obj] = getTargetInfo(obj, target, dt)
            % 根据输入参数调用

            obj.t = obj.t + dt;
            %             obj.t = [obj.t; obj.t(end, :) + dt]
            relative_position = target.Position(end, :) - obj.position(end, :); % 从最后一个数据中更新
            %             disp(relative_position);
            distance = norm(relative_position);
            angle = atan2d(relative_position(2), relative_position(1)); % 与x正的夹角
            %             angle(angle < 0) = angle(angle  < 0) + 360;
            t_delay = distance / obj.c;
            type = target.Type; % 获得信号类型
            if strcmp(type, 'CW')
                %                 fre = target.Frequency; % 获得信号频率
                % 如果目标做匀速或匀加速直线运动，则可以加上多普勒频移
                % 为了简化问题，先假设目标做水平匀速运动 则多普勒频率就是这样计算 公式来源于
                % 单水听器被动测距的信赖域最优化方法 式11
                % 这个t是累计运行时间，需要再加一个特性

                fre = target.Frequency - target.Frequency .* (target.Velocity(1) ...
                    * (target.Position(1, 1) + target.Velocity(1) .* obj.t) ...
                    ./ (1500 * sqrt((target.Position(1, 1) + target.Velocity(1) ...
                    .* obj.t).^2+target.Position(1, 2)^2)));

            else
                fre = 0; % 其它信号频率先设置为0
            end

        end
        % 获取目标信息的方法  在这里加上传播时延
        function [angle, distance, t_delay, type, fre, obj] = getTargetInfoFre(obj, target, dt, k, b)
            % 根据输入参数调用
            obj.t = obj.t + dt;
            %             obj.t = [obj.t; obj.t(end, :) + dt]
            relative_position = target.Position(end, :) - obj.position(end, :); % 从最后一个数据中更新
            %             disp(relative_position);
            distance = norm(relative_position);
            angle = atan2d(relative_position(2), relative_position(1)); % 与x正的夹角
            %             angle(angle < 0) = angle(angle  < 0) + 360;
            t_delay = distance / obj.c;
            type = target.Type; % 获得信号类型
            if strcmp(type, 'CW')
                % fre = target.Frequency; % 获得信号频率
                % 如果目标做匀速或匀加速直线运动，则可以加上多普勒频移
                % 为了简化问题，先假设目标做水平匀速运动 则多普勒频率就是这样计算 公式来源于
                % 单水听器被动测距的信赖域最优化方法 式11
                % 这个t是累计运行时间，需要再加一个特性
                % 需要计算以当前平台为新原点坐标系的x0,y0
                % 计算角度
                theta = atan(k); % 返回弧度值
%                 TT = [cos(theta), sin(theta); -sin(theta), cos(theta)];
                
                % 需要已知目标运动方程 参考https://zhuanlan.zhihu.com/p/283015520
                x0 = (target.Position(1, 1) - obj.position(1)) * cos(theta) + (target.Position(1, 2) - obj.position(2)) * sin(theta); 
                y0 = -(target.Position(1, 1) - obj.position(1)) * sin(theta) + (target.Position(1, 2) - obj.position(2)) * cos(theta);
%                 y0 = -(obj.position(2) - (k * obj.position(1) + b)) * cos(theta); % 这个是正确的
%                 x0 = target.Position(1, 1) * cos(theta) + target.Position(1, 2) * sin(theta) ...
%                     -obj.position(1) * cos(theta) - obj.position(2) * sin(theta);

                v = sqrt(target.Velocity(end, 1).^2+target.Velocity(end, 2).^2);

                %                 v = 8;
                %                 v = target.Velocity(1);
                fre = target.Frequency - target.Frequency * (v * (x0 + v * obj.t) ...
                    / obj.c / sqrt((x0 + v * obj.t)^2 + y0^2));
            else
                fre = 0; % 其它信号频率先设置为0
            end

        end
    end
end
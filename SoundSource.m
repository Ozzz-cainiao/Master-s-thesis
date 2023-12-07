%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\SoundSource.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-21
% 描述: 定义一个声源类
% 输入:
% 输出:
%**************************************************************************

classdef SoundSource
    properties
        Type % 信号类型 ('CW' 或 'Noise')
        Frequency
        Amplitude
        Position % 位置(x,y)，作为一个数组保存起来吧
        Velocity % 速度
        X % [x0;y0;vx;vy]; %作为描述它的运动方程
        Acceleration = [0, 0] % 加速度
        PulseWidth = 1 % 脉宽 (仅适用于 'CW')
        DutyCycle = 0.5 % 脉宽占空比 (仅适用于 'CW')
        F1 = [] % 速度运动矩阵
        F2 = [] % 加速度运动矩阵
        Var = 0 % 均方根  ————这个定义还需要具体去落实
    end

    methods (Access = public)
        % 构造函数1
        %         function obj = SoundSource(type, frequency, amplitude, initialPosition, volicity, acc)
        %             obj.Type = type;
        %             obj.Frequency = frequency;
        %             obj.Amplitude = amplitude;
        %             obj.Position = initialPosition;
        %             obj.Velocity = volicity;
        %             obj.Acceleration = acc;
        %         end
        % 构造函数2 利用F1， F2来更新的
        function obj = SoundSource(type, frequency, amplitude, initialPosition, velocity, f1, f2, a)
            obj.Type = type;
            obj.Frequency = frequency;
            obj.Amplitude = amplitude;
            obj.Position = initialPosition;
            obj.Velocity = velocity;
            obj.F1 = f1;
            obj.F2 = f2;
            obj.Acceleration = a;
%             obj.Var = var;
            obj.X = [initialPosition(1); initialPosition(2); velocity(1); velocity(2);];
        end
        % 方法用于更新位置（假设简单的匀速直线运动）
        function obj = updatePosition(obj)
            %             disp("调用updatePosition")
            % 更新为运动模型形式
            ax = obj.Acceleration * (2 * rand - 1);
            ay = obj.Acceleration * (2 * rand - 1);
            a = [ax, ay]';
            obj.X = [obj.X, obj.F1 * obj.X(:, end) + obj.F2 * a]; % 更新的目标状态
            obj.Position = [obj.Position; [obj.X(1, end), obj.X(2, end)]]; % 更新最新位置
            obj.Velocity = [obj.Velocity; [obj.X(3, end), obj.X(4, end)]]; % 更新最新位置
            % 将新的位置追加到位置数组中
            %             obj.Position = [obj.Position; obj.Position(end, :) + obj.Velocity(end, :) .* delta_time;];
            %             obj.Velocity = [obj.Velocity; obj.Velocity(end, :) + delta_time * obj.Acceleration];
            %  obj.Position = obj.Position + obj.Volicity .* delta_time; % 从上一次的位置中更新
            %  obj.Volicity = obj.Volicity + delta_time * obj.Acceleration;
        end
        % 生成信号的方法
        function signal = generateSignal(obj, time)
            if strcmp(obj.Type, 'CW')
                %   signal = obj.Amplitude * cos(2 * pi * obj.Frequency * time);
                signal = obj.Amplitude * cos(2*pi*obj.Frequency*time) .* (time < time * obj.DutyCycle);
            elseif strcmp(obj.Type, 'Noise')
                signal = obj.Amplitude * randn(size(time));
            elseif strcmp(obj.Type, 'LFM')
                error('需要设置LFM信号的生成函数');
                %                 signal = obj.Amplitude * randn(size(time));
            else
                error('未知的信号类型');
            end
        end
    end
end

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
        Type     % 信号类型 ('CW' 或 'Noise')
        Frequency
        Amplitude
        Position % 位置(x,y)
        Volicity % 速度
        Acceleration    % 加速度
        PulseWidth = 1 % 脉宽 (仅适用于 'CW')
        DutyCycle = 0.5 % 脉宽占空比 (仅适用于 'CW')
    end
    
    methods (Access = public)
        % 构造函数
        function obj = SoundSource(type, frequency, amplitude, initialPosition,volicity, acc)
            obj.Type = type;
            obj.Frequency = frequency;
            obj.Amplitude = amplitude;
            obj.Position = initialPosition;
            obj.Volicity = volicity;
            obj.Acceleration = acc;
        end
        
        % 方法用于更新位置（假设简单的匀速直线运动）
        function obj = updatePosition(obj, delta_time)
%             disp("调用updatePosition")
%             obj.Position
%             obj.Volicity
            obj.Position = obj.Position + obj.Volicity .* delta_time; % 从上一次的位置中更新
            obj.Volicity = obj.Volicity + delta_time * obj.Acceleration;
        end
        % 生成信号的方法
        function signal = generateSignal(obj, time)
            if strcmp(obj.Type, 'CW')
                %   signal = obj.Amplitude * cos(2 * pi * obj.Frequency * time);
                signal = obj.Amplitude * cos(2 * pi * obj.Frequency * time) .* (time < time * obj.DutyCycle);
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

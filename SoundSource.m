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
        Type
        Frequency
        Amplitude
        Position % 位置(x,y)
        Volicity % 速度
    end
    
    methods
        % 构造函数
        function obj = SoundSource(type, frequency, amplitude, initialPosition,volicity)
            obj.Type = type;
            obj.Frequency = frequency;
            obj.Amplitude = amplitude;
            obj.Position = initialPosition;
            obj.Volicity = volicity;
        end
        
        % 方法用于更新位置（假设简单的匀速直线运动）
        function obj = updatePosition(obj, delta_time)
            disp("调用updatePosition")
            obj.Position
            obj.Volicity
            obj.Position = obj.Position + obj.Volicity .* delta_time; % 从上一次的位置中更新
        end
    end
end

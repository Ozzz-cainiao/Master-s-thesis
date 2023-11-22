%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\temp.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-11-21
% 描述: 这是函数功能说明
% 输入:  
% 输出:  
%**************************************************************************
%% 将结构体保存在cell数组中

% 创建一个包含结构体的 cell 数组
cellArray = cell(1, 2);

% 创建一个结构体并赋值
myStruct1.name = 'John';
myStruct1.age = 30;

% 将结构体存储在 cell 数组的第一个位置
cellArray{1} = myStruct1;

% 创建另一个结构体并赋值
myStruct2.name = 'Jane';
myStruct2.age = 28;

% 将结构体存储在 cell 数组的第二个位置
cellArray{2} = myStruct2;
cellArray


%% 创建声源对象

soundSource = SoundSource(90, 'lfm', [1000, 0.5], 0.2, [0, 0], 10, '向右');

% 获取初始位置
initialPosition = soundSource.InitialPosition;

% 更新位置
newPosition = [1, 2];
soundSource.updatePosition(newPosition);

% 更新运动时间
newMotionTime = 20;
soundSource.updateMotionTime(newMotionTime);

% 更新运动方向
newMotionDirection = '向左';
soundSource.updateMotionDirection(newMotionDirection);


a = 10




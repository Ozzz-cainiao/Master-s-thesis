% 定义cell数组
C1 = {[], [3, 4]};  % 一个单元格为空，另一个包含[3, 4]
C2 = {[1, 2], []};  % 一个单元格包含[1, 2]，另一个为空
C3 = {[], []};      % 两个单元格都为空

% 应用转换函数
result1 = cellToNanFilledMatrix4x1(C1);
result2 = cellToNanFilledMatrix4x1(C2);
result3 = cellToNanFilledMatrix4x1(C3);

% 显示结果
disp('结果1:');
disp(result1);

disp('结果2:');
disp(result2);

disp('结果3:');
disp(result3);
function output = cellToNanFilledMatrix4x1(cellArray)
    numCells = numel(cellArray);  % 获取cell数组中的单元格数量
    output = NaN(4, 1);  % 初始化输出矩阵，大小为4x1，填充NaN
    
    currentIndex = 1;
    for i = 1:numCells
        if ~isempty(cellArray{i})
            len = numel(cellArray{i});  % 当前单元格向量的长度
            output(currentIndex:(currentIndex+len-1)) = cellArray{i}';
            currentIndex = currentIndex + len;  % 更新当前索引
        end
    end
end

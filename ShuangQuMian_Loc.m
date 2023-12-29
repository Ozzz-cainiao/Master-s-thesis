%浮标全部换为应答器
close;
clear;
clc;

%% 参数初始化
%阵位
transponders = [0, 0, 200; ...
    4000, 0, 100; ...
    0, 4000, 300];

transponderDelay = [0, 0, 0]; %应答延时/s
transponders_num = length(transponders(:, 1)); %应答器个数

synT = 2; %同步周期
c0 = 1500; %声速

%% 目标参数
xTemp = -2e3:5:6e3; %水平分格
yTemp = -2e3:5:6e3; %垂直分格
[xGrid, yGrid] = meshgrid(xTemp, yTemp); %目标位置（遍历）
[rowNum, colNum] = size(xGrid);
z = 100; %目标深度

%% 计算斜距（注：误差引起斜距的变化，阵位，深度，声速均采用默认值）
slantRangeSingle = zeros(rowNum, colNum); %相对一个应答器斜距
slantRangeMultiple = zeros([transponders_num, size(slantRangeSingle)]); %所有应答器的斜距

disp('计算斜距')
% tic

%阵位误差
transponderError = zeros(size(transponders));
% transponderError=randn(size(transponders));
% transponderError=[-0.5,-0.5,0
%                   0,0,0;
%                   0,0,0;
%                   0,0,0];
% transponderError=[-0.5,-0.5,0.5;
%                   0.5,-0.5,0.5;
%                   0.5,0.5,0.5;
%                   -0.5,0.5,0.5];
realTranPos = transponders + transponderError;

for i_trans = 1:transponders_num %应答器
    for i_row = 1:rowNum %行
        for i_col = 1:colNum %列
            slantRangeSingle(i_row, i_col) = norm([xGrid(i_row, i_col), yGrid(i_row, i_col), z]-realTranPos(i_trans, :), 2);
        end
    end
    slantRangeMultiple(i_trans, :, :) = slantRangeSingle;
end

% toc

%% 解算
slantRange = zeros(transponders_num, 1); %每次计算时用到的斜距
initPosition_Rec1 = zeros(rowNum, colNum, 2); %目标解算位置 2-平面 3-立体

disp('阵位解算')
% tic
% parfor i_row = 1:rowNum
for i_row = 1:rowNum

    for i_col = 1:colNum
        slantRange = slantRangeMultiple(:, i_row, i_col); %斜距
        %slantRange = slantRange+100;

        result = SolTrack(slantRange', ...
            [transponders(1, :), transponders(2, :), transponders(3, :)], ...
            z, 1.0, 4);

        initPosition_Rec1(i_row, i_col, :) = result(1:2);
    end

end
% toc

%偏移量
errLoc = ones(rowNum, colNum) * nan;
for i_row = 1:rowNum
    for i_col = 1:colNum
        errLoc(i_row, i_col) = sqrt((initPosition_Rec1(i_row, i_col, 1) - xGrid(i_row, i_col))^2+(initPosition_Rec1(i_row, i_col, 2) - yGrid(i_row, i_col))^2);
    end
end

errLoc(abs(errLoc) < 1e0) = 1;
errLoc(errLoc ~= 1) = 2;

% load matlab.mat

%% 画图
figure

pcolor(xGrid, yGrid, errLoc)
axis equal
axis([xTemp(1), xTemp(end), yTemp(1), yTemp(end)])
% caxis([0 10])
v = caxis;
shading interp
% colorbar
mymap = [54, 48, 147; ...
    255, 200, 50] / 255;
colormap(mymap)

hold on
plot([-2000, 6000], [0, 0], 'k-', 'LineWidth', 3)
plot([-2000, 6000], [4000, 4000], 'k-', 'LineWidth', 3)
plot([0, 0], [-2000, 6000], 'k-', 'LineWidth', 3)
plot([4000, 4000], [-2000, 6000], 'k-', 'LineWidth', 3)
plot(0, 0, 'g*', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
plot(0, 4000, 'g*', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
plot(4000, 0, 'g*', 'LineWidth', 2, 'MarkerSize', 10);

view(0, 90)
% title(['非同步定位双解适用范围']);
xlabel('X/m');
ylabel('Y/m');
set(gca, 'FontSize', 15)
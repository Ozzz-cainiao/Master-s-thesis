%����ȫ����ΪӦ����
close;
clear;
clc;

%% ������ʼ��
%��λ
transponders = [0, 0, 200; ...
    4000, 0, 100; ...
    0, 4000, 300];

transponderDelay = [0, 0, 0]; %Ӧ����ʱ/s
transponders_num = length(transponders(:, 1)); %Ӧ��������

synT = 2; %ͬ������
c0 = 1500; %����

%% Ŀ�����
xTemp = -2e3:5:6e3; %ˮƽ�ָ�
yTemp = -2e3:5:6e3; %��ֱ�ָ�
[xGrid, yGrid] = meshgrid(xTemp, yTemp); %Ŀ��λ�ã�������
[rowNum, colNum] = size(xGrid);
z = 100; %Ŀ�����

%% ����б�ࣨע���������б��ı仯����λ����ȣ����پ�����Ĭ��ֵ��
slantRangeSingle = zeros(rowNum, colNum); %���һ��Ӧ����б��
slantRangeMultiple = zeros([transponders_num, size(slantRangeSingle)]); %����Ӧ������б��

disp('����б��')
% tic

%��λ���
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

for i_trans = 1:transponders_num %Ӧ����
    for i_row = 1:rowNum %��
        for i_col = 1:colNum %��
            slantRangeSingle(i_row, i_col) = norm([xGrid(i_row, i_col), yGrid(i_row, i_col), z]-realTranPos(i_trans, :), 2);
        end
    end
    slantRangeMultiple(i_trans, :, :) = slantRangeSingle;
end

% toc

%% ����
slantRange = zeros(transponders_num, 1); %ÿ�μ���ʱ�õ���б��
initPosition_Rec1 = zeros(rowNum, colNum, 2); %Ŀ�����λ�� 2-ƽ�� 3-����

disp('��λ����')
% tic
% parfor i_row = 1:rowNum
for i_row = 1:rowNum

    for i_col = 1:colNum
        slantRange = slantRangeMultiple(:, i_row, i_col); %б��
        %slantRange = slantRange+100;

        result = SolTrack(slantRange', ...
            [transponders(1, :), transponders(2, :), transponders(3, :)], ...
            z, 1.0, 4);

        initPosition_Rec1(i_row, i_col, :) = result(1:2);
    end

end
% toc

%ƫ����
errLoc = ones(rowNum, colNum) * nan;
for i_row = 1:rowNum
    for i_col = 1:colNum
        errLoc(i_row, i_col) = sqrt((initPosition_Rec1(i_row, i_col, 1) - xGrid(i_row, i_col))^2+(initPosition_Rec1(i_row, i_col, 2) - yGrid(i_row, i_col))^2);
    end
end

errLoc(abs(errLoc) < 1e0) = 1;
errLoc(errLoc ~= 1) = 2;

% load matlab.mat

%% ��ͼ
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
% title(['��ͬ����λ˫�����÷�Χ']);
xlabel('X/m');
ylabel('Y/m');
set(gca, 'FontSize', 15)
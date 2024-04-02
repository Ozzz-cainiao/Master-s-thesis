%% 对比算法场景3--虚警多目标实验--作废
% 虚警服从均值泊松分布,虚警特征实际角度根据叠加产生，效果可能不好
% Author LI Shuo
% 2023年5月16日 15点48分
clc
clear
close all
warning('off')
tic;                                % tic;与toc;配合使用能够返回程序运行时间
% bar     = waitbar(0,'读取数据中...');    % waitbar显示进度条
%% ==========================参数===============================
times   = 5;
var2d   = 1 ;
var2    = var2d*(pi/180)^2;   	% 角度测量方差单位rad，角度测量误差服从零均值高斯分布
Fai     = pi/2;
% DCGT参数 M、Q、I
M       = 5;                   % 选取最少线点数
I       = 10;                   % 并行次优条数
% MDADA参数 area
area    = -3:0.5:3;
% MDADA参数 area
detla   = 0.3;                            % 成功关联的门限

%% ==========================布放===============================
% load('F-Measure1228.mat')            	% 导入关联特征
% load('randFA1228_3.mat')
node    = [-2,-2;2,-2;2,2;-2,2];                        % 节点位置
source  = [-1.5,0.05;-0.3,-0.05;0.05,-1.5;1.5,-0.05;0.3,0.05;-0.05,1.5]; 
S       = size(node,1);                                 % 节点数
choNum  = [1,3,4];
sourceCho   = source(choNum,:);
N           = size(sourceCho,1);
angR        = atan2d((sourceCho(:,2)-node(:,2).'),(sourceCho(:,1)-node(:,1).'));
fs      = 50e3;
Q       = [0,10,15,15,20,20];
Q       = 20;
PD      = 1;


%% ==========================生成测量===============================
zk          = angR + sqrt(var2d)*randn(N,S);                            % 方位测量生成 
Fk          = mat2cell(num2cell(zk),N,ones(1,S));                       % 分割方位测量矩阵
ZZ          = cellfun(@(x,y) x,Fk,'un',0);                    	% 正确关联测量集
[~,posZ]    = arrayfun(@(x) sort(cell2mat(x{1}(:,1))),ZZ,'un',0);
[~,posR]    = cellfun(@(x) sort(x),posZ,'un',0);                        % 正确关联顺序
posR        = cat(2,posR{:});
ns          = cellfun(@(x) size(x,1),ZZ);                               % 每个平台测量数目
Z           = cellfun(@(u,v) u(v,:),ZZ,posZ,'un',0);                    % 所有测量
outMDADA    = mdada1(Z,node,area,[var2,PD-0.01,Fai]);







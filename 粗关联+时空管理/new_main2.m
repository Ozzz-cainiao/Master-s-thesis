%% 利用分治贪心算法实现709项目关联
% Author LI Shuo
% 2023�?9�?19�? 10�?20�?
clc
clear
close all

%% ==========================布放参数===============================
rand('seed',1)
c       = 1500;
var2d   = 1.5^2 ;
% var2d   = 0.5^2 ;
var2    = var2d*(pi/180)^2; 
% v0     	= [10];
% range 	= [ 6000 ];         % 目标产生位置的距�?
% bear    = [ 60 ];         	% 目标产生位置的角�?
% course  = [ 90];           	% 运动方向
v0     	= [10,10];
range 	= [ 6000 8e3 ];         % 目标产生位置的距�?
bear    = [ 60 30];         	% 目标产生位置的角�?
course  = [ 90 50];           	% 运动方向
num     = length(v0);
S     	= 3; % 平台数目
node   	= round( 5e3*sqrt(2)*[cosd(-135:360/S:224)',sind(-135:360/S:224)']+[5e3,5e3] ); % 节点位置
Dth     = 500; 
dPonit  = 10;
PD      = 0.99;%0.9        	% �?测概�?2~3个目标时�?0.99�?4~5个目标时�?0.9
Fai     = 2*pi;
M       = 3;                % 选取�?少线点数
Q       = 10;                % 列表�?大长�?
I       = 3;                % 并行次优条数
%%
x0      = range.*cosd(bear);
y0      = range.*sind(bear);
vy      = v0.*cosd(course);
vx      = v0.*sind(course);
X0      = [x0;y0;vx;vy];
T       = 1e-3;                 % 观测周期
T_all   = 20;                	% 目标持续时间
T_num   = T_all/T;              % 观测次数
dt      = T;
%�?速运动矩�?
F1 = [1,0,T,0;
    0,1,0,T;
    0,0,1,0;
    0,0,0,1];
%加�?�度矩阵
F2 = [0.5*T^2,0;
    0,0.5*T^2;
    T,0;
    0,T];
x       = zeros(num,T_num); % 目标x坐标
y       = zeros(num,T_num); % 目标y坐标
x_r     = zeros(num,T_num); % 目标相对观测者x坐标
y_r     = zeros(num,T_num); % 目标相对观测者y坐标
angle_r = zeros(num,T_num); % 目标相对观测者方�?
a_max   = 0/1e3;         	% 目标�?大加速度
X       = arrayfun(@(j) zeros(4,T_num),1:num,'un',0);
%% ==========================目标真实状�??===============================
for t = 1:T_num
    for j = 1:length(x0)
        ax = a_max*(2*rand-1);
        ay = a_max*(2*rand-1);
        a  = [ax ay]';
        if t==1
            X{j}(:,t) = X0(:,j);
        else
            X{j}(:,t) = F1*X{j}(:,t-1)+F2*a;
        end
    end
end
for i = 1:num
    x(i,:) = X{i}(1,:);
    y(i,:) = X{i}(2,:);
end
figure
hold on
for i = 1:num
    plot(x(i,:),y(i,:),'.')
end
scatter(node(:,1),node(:,2),'b^','filled','LineWidth',0.5,'SizeData',100);
% for kk = 1:1
%% ==========================观测===============================
dT              = 5;
t_obs           = T+dT*2:T:dT*3;
X               = repmat( node,1,1,T_num);
[x_obs,y_obs]   = deal(zeros(size(node,1),T_num));
for ii = 1:S
    x_obs(ii,:) = X(ii,1,:);
    y_obs(ii,:) = X(ii,2,:);
    angR{ii}   	= nan(num,length(t_obs)*10);
    for i=1: num
        x_r(i,:)    = x(i,:)-x_obs(ii,1:T_num);
        y_r(i,:)    = y(i,:)-y_obs(ii,1:T_num);
        r_r         = sqrt(x_r(i,:).^2+y_r(i,:).^2);
        angle_r(i,:)= atan2d(y_r(i,:), x_r(i,:));
        angle_r(angle_r<0) = angle_r(angle_r<0)+360;
        t_r         = r_r/c;
        t_delay{i,ii} 	= round(t_r, 1);
        for iii = 1:T_num
            tNum = round(t_delay{i,ii}(iii)/T) + iii;
            angR{ii}(i,tNum) = angle_r(i,iii) +  sqrt(var2d)* randn;
        end  
    end 
end
angM = cell(length(t_obs),S);
for iii = 1:length(t_obs)
    angM(iii,:) = arrayfun(@(s) angR{s}( ~isnan( sort( angR{s}(:,t_obs(1)/T+iii-1) ) ) ,t_obs(1)/T+iii-1) ,1:S,'un',0);
end

%% 
outLoctionCAX = zeros(num, length(t_obs));
outLoctionCAY = zeros(num, length(t_obs));
for iii = 1 : length(t_obs)
    disp(['now', num2str(iii)])
    angM1 = angM(iii,:);
[outLoctionCAX(:, iii), outLoctionCAY(:, iii)] = calcOne_new(angM1, S, node, num, iii);
end
%% ==========================绘图�?===============================
figure
hold on
for ii = 1:num
plot(outLoctionCAX(ii,:),outLoctionCAY(ii,:),'.')
axis([0,10e3,0,10e3])
end




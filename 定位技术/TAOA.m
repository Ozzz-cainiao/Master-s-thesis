% 时延差/方位联合解算

%% 双平台

clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c theta1 theta2
f1 = (x1-xs)^2 + (y1-ys)^2 - c^2*(t1- ts)^2;
f2 = (x2-xs)^2 + (y2-ys)^2 - c^2*(t2- ts)^2;
f3 = (x1-xs)^2 + (y1-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x1-xs)^2 + (y1-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta1) - (x2-xs)^2 - (y2-ys)^2;
f4 = (x2-xs)^2 + (y2-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x2-xs)^2 + (y2-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta2) - (x1-xs)^2 - (y1-ys)^2;


A_M = [diff(f1,xs),diff(f1,ys);
    diff(f2,xs),diff(f2,ys);
    diff(f3,xs),diff(f3,ys);
    diff(f4,xs),diff(f4,ys)];

A_Mt = [diff(f1,t1),diff(f1,t2);...
    diff(f2,t1),diff(f2,t2);...
    diff(f3,t1),diff(f3,t2);
    diff(f4,t1),diff(f4,t2)];

A_Mtheta =  [diff(f1,theta1),diff(f1,theta2);
    diff(f2,theta1),diff(f2,theta2);
    diff(f3,theta1),diff(f3,theta2);
    diff(f4,theta1),diff(f4,theta2)];

M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);

Mtheta = matlabFunction(A_Mtheta);
clearvars -except M Mt  Mtheta

x = 0:10:2000;
y = (0:10:2000)';
[x1,y1,x2,y2] = deal(300,300,1700,300);%% 平台位置
% [x1,y1,x2,y2,x3,y3] = deal(300,300,1700,300,1000,1700);%% 平台位置

[lenx,leny] = deal(length(x),length(y));

theta1 = atan((x1-x)./(y1-y));
theta2 = atan((x2-x)./(y2-y));
errornor = [0.0001,0];      % 测时误差，测向误差
% errornor = [0,(0.5/180*pi)^2];      % 测时误差，测向误差
errorsum = zeros(lenx,leny);
c=1500;
ts = 0;
% t1 = sqrt((x1-x)^2 + (y1-y)^2) / c; 
% t2 = sqrt((x2-x)^2 + (y2-y)^2) / c; 
for ii = 1:lenx
    for jj = 1:leny
        t1 = sqrt((x1-x(ii))^2 + (y1-y(jj))^2) / c; 
        t2 = sqrt((x2-x(ii))^2 + (y2-y(jj))^2) / c; 
        m  = M(theta1(jj,ii),theta2(jj,ii),x1,x2,x(ii),y1,y2,y(jj));
        mtheta = Mtheta(theta1(jj,ii),theta2(jj,ii),x1,x2,x(ii),y1,y2,y(jj));
        mt = Mt(c, t1, t2, ts);
        error = (m'*m)\m'*(mt*diag([errornor(1),errornor(1)])*mt' ...
                   +mtheta*diag([errornor(2),errornor(2)])*mtheta')*m/(m'*m);
        errorsum(jj,ii)  = sqrt(trace(error));
    end
end

figure
surf(x,y,abs(errorsum))
set(gca, 'YDir', 'normal');colorbar;
xlabel('x/m');ylabel('y/m') ;title('平面角解算误差');
clim([0 100]);colormap jet
shading interp;view(0,90) 

mean2(abs(errorsum(1:100,:)))


%% 三平台

clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 xs ys n1 n2 n3 t1 t2 t3 ts c theta1 theta2 theta3
f1 = (x1-xs)^2 + (y1-ys)^2 - c^2*(t1- ts)^2;
f2 = (x2-xs)^2 + (y2-ys)^2 - c^2*(t2- ts)^2;
f3 = (x3-xs)^2 + (y3-ys)^2 - c^2*(t3- ts)^2;

f4 = (x1-xs)^2 + (y1-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x1-xs)^2 + (y1-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta1);
f5 = (x2-xs)^2 + (y2-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x2-xs)^2 + (y2-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta2);
f6 = (x3-xs)^2 + (y3-ys)^2 + (x3-x1)^2 + (y3-y1)^2 - 2*sqrt((x3-xs)^2 + (y3-ys)^2)*sqrt((x3-x1)^2 + (y3-y1)^2)*cos(theta3);

A_M =  [diff(f1,xs),diff(f1,ys);...
    diff(f2,xs),diff(f2,ys);...
    diff(f3,xs),diff(f3,ys);...
    diff(f4,xs),diff(f4,ys);...
    diff(f5,xs),diff(f5,ys);...
    diff(f6,xs),diff(f6,ys)];

% A_Mt = [diff(f1,t1),diff(f1,t2),diff(f1,t3);...
%     diff(f2,t1),diff(f2,t2),diff(f2,t3);...
%     diff(f3,t1),diff(f3,t2),diff(f3,t3)...
%     diff(f4,t1),diff(f4,t2),diff(f4,t3);...
%     diff(f5,t1),diff(f5,t2),diff(f5,t3);...
%     diff(f6,t1),diff(f6,t2),diff(f6,t3)];
A_Mt = [diff(f1,t1),diff(f1,t2),diff(f1,t3);...
    diff(f2,t1),diff(f2,t2),diff(f2,t3);...
    diff(f3,t1),diff(f3,t2),diff(f3,t3);...
    diff(f4,t1),diff(f4,t2),diff(f4,t3);...
    diff(f5,t1),diff(f5,t2),diff(f5,t3);...
    diff(f6,t1),diff(f6,t2),diff(f6,t3)];

A_Mtheta =  [diff(f1,theta1),diff(f2,theta2),diff(f1,theta3);...
    diff(f2,theta1),diff(f2,theta2),diff(f2,theta3);...
    diff(f3,theta1),diff(f3,theta2),diff(f3,theta3);...
    diff(f4,theta1),diff(f4,theta2),diff(f4,theta3);...
    diff(f5,theta1),diff(f5,theta2),diff(f5,theta3);...
    diff(f6,theta1),diff(f6,theta2),diff(f6,theta3)];

M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);

Mtheta = matlabFunction(A_Mtheta);
clearvars -except M Mt  Mtheta

x = 0:10:2000;
y = (0:10:2000)';
% [x1,y1,x2,y2] = deal(300,1000,1700,1000);%% 平台位置
[x1,y1,x2,y2,x3,y3] = deal(300,300,1700,300,1000,1700);%% 平台位置

[lenx,leny] = deal(length(x),length(y));

theta1 = atan((x1-x)./(y1-y));
theta2 = atan((x2-x)./(y2-y));
theta3 = atan((x3-x)./(y3-y));
errornor = [0.0001,0];      % 测时误差，测向误差
% errornor = [0,(0.5/180*pi)^2];      % 测时误差，测向误差
% errornor = [0.001^2,0];      % 测时误差，测向误差
errorsum = zeros(lenx,leny);
c=1500;
ts = 0;
% t1 = sqrt((x1-x)^2 + (y1-y)^2) / c; 
% t2 = sqrt((x2-x)^2 + (y2-y)^2) / c; 
for ii = 1:lenx
    for jj = 1:leny

        t1 = sqrt((x1-x(ii))^2 + (y1-y(jj))^2) / c; 
        t2 = sqrt((x2-x(ii))^2 + (y2-y(jj))^2) / c; 
        t3 = sqrt((x3-x(ii))^2 + (y3-y(jj))^2) / c; 

        m  = M(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii),x1,x2,x3,x(ii),y1,y2,y3,y(jj));
        mtheta = Mtheta(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii),x1,x2,x3,x(ii),y1,y2,y3,y(jj));
    
        mt = Mt(c, t1, t2, t3, ts);
        error = (m'*m)\m'*(mt*diag([errornor(1),errornor(1),errornor(1)])*mt' ...
                   +mtheta*diag([errornor(2),errornor(2),errornor(2)])*mtheta')*m/(m'*m);
        errorsum(jj,ii)  = sqrt(trace(error));
    end
end

figure
surf(x,y,abs(errorsum))
set(gca, 'YDir', 'normal');colorbar;
xlabel('x/m');ylabel('y/m') ;title('平面角解算误差');
clim([0 30]);colormap jet
shading interp;view(0,90) 

mean2(abs(errorsum(1:100,:)))


%% （四）平台


clc
clear
close all
% 求出解析解
syms x1 y1 x2 y2 x3 y3 x4 y4 xs ys t1 t2 t3 t4 ts c theta1 theta2 theta3 theta4
f1 = (x1-xs)^2 + (y1-ys)^2 - c^2*(t1- ts)^2;
f2 = (x2-xs)^2 + (y2-ys)^2 - c^2*(t2- ts)^2;
f3 = (x3-xs)^2 + (y3-ys)^2 - c^2*(t3- ts)^2;
f4 = (x4-xs)^2 + (y4-ys)^2 - c^2*(t4- ts)^2;

f5 = (x1-xs)^2 + (y1-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x1-xs)^2 + (y1-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta1);
f9 = (x1-xs)^2 + (y1-ys)^2 + (x3-x1)^2 + (y3-y1)^2 - 2*sqrt((x1-xs)^2 + (y1-ys)^2)*sqrt((x3-x1)^2 + (y3-y1)^2)*cos(theta1);

f6 = (x2-xs)^2 + (y2-ys)^2 + (x2-x1)^2 + (y2-y1)^2 - 2*sqrt((x2-xs)^2 + (y2-ys)^2)*sqrt((x2-x1)^2 + (y2-y1)^2)*cos(theta2);
f10 = (x2-xs)^2 + (y2-ys)^2 + (x2-x4)^2 + (y2-y4)^2 - 2*sqrt((x2-xs)^2 + (y2-ys)^2)*sqrt((x2-x4)^2 + (y2-y4)^2)*cos(theta2);

f7 = (x3-xs)^2 + (y3-ys)^2 + (x3-x4)^2 + (y3-y4)^2 - 2*sqrt((x3-xs)^2 + (y3-ys)^2)*sqrt((x3-x4)^2 + (y3-y4)^2)*cos(theta3);
f11 = (x3-xs)^2 + (y3-ys)^2 + (x3-x1)^2 + (y3-y1)^2 - 2*sqrt((x3-xs)^2 + (y3-ys)^2)*sqrt((x3-x1)^2 + (y3-y1)^2)*cos(theta3);

f8 = (x4-xs)^2 + (y4-ys)^2 + (x4-x3)^2 + (y4-y3)^2 - 2*sqrt((x4-xs)^2 + (y4-ys)^2)*sqrt((x4-x3)^2 + (y4-y3)^2)*cos(theta4);
f12 = (x4-xs)^2 + (y4-ys)^2 + (x4-x2)^2 + (y4-y2)^2 - 2*sqrt((x4-xs)^2 + (y4-ys)^2)*sqrt((x4-x2)^2 + (y4-y2)^2)*cos(theta4);

A_M =  [diff(f1,xs),diff(f1,ys);diff(f2,xs),diff(f2,ys);...
    diff(f3,xs),diff(f3,ys);diff(f4,xs),diff(f4,ys);...
    diff(f5,xs),diff(f5,ys);diff(f6,xs),diff(f6,ys);...
    diff(f7,xs),diff(f7,ys);diff(f8,xs),diff(f8,ys)];
% A_M =  [diff(f1,xs),diff(f1,ys);diff(f2,xs),diff(f2,ys);...
%     diff(f3,xs),diff(f3,ys);diff(f4,xs),diff(f4,ys);...
%     diff(f5,xs),diff(f5,ys);diff(f6,xs),diff(f6,ys);...
%     diff(f7,xs),diff(f7,ys);diff(f8,xs),diff(f8,ys);...
%     diff(f9,xs),diff(f9,ys);diff(f10,xs),diff(f10,ys);...
%     diff(f11,xs),diff(f11,ys);diff(f12,xs),diff(f12,ys)];

% A_Mt = [diff(f1,t1),diff(f1,t2),diff(f1,t3);...
%     diff(f2,t1),diff(f2,t2),diff(f2,t3);...
%     diff(f3,t1),diff(f3,t2),diff(f3,t3)...
%     diff(f4,t1),diff(f4,t2),diff(f4,t3);...
%     diff(f5,t1),diff(f5,t2),diff(f5,t3);...
%     diff(f6,t1),diff(f6,t2),diff(f6,t3)];
A_Mt = [diff(f1,t1),diff(f1,t2),diff(f1,t3),diff(f1,t4);...
    diff(f2,t1),diff(f2,t2),diff(f2,t3),diff(f2,t4);...
    diff(f3,t1),diff(f3,t2),diff(f3,t3),diff(f3,t4);...
    diff(f4,t1),diff(f4,t2),diff(f4,t3),diff(f4,t4);...
    diff(f5,t1),diff(f5,t2),diff(f5,t3),diff(f5,t4);...
    diff(f6,t1),diff(f6,t2),diff(f6,t3),diff(f6,t4);...
    diff(f7,t1),diff(f7,t2),diff(f7,t3),diff(f7,t4);...
    diff(f8,t1),diff(f8,t2),diff(f8,t3),diff(f8,t4)];
% A_Mt = [diff(f1,t1),diff(f1,t2),diff(f1,t3),diff(f1,t4);...
%     diff(f2,t1),diff(f2,t2),diff(f2,t3),diff(f2,t4);...
%     diff(f3,t1),diff(f3,t2),diff(f3,t3),diff(f3,t4);...
%     diff(f4,t1),diff(f4,t2),diff(f4,t3),diff(f4,t4);...
%     diff(f5,t1),diff(f5,t2),diff(f5,t3),diff(f5,t4);...
%     diff(f6,t1),diff(f6,t2),diff(f6,t3),diff(f6,t4);...
%     diff(f7,t1),diff(f7,t2),diff(f7,t3),diff(f7,t4);...
%     diff(f8,t1),diff(f8,t2),diff(f8,t3),diff(f8,t4);...
%     diff(f9,t1),diff(f9,t2),diff(f9,t3),diff(f9,t4);...
%     diff(f10,t1),diff(f10,t2),diff(f10,t3),diff(f10,t4);...
%     diff(f11,t1),diff(f11,t2),diff(f11,t3),diff(f11,t4);...
%     diff(f12,t1),diff(f12,t2),diff(f12,t3),diff(f12,t4)];
A_Mtheta =  [diff(f1,theta1),diff(f2,theta2),diff(f1,theta3),diff(f1,theta4);...
    diff(f2,theta1),diff(f2,theta2),diff(f2,theta3),diff(f2,theta4);...
    diff(f3,theta1),diff(f3,theta2),diff(f3,theta3),diff(f3,theta4);...
    diff(f4,theta1),diff(f4,theta2),diff(f4,theta3),diff(f4,theta4);...
    diff(f5,theta1),diff(f5,theta2),diff(f5,theta3),diff(f5,theta4);...
    diff(f6,theta1),diff(f6,theta2),diff(f6,theta3),diff(f6,theta4);...
    diff(f7,theta1),diff(f7,theta2),diff(f7,theta3),diff(f7,theta4);...
    diff(f8,theta1),diff(f8,theta2),diff(f8,theta3),diff(f8,theta4)];
% A_Mtheta =  [diff(f1,theta1),diff(f2,theta2),diff(f1,theta3),diff(f1,theta4);...
%     diff(f2,theta1),diff(f2,theta2),diff(f2,theta3),diff(f2,theta4);...
%     diff(f3,theta1),diff(f3,theta2),diff(f3,theta3),diff(f3,theta4);...
%     diff(f4,theta1),diff(f4,theta2),diff(f4,theta3),diff(f4,theta4);...
%     diff(f5,theta1),diff(f5,theta2),diff(f5,theta3),diff(f5,theta4);...
%     diff(f6,theta1),diff(f6,theta2),diff(f6,theta3),diff(f6,theta4);...
%     diff(f7,theta1),diff(f7,theta2),diff(f7,theta3),diff(f7,theta4);...
%     diff(f8,theta1),diff(f8,theta2),diff(f8,theta3),diff(f8,theta4);...
%     diff(f9,theta1),diff(f9,theta2),diff(f9,theta3),diff(f9,theta4);...
%     diff(f10,theta1),diff(f10,theta2),diff(f10,theta3),diff(f10,theta4);...
%     diff(f11,theta1),diff(f11,theta2),diff(f11,theta3),diff(f11,theta4);...
%     diff(f12,theta1),diff(f12,theta2),diff(f12,theta3),diff(f12,theta4)];

M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);

Mtheta = matlabFunction(A_Mtheta);
clearvars -except M Mt  Mtheta

x = 0:10:2000;
y = (0:10:2000)';
% [x1,y1,x2,y2] = deal(300,1000,1700,1000);%% 平台位置
[x1,y1,x2,y2,x3,y3, x4, y4] = deal(300, 300, 1700, 300, 300, 1700, 1700, 1700);%% 平台位置

[lenx,leny] = deal(length(x),length(y));

theta1 = atan((x1-x)./(y1-y));
theta2 = atan((x2-x)./(y2-y));
theta3 = atan((x3-x)./(y3-y));
theta4 = atan((x4-x)./(y4-y));

% errornor = [0,(0.5/180*pi)^2];      % 测时误差，测向误差
errornor = [0.0001,0];      % 测时误差，测向误差
% errornor = [0.001^2,0];      % 测时误差，测向误差
errorsum = zeros(lenx,leny);
c=1500;
ts = 0;
% t1 = sqrt((x1-x)^2 + (y1-y)^2) / c; 
% t2 = sqrt((x2-x)^2 + (y2-y)^2) / c; 
for ii = 1:lenx
    for jj = 1:leny

        t1 = sqrt((x1-x(ii))^2 + (y1-y(jj))^2) / c; 
        t2 = sqrt((x2-x(ii))^2 + (y2-y(jj))^2) / c; 
        t3 = sqrt((x3-x(ii))^2 + (y3-y(jj))^2) / c; 
        t4 = sqrt((x4-x(ii))^2 + (y4-y(jj))^2) / c;
        m  = M(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii),theta4(jj,ii),x1,x2,x3,x4,x(ii),y1,y2,y3,y4,y(jj));
        mtheta = Mtheta(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii),theta4(jj,ii),x1,x2,x3,x4,x(ii),y1,y2,y3,y4,y(jj));
    
        mt = Mt(c, t1, t2, t3, t4, ts);
        error = (m'*m)\m'*(mt*diag([errornor(1),errornor(1),errornor(1),errornor(1)])*mt' ...
                   +mtheta*diag([errornor(2),errornor(2),errornor(2),errornor(2)])*mtheta')*m/(m'*m);
        errorsum(jj,ii)  = sqrt(trace(error));
    end
end

figure
surf(x,y,abs(errorsum))
set(gca, 'YDir', 'normal');colorbar;
xlabel('x/m');ylabel('y/m') ;title('平面角解算误差');
clim([0 30]);colormap jet
shading interp;view(0,90) 

mean2(abs(errorsum(1:100,:)))

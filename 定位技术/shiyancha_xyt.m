%% 时延差异步

%
clc
clear
% close all
% 求出解析解
syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 xs ys zs t1 t2 t3 t4 ts 
c = 1500;
f1 = (x1-xs)^2+(y1-ys)^2-(c*t1-c*ts)^2;
f2 = (x2-xs)^2+(y2-ys)^2-(c*t2-c*ts)^2;
f3 = (x3-xs)^2+(y3-ys)^2-(c*t3-c*ts)^2;

A_M1= [diff(f1,x1),diff(f1,y1);diff(f2,x1),diff(f2,y1);diff(f3,x1),diff(f3,y1)];
A_M2= [diff(f1,x2),diff(f1,y2);diff(f2,x2),diff(f2,y2);diff(f3,x2),diff(f3,y2)];
A_M3= [diff(f1,x3),diff(f1,y3);diff(f2,x3),diff(f2,y3);diff(f3,x3),diff(f3,y3)];

A_M = [diff(f1,xs),diff(f1,ys);diff(f2,xs),diff(f2,ys);diff(f3,xs),diff(f3,ys)];

A_Mt =  [diff(f1,t1),diff(f1,t2),diff(f1,t3);...
    diff(f2,t1),diff(f2,t2),diff(f2,t3);...
    diff(f3,t1),diff(f3,t2),diff(f3,t3)];

A_Mts =  [diff(f1,t1),diff(f1,t2),diff(f1,t3);...
    diff(f2,t1),diff(f2,t2),diff(f2,t3);...
    diff(f3,t1),diff(f3,t2),diff(f3,t3)];

%A_M = [diff(f1,x1),diff(f1,y1),diff(f1,t1),diff(f1,x2),diff(f1,y2),diff(f1,t2),diff(f1,x3),diff(f1,y3),diff(f1,t3),diff(f1,xs),diff(f1,ys),diff(f1,ts);
%    diff(f2,x1),diff(f2,y1),diff(f2,t1),diff(f2,x2),diff(f2,y2),diff(f2,t2),diff(f2,x3),diff(f2,y3),diff(f2,t3),diff(f2,xs),diff(f2,ys),diff(f2,ts);
%     diff(f3,x1),diff(f3,y1),diff(f3,t1),diff(f3,x2),diff(f3,y2),diff(f3,t2),diff(f3,x3),diff(f3,y3),diff(f3,t3),diff(f3,xs),diff(f3,ys),diff(f3,ts);];

% 

M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
M3 = matlabFunction(A_M3);
M = matlabFunction(A_M);
Mt = matlabFunction(A_Mt);
Mts = matlabFunction(A_Mts);

clearvars -except M1 M2 M3 Mts M Mt


x = 0:10:2000;
y = (0:10:2000)';

[x1,y1,x2,y2,x3,y3] = deal(300,300,1700,300,1000,1700);%% 平台位置

[lenx,leny] = deal(length(x),length(y));

errornor = [0,0,0.01];      % x y ts误差  假设只有测时误差
errorsum = zeros(lenx,leny);



t1 = sqrt((x1 - x).^2 + (y1 - y).^2)/1500;
t2 = sqrt((x2 - x).^2 + (y2 - y).^2)/1500;
t3 = sqrt((x3 - x).^2 + (y3 - y).^2)/1500;

for ii = 1:lenx
    for jj = 1:leny
        m  = M(t1(jj,ii,1),t2(jj,ii,1),t3(jj,ii,1));
        mts = Mts(t1(jj,ii),t2(jj,ii),t3(jj,ii),x1,x2,x3,x(ii),y1,y2,y3,y(jj));
        error = (m'*m)\m'*((m1*diag([errornor(1),errornor(2)])*m1'+m2*diag([errornor(1),errornor(2)])*m2'...
            +m3*diag([errornor(1),errornor(2)])*m3'+mtheta*diag([errornor(3),errornor(3),errornor(3)])*mtheta')*m/(m'*m));
        errorsum(jj,ii)  = sqrt(trace(error));
    end
end
figure
surf(x,y,abs(errorsum))
set(gca, 'YDir', 'normal');colorbar;
xlabel('x/m');ylabel('y/m') ;title('平面角解算误差');
clim([0 15]);colormap jet
shading interp;view(0,90) 

mean2(abs(errorsum(1:100,:)))
%% 时延差异步

%
clc
clear
% close all
% 求出解析解
syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 xs ys zs t1 t2 t3 t4 ts 

f1 = (x1-xs)^2+(y1-ys)^2+(z1-zs)^2-(c*t1-c*ts)^2;
f2 = (x2-xs)^2+(y2-ys)^2+(z2-zs)^2-(c*t2-c*ts)^2;
f3 = (x3-xs)^2+(y3-ys)^2+(z3-zs)^2-(c*t3-c*ts)^2;
f4 = (x4-xs)^2+(y4-ys)^2+(z4-zs)^2-(c*t4-c*ts)^2;

A_1= [diff(f1,x1),diff(f1,y1),diff(f1,z1);diff(f2,x1),diff(f2,y1),diff(f2,z1);diff(f3,x1),diff(f3,y1),diff(f3,z1);diff(f4,x1),diff(f4,y1),diff(f4,z1)];
A_2= [diff(f1,x2),diff(f1,y2),diff(f1,z2);diff(f2,x2),diff(f2,y2),diff(f2,z2);diff(f3,x2),diff(f3,y2),diff(f3,z2);diff(f3,x2),diff(f3,y2),diff(f4,z2)];
A_3= [diff(f1,x3),diff(f1,y3),diff(f1,z3);diff(f2,x3),diff(f2,y3),diff(f2,z3);diff(f3,x3),diff(f3,y3),diff(f3,z3);diff(f3,x4),diff(f3,y3),diff(f4,z3)];
A_4= [diff(f1,x4),diff(f1,y4),diff(f1,z4);diff(f2,x4),diff(f2,y4),diff(f2,z4);diff(f3,x4),diff(f3,y4),diff(f3,z4);diff(f3,x4),diff(f3,y4),diff(f4,z4)];
A_M = [diff(f1,xs),diff(f1,ys),diff(f1,zs);diff(f2,xs),diff(f2,ys),diff(f2,zs);diff(f3,xs),diff(f3,ys),diff(f3,zs);diff(f4,xs),diff(f4,ys),diff(f4,zs)];
A_Mtime = [diff(f1,t1),diff(f1,t2),diff(f1,t3),diff(f1,t4),diff(f1,t5);
    diff(f2,t1),diff(f2,t2),diff(f2,t3),diff(f2,t4),diff(f2,t5);...
    diff(f3,t1),diff(f3,t2),diff(f3,t3),diff(f3,t4),diff(f3,t5);...
    diff(f4,t1),diff(f4,t2),diff(f4,t3),diff(f4,t4),diff(f4,t5);...
    diff(f5,t1),diff(f5,t2),diff(f5,t3),diff(f5,t4),diff(f5,t5)];

M1 = matlabFunction(A_M1);
M2 = matlabFunction(A_M2);
M3 = matlabFunction(A_M3);
M4 = matlabFunction(A_4);
M = matlabFunction(A_M);
Mtime = matlabFunction(A_Mtime);
clearvars -except M1 M2 M3 M4 M Mtime


x = 0:10:2000;
y = (0:10:2000)';

[x1,y1,x2,y2,x3,y3,x4,y4] = deal(300,300,1700,300,300,1700,1700,1700);

[lenx,leny] = deal(length(x),length(y));


errornor = [0,0,0,0.01];      % x y z t误差
t1 = 
errorsum = zeros(lenx,leny);
for ii = 1:lenx
    for jj = 1:leny
        m1 = M1(ii,jj,0);
        m2 = M2(ii,jj,0);
        m3 = M3(ii,jj,0);
        m4 = M4(ii,jj,0);
        m  = M(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii));
        mtheta = Mtime(theta1(jj,ii),theta2(jj,ii),theta3(jj,ii),x1,x2,x3,x(ii),y1,y2,y3,y(jj));
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
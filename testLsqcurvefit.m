

%Optimset函数：‘Gradobj’指用户自定义的目标函数梯度；‘MaxITer’指最大迭代次数，‘100’也就是最大迭代次数，这一项只能为整数。
options = optimset('MaxFunEvals',800,'MaxIter',500);
lb = [];
ub = [];
%test
xdata = [0.9 1.5 13.8 19.8 24.1 28.2 35.2 60.3 74.6 81.3];
ydata = [455.2 428.6 124.1 67.3 43.2 28.1 13.1 -0.4 -1.3 -1.5];

fun = @(x,xdata)x(1)*exp(x(2)*xdata);
x0 = [100,-1];
x = lsqcurvefit(fun,x0,xdata,ydata);

times = linspace(xdata(1),xdata(end));
plot(xdata,ydata,'ko',times,fun(x,times),'b-')
legend('Data','Fitted exponential')
title('Data and Fitted Curve')

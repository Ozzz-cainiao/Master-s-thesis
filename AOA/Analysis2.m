%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\AOA\Analysis2.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2022-09-01
% 描述: 精度分析主程序，输入观测平台的位置，设置的观测范围 步长 测量误差  参考论文：臧传斌
% 
% 输入:  arrx, arry, Xmin, Xmax, Ymin, Ymax, Step, errornor
% 输出:  无，直接画图
%**************************************************************************



% 现在存在的疑惑，多平台的误差分析不该是这个解算方法吧，没有用到不同角的统计
%
function [] = Analysis2(arrx, arry, Xmin, Xmax, Ymin, Ymax, Step, errornor)

x = Xmin:Step:Xmax;
y = (Ymin:Step:Ymax)';

[lenx, leny] = deal(length(x), length(y));
errorsum = zeros(lenx, leny);

% theta = zeros(5, lenx, leny);
theta = zeros(length(arrx), lenx, leny);
% [theta1, theta2, theta3, theta4, theta5] = deal(zeros(, leny));
for kk = 1:length(arrx)
    for ii = 1:lenx
        for jj = 1:leny
            theta(kk, ii, jj) = atan((arrx(kk) - x(ii))./(arry(kk) - y(jj))); %弧度单位
        end
    end
end
m = zeros(length(arrx), 2, lenx, leny);
mtheta = zeros(length(arrx), length(arrx));
diagerror = errornor(3) * eye(length(arrx));
diagerrorxy =  diag([errornor(1), errornor(2)]);

for ii = 1:lenx
    for jj = 1:leny
        mx = zeros(length(arrx), 2);
        for kk = 1:length(arrx)
            m(kk, 1, ii, jj) = cos(theta(kk, ii, jj));
            m(kk, 2, ii, jj) = -sin(theta(kk, ii, jj));
            mtheta(kk, kk) = arry(kk) .* cos(theta(kk, ii, jj)) - y(jj) .* cos(theta(kk, ii, jj)) + arrx(kk) .* sin(theta(kk, ii, jj)) - x(ii) .* sin(theta(kk, ii, jj));
            mx(kk,:) = [m(kk, 1, ii, jj), m(kk, 2, ii, jj)];
            
        end
        %         xxx=mtheta * diagerror * mtheta';
        error1 = (m(:, :, ii, jj)' * m(:, :, ii, jj)) \ m(:, :, ii, jj)';
%         error2 = mtheta * diagerror * mtheta';
        error2 = mtheta * diagerror * mtheta';
        for kk = 1 : length(arrx)
            error2 = error2 +  mx(kk) * diagerrorxy * mx(kk)';
        end
 
        error2_1 = (error2 * m(:, :, ii, jj) / ...
            (m(:, :, ii, jj)' * m(:, :, ii, jj)));
        error = error1 * error2_1;
        %         error = (m(:,:,ii,jj)' * m(:,:,ii,jj)) \ m(:,:,ii,jj)'...
        %             * (xxx * m(:,:,ii,jj) / ...
        %         (m(:,:,ii,jj)' * m(:,:,ii,jj)));
        errorsum(jj, ii) = sqrt(trace(error));
    end
end

figure
surf(x, y, abs(errorsum));
view(0,90);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x/m');
ylabel('y/m');
title('平面角解算误差');
clim([0, 10]);
colormap jet;
shading interp;
hold on;
plot(arrx, arry, 'r*');
if (max(abs(errorsum)) > 100)
    clim([0, 100]);
    mean2(abs(errorsum(1:100, :)))
end

function zs=Chuizhi_ruan(posi,posj,poss,c,tao)
xi=posi(1);
yi=posi(2);
zi=posi(3);
xj=posj(1);
yj=posj(2);
zj=posj(3);
xs=poss(1);
ys=poss(2);

E=(zj-zi)/(c*tao);
% F=(zj^2-zi^2)/(2*c*tao)+(c*tao)/2;
F=((xj-xs)^2+(yj-ys)^2+zj^2-(xi-xs)^2-(yi-ys)^2-zi^2)/(2*c*tao)+(c*tao)/2;
if (tao>=0)
    zs=(2*(E*F-zj)+sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
else
    zs=(2*(E*F-zj)-sqrt(4*(zj-E*F)^2-4*(E^2-1)*(F^2-(xj-xs)^2-(yj-ys)^2-zj^2)))/(2*(E^2-1));
end
end
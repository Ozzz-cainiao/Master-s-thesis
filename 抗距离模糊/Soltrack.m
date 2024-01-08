function track=Soltrack(pos1,pos2,pos3,timedelay1,timedelay2,timedelay3,zs,soundspeed)
x_differ=abs(pos1(1)-pos2(1));
y_differ=abs(pos1(2)-pos2(2));
d0=sum(pos1.^2);
d1=sum(pos2.^2);
d2=sum(pos3.^2);
r0=(timedelay1*soundspeed)^2;
r1=(timedelay2*soundspeed)^2;
r2=(timedelay3*soundspeed)^2;
x0=pos1(1);y0=pos1(2);z0=pos1(3);
x1=pos2(1);y1=pos2(2);z1=pos2(3);
x2=pos3(1);y2=pos3(2);z2=pos3(3);
if (x_differ<y_differ)
    A=-1*(x1-x0)/(y1-y0);
    B=(2*(z0-z1)*zs+d1-r1-d0+r0)/(2*(y1-y0));
    C=1+A^2;
    D=-2*(x0+A*y0-A*B);
    E=d0-r0+B^2-2*B*y0-2*z0*zs+zs^2;
    rotate=0;
else
    A=-1*(y1-y0)/(x1-x0);
    B=(2*(z0-z1)*zs+d1-r1-d0+r0)/(2*(x1-x0));
    C=1+A*A;
    D=-2*(y0+A*x0-A*B);
    E=d0-r0+B*B-2*B*x0-2*z0*zs+zs*zs;
    rotate=1;
end
v=D*D-4*E*C;
if (v>=0)
    if (rotate)
        track(2)=(-1*D+sqrt(D*D-4*C*E))/(2*C);
        track(1)=A*track(2)+B;
        track(4)=(-1*D-sqrt(D*D-4*C*E))/(2*C);
        track(3)=A*track(4)+B;
    else
        track(1)=(-1*D+sqrt(D*D-4*C*E))/(2*C);
        track(2)=A*track(1)+B;
        track(3)=(-1*D-sqrt(D*D-4*C*E))/(2*C);
        track(4)=A*track(3)+B;
    end
    if (abs(sqrt((x2-track(1))^2+(y2-track(2))^2+(z2-zs)^2)-sqrt(r2))>abs(sqrt((x2-track(3))^2+(y2-track(4))^2+(z2-zs)^2)-sqrt(r2)))
        track(1)=track(3);
        track(2)=track(4);
        track(3)=[];
        track(3)=[];
    else
        track(3)=[];
        track(3)=[];
    end
else
    track(1)=99999;
    track(2)=99999;
end
end
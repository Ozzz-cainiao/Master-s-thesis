function res=AngleCross(pos1,pos2,angle1,angle2)
pos_x1=pos1(1);pos_y1=pos1(2);
pos_x2=pos2(1);pos_y2=pos2(2);%GPS�������ƽ̨λ��

L=sqrt((pos_x1-pos_x2)^2+(pos_y1-pos_y2)^2);%���߳���

beta=(atan((pos_y2-pos_y1)/(pos_x2-pos_x1)));%��������������н� ��piΪ��λ

R1=L*abs(cos(angle2+beta)/sin(angle2-angle1));
R2=L*abs(cos(angle1+beta)/sin(angle2-angle1));%����õ���ƽ̨��Ŀ��ľ���

xx1=pos_x1+R1*sin(angle1);
xx2=pos_x2+R2*sin(angle2);
yy1=pos_y1+R1*cos(angle1);
yy2=pos_y2+R2*cos(angle2);

res(1)=(xx1+xx2)/2;
res(2)=(yy1+yy2)/2;
end
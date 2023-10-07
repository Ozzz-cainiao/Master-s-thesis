clc
clear

fclose(instrfindall);%先关闭之前可能存在的UDP

u2=udp('127.0.0.1','RemotePort',8848,'LocalPort',8849);
u3=udp('127.0.0.1','RemotePort',8848,'LocalPort',8850);

u2.DatagramReceivedFcn = @instrcallback;%设置u2接收到数据包时，调用回调函数显示
u3.DatagramReceivedFcn = @instrcallback;%设置u3接收到数据包时，调用回调函数显示

fopen(u2);%建立u2、u3连接
fopen(u3);
%--------------------u2、u3接收消息-------------------------
fscanf(u2)
fscanf(u3)

%--------------------u2、u3发送消息-------------------------
fprintf(u2,'u1 reveive data from u2');%u2发送消息给u1
fprintf(u3,'u1 reveive data from u3');%u3发送消息给u1

fclose(u2);%关闭连接
fclose(u3);

delete(u2);%删除连接
delete(u3);

clear u2;
clear u3;
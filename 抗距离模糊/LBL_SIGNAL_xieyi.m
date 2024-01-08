clc
clear
close all
channel = 2;  %ͨ����Ŀ
M = 1;        %������Ԫ��Ŀ
T_source = 1; %�������ڣ���֪���ǲ���ͬ������
dt = T_source;%�۲�����
T = 60;       %�۲�ʱ��
c = 1500;     %����
mode = 1;     %ģʽ1���˶�  ģʽ2����ֹ
snr  = 10;
d = 100;
%% ��Ԫλ��
Observer_x = [1000-d -1000+d  -1000+d  1000-d;
              1000 -1000 -1000 1000];
Observer_y = [1000 1000 -1000 -1000;
               1000-d  1000-d -1000+d  -1000+d];
%% Ŀ��켣
X = zeros(4,T/dt);             %Ŀ��켣
if mode ==1
    v = 0.1;                       %Ŀ����ٶ�
    X0 =[0,10,-0,10]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
    file_name='s_move';
else
    v = 0;                       %Ŀ����ٶ�
    X0 =[500,0,-500,0]';           %Ŀ���ʼ�˶�״̬[x,vx,y,vy]';
    file_name='s_stop';
end

%----------����ֱ���˶�-------------
F1 = [1,dt,0,0;
    0,1,0,0;
    0,0,1,dt;
    0,0,0,1,];
%----------��ʱ��ת���˶�-------------
F2 = [1,sin(v*dt)/v,0,-(1-cos(v*dt))/v;
    0,cos(v*dt),0,-sin(v*dt);
    0,(1-cos(v*dt))/v,1,sin(v*dt)/v;
    0,sin(v*dt),0,cos(v*dt)];
%----------˳ʱ��ת���˶�-------------
F3 = [1,-sin(-v*dt)/v,0,(1-cos(-v*dt))/v;
    0,cos(-v*dt),0,-sin(-v*dt);
    0,-(1-cos(-v*dt))/v,1,-sin(-v*dt)/v;
    0,sin(-v*dt),0,cos(-v*dt);];
%-----------������ʵ�켣----------
k=1;
for t = dt:dt:T
    if t==dt
        X(:,k)=X0;
    elseif t>dt&&t<20
        X(:,k)=F1*X(:,k-1);                          %�����˶�����
    elseif t>=20&&t<35
        if mode == 2
            F=F1;
        else
            F=F2;
        end
        X(:,k)=F*X(:,k-1);                          %��ʱ��ת���˶�����
    elseif t>=35&&t<=T
        X(:,k)=F1*X(:,k-1);                          %�����˶�����
    end
    k=k+1;
end
figure
hold on
for i=1:size(Observer_x,1)
    scatter(Observer_x(i,:),Observer_y(i,:),50,'r','p')
end
grid on
axis([-2000 2000 -2000 2000]);
% plot(X(1,:),X(3,:),'Color','b','LineWidth',1)
plot(X(1,:),X(3,:),'b.','MarkerSize',3)
scatter(X0(1,:),X0(3,:),100,'k','x','LineWidth',1.5)
legend('��Ԫ','Ŀ��켣','��ʼλ��')
xlabel('X/m');ylabel('Y/m');title('Ŀ�꺽������Ԫλ��')
%% ����ʱ��
r = zeros(channel*M,T/dt);             %Ŀ�������Ԫ�ľ���
time = zeros(channel*M,T/dt);          %Ŀ���źŵ���ʱ��
T_fix = 0:dt:T-dt;
for j=1:channel
    for i = 1:M
        r   ((j-1)*M+i,:)=sqrt( (X(1,:)-Observer_x(j,i)).^2 + (X(3,:)-Observer_y(j,i)).^2 );
        time((j-1)*M+i,:) = r((j-1)*M+i,:)/c + T_fix;       
    end        
end

%% �����ź�
T0 = 0.05;                    %����
f0 = 10e3;                     %Ƶ��
fs = 120e3;                   %������
ts = 1/fs:1/fs:T0;
T0_num = T0*fs;               %�����Ӧ�������� 
S = zeros(M*channel,T*fs);            %�����ź�
s_no = zeros(1,channel*M*T*fs);       %�������ź�
s_exist = zeros(1,channel*M*T*fs);    %�������ź�
signal_start = ceil(time*fs);
signal_end = signal_start+T0_num-1;
signal_end(signal_end>T*fs) = T*fs;
signal_end(signal_start>T*fs)=1;
s  = cos(2*pi*f0*ts);  
s_ch1 = zeros(M,T*fs);       %ͨ��1�ź�
s_ch2 = zeros(M,T*fs);       %ͨ��2�ź�
for i = 1:size(time,1)
    for j = 1:T
        ind = signal_start(i,j):signal_end(i,j);
        S(i,ind) = s(1,1:length(ind));
    end
    if i <=M
        s_ch1(i  ,:) = S(i,:);
    else
        s_ch2(i-M,:) = S(i,:);
    end
end
%%              Э��
ADB_HEAD     = 23205;
ADB_CMD_TYPE = 65534;
ADB_RSV      = 0;
ADB_SYS_TYPE = 1;
ADB_MS_CNT   = 0;
ADB_HEADING  = 1;
ADB_PITCH    = 2;
ADB_ROLL     = 3;
ADB_DEPTH    = 4;
ADB_LEN      = 480;
ADB_DATA     = 0;
ADB_CHK      = 0;
Fs=1;
s_no(1,1:2:end)=s_ch1;
s_no(1,2:2:end)=s_ch2;
s_no=round(s_no*32768/2);
for i = 1:M
    s_ch1(i,:) = awgn(s_ch1(i,:),snr);
    s_ch2(i,:) = awgn(s_ch2(i,:),snr);
end
max_s1=s_ch1(1,:);
max_s2=s_ch2(1,:);
for i = 2:M
    max_s1=[max_s1 s_ch1(i,:)];
    max_s2=[max_s2 s_ch2(i,:)];
end
max_s1 = max(max_s1);
max_s2 = max(max_s2);
for i = 1:M
    s_ch1(i,:) = s_ch1(i,:)/max_s1;
    s_ch2(i,:) = s_ch2(i,:)/max_s2;
end
s_exist(1,1:2:end)=s_ch1;
s_exist(1,2:2:end)=s_ch2;
% figure
% plot(1/fs:1/fs:T,s_no(1,:))
%% �洢�������ļ�

%д��ά�����������ļ�
DATA_LENGTH=0.001*fs*2;
fip=fopen(['.\',file_name,'_no.bin'],'wb');
for i = 898:T/0.001
    fwrite(fip,ADB_HEAD,'uint16','b');
    fwrite(fip,ADB_CMD_TYPE,'uint16','b');
    fwrite(fip,ADB_RSV,'uint8','b');
    fwrite(fip,ADB_SYS_TYPE,'uint8','b');
    fwrite(fip,ADB_MS_CNT,'uint16','b');
    ADB_MS_CNT = ADB_MS_CNT+1;
    if ADB_MS_CNT>999
        ADB_MS_CNT=0;
    end
    fwrite(fip,ADB_HEADING,'uint32','b');
    fwrite(fip,ADB_PITCH,'uint32','b');
    fwrite(fip,ADB_ROLL,'uint32','b');
    fwrite(fip,ADB_DEPTH,'uint16','b');
    fwrite(fip,ADB_LEN,'uint16','b');
    ADB_DATA=s_no(DATA_LENGTH*(i-1)+1:DATA_LENGTH*i);
    fwrite(fip,ADB_DATA,'int16','b');
    fwrite(fip,ADB_CHK,'uint32','b');
end
fclose(fip); %����ָ���ֵΪ0�����ʾ�洢��������
% %д��ά�����������ļ�
% fip=fopen(['.\',file_name,'_exist.bin'],'wb');
% fwrite(fip,M,'int32');
% fwrite(fip,channel,'int32');
% fwrite(fip,fs,'int32');
% fwrite(fip,T_source,'int32');
% fwrite(fip,s_exist,'float');
% fclose(fip); %����ָ���ֵΪ0�����ʾ�洢��������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                xyת��Ϊ��γ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input: (long1,lat1)ԭ�㾭γ�ȣ���.�ȣ�
%        (x,y)Ŀ�꾭γ��
%        unit:���뵥λ  0-rad  1-��  2-��  3-��
%output: Ŀ���˹����
%lat:     Ŀ���γ��
%lon:     Ŀ��ľ���
function  [lat, long] = ACalculateDistance(long1, lat1, x, y,unit)
    [m_Long_sec1, m_Lat_sec1]=CGeopos(lat1,long1,unit);
    [x1, y1]=GeoToGauss(m_Long_sec1, m_Lat_sec1);  
    x2=x+x1;
    y2=y+y1;
    
    [m_Lat_sec2, m_Long_sec2]=GaussToGeo(x2,y2);
    long=m_Long_sec2/3600;
    lat=m_Lat_sec2/3600;



%     out=[dx dy dr th];   %dr��������Ŀ�굽ԭ��ľ���
    %###############################################################
end

function [m_Long_sec, m_Lat_sec]=CGeopos(lat,lon,unit)
DH_width=6;%���ȴ�
switch unit
    case 0
        m_Lat_rad=lat;
        m_Long_rad=lon;
        m_Lat_deg=lat*180/pi;
        m_Long_deg=lon*180/pi;
        m_Lat_min=lat*180*60/pi;
        m_Long_min=lon*180*60/pi;
        m_Lat_sec=lat*180*3600/pi;
        m_Long_sec=lon*180*3600/pi;
        return;
    case 1
        m_Lat_rad=lat*pi/180;
        m_Long_rad=lon*pi/180;
        m_Lat_deg=lat;
        m_Long_deg=lon;
        m_Lat_min=lat*60;
        m_Long_min=lon*60;
        m_Lat_sec=lat*3600;
        m_Long_sec=lon*3600;
        return;
    case 2
        m_Lat_rad=lat*pi/180/60;
        m_Long_rad=lon*pi/180/60;
        m_Lat_deg=lat/60;
        m_Long_deg=lon/60;
        m_Lat_min=lat;
        m_Long_min=lon;
        m_Lat_sec=lat*60;
        m_Long_sec=lon*60;
        return;
    case 3
        m_Lat_rad=lat*pi/180/3600;
        m_Long_rad=lon*pi/180/3600;
        m_Lat_deg=lat/3600;
        m_Long_deg=lon/3600;
        m_Lat_min=lat/60;
        m_Long_min=lon/60;
        m_Lat_sec=lat;
        m_Long_sec=lon;
        return;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /* ����˵���� �����Ը�˹����(y,x)ת���ɾ��Եĵ�������(wd,jd)��        */
% // double y;     �������: ��˹����ĺ����꣬����Ϊ��λ
% // double x;  �������: ��˹����������꣬����Ϊ��λ
% // short  DH;     �������: ���ţ���ʾ������˹�������ĸ�����
% // double *LongSecond;     �������: ָ�򾭶������ָ�룬���о�����������Ϊ��λ
% // double *LatSecond;     �������: ָ��γ�������ָ�룬����γ����������Ϊ��λ
% out = [LatSecond LongSecond]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [LatSecond LongSecond] = GaussToGeo(xx, yy)
    x=yy;
	y=xx;    
    DH=20;
    DH_width=6;
    LP= -1000;
    b_e2=0.0067385254147;
    b_c=6399698.90178271;

    y = y-500000;
	X_3 = x / 1000000.00  - 3 ;      % �����ף�1000000��Ϊ��λ
	% ���ڿ�������˹�����򣬼���Bf0
	Bf0 = 27.11115372595 + 9.02468257083 * X_3 - 0.00579740442 * X_3^2 ...
		- 0.00043532572 * X_3^3 + 0.00004857285 * X_3^4 ... 
		+ 0.00000215727 * X_3^5 - 0.00000019399 * X_3^6 ;
	tf = tan(Bf0*pi/180);       %  tf = tg(Bf),ע�����ｫBfת�����Ի���Ϊ��λ
	etf = b_e2 * cos(Bf0*pi/180)^2;   %  etf = e'**2 * cos(Bf) ** 2
	nf = y * sqrt( 1 + etf ) / b_c;     %  n = y * sqrt( 1 + etf ** 2) / c
	% ����γ�ȣ�ע�������������Ľ�����Զ�Ϊ��λ��
	t_B0 = Bf0 - (1.0+etf) * tf / pi * (90.0 * nf^2 ...
		- 7.5 * (5.0 + 3 * tf^2 + etf - 9 * etf * tf^2) * nf^4 ...
		+ 0.25 * (61 + 90 * tf^2 + 45 * tf^4) * nf^6);
	% ���㾭�ע�������������Ľ�����Զ�Ϊ��λ��
	t_l0 = (180 * nf - 30 * ( 1 + 2 * tf^2 + etf ) * nf^3 ...           
		+ 1.5 * (5 + 28 * tf^2 + 24 * tf^4) * nf^5)  ...       
		/ ( pi * cos(Bf0*pi/180) ) ;
	l0 = (t_l0 * 3600.0);       %  ������ת����
	
	if (LP == -1000)

		LongSecond = ((DH * DH_width - DH_width/2) * 3600.0 + l0);  % ���ݴ��ż��������Ϊ��λ�ľ��Ծ��ȣ�����ָ��

	else

		LongSecond = LP * 3600.0 + l0;  % ���ݴ��ż��������Ϊ��λ�ľ��Ծ��ȣ�����ָ��
    end
	%----------------------------------
	LatSecond = t_B0 * 3600.0;     %  ��γ��ת���룬������ָ��
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	t;     %  t=tgB
% 	L;     %  ���뾭�ߵľ���
% 	l0;    %  ����
%     wd_hd;  %  ��jd��wdת�����Ի���Ϊ��λ
% 	et2;    %  et2 = (e' ** 2) * (cosB ** 2)
% 	N;     %  N = C / sqrt(1 + et2)
% 	X;     %  ��������˹�����������绡��
% 	m;     %  m = cosB * PI/180 * l0
% 	tsin,tcos;   %  sinB,cosB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x y]=GeoToGauss(jd, wd)
DH=20;
DH_width=6;
b_e2=0.0067385254147;
b_c=6399698.90178271;

wd_hd = wd / 3600.0 * pi / 180.0 ;    % ������Ϊ��λ��γ��ת���ɻ���

% 	% ����������뾭�ߣ�ȱʡ����: -1000������������뾭�ߣ�
% 	% ����ʹ�ô�������뾭�ߣ�����ʹ�ô��źʹ������
% 	%L = (DH - 0.5) * DH_width ;      % �������뾭�ߵľ���
% %	if (LP == -1000)
% %	{
L = (DH - 0.5) * DH_width ;      % �������뾭�ߵľ���
% %	}
% %	else
% %	{
% %		L = LP ;
% %	}
%
l0 = jd/3600.0 - L  ;       % ���㾭��
tsin = sin(wd_hd);        % ����sinB
tcos = cos(wd_hd);        % ����cosB
% 	% �����������˹�����������绡��X
X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * (tsin^3)...
    + 0.6976 * (tsin^5) + 0.0039 * (tsin^7) ) * tcos;
et2 = b_e2 * (tcos^2) ;      %  et2 = (e' ** 2) * (cosB ** 2)
N  = b_c / sqrt( 1 + et2 ) ;      %  N = C / sqrt(1 + et2)
t  = tan(wd_hd);         %  t=tgB
m  = pi/180 * l0 * tcos;       %  m = cosB * pi/180 * l0
y = X + N * t * ( 0.5 * (m^2) ...
    + (5.0 - (t^2) + 9.0 * et2 + 4 * (et2^2)) * (m^4)/24.0...
    + (61.0 - 58.0 * (t^2) + (t^4)) * (m^6) / 720.0 ) ;
x = N * ( m + ( 1.0 - (t^2) + et2 ) * (m^3) / 6.0...
    + ( 5.0 - 18.0 * (t^2) + (t^4) + 14.0 * et2...
    - 58.0 * et2 * (t^2) ) * (m^5) / 120.0 )+500000;    %ע��+500000
end
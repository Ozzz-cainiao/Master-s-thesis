%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                xy转换为经纬坐标
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input: (long1,lat1)原点经纬度（度.度）
%        (x,y)目标经纬度
%        unit:输入单位  0-rad  1-度  2-分  3-秒
%output: 目标高斯坐标
%lat:     目标的纬度
%lon:     目标的经度
function  [lat, long] = ACalculateDistance(long1, lat1, x, y,unit)
    [m_Long_sec1, m_Lat_sec1]=CGeopos(lat1,long1,unit);
    [x1, y1]=GeoToGauss(m_Long_sec1, m_Lat_sec1);  
    x2=x+x1;
    y2=y+y1;
    
    [m_Lat_sec2, m_Long_sec2]=GaussToGeo(x2,y2);
    long=m_Long_sec2/3600;
    lat=m_Lat_sec2/3600;



%     out=[dx dy dr th];   %dr在这里是目标到原点的距离
    %###############################################################
end

function [m_Long_sec, m_Lat_sec]=CGeopos(lat,lon,unit)
DH_width=6;%六度带
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
% /* 功能说明： 将绝对高斯坐标(y,x)转换成绝对的地理坐标(wd,jd)。        */
% // double y;     输入参数: 高斯坐标的横坐标，以米为单位
% // double x;  输入参数: 高斯坐标的纵坐标，以米为单位
% // short  DH;     输入参数: 带号，表示上述高斯坐标是哪个带的
% // double *LongSecond;     输出参数: 指向经度坐标的指针，其中经度坐标以秒为单位
% // double *LatSecond;     输出参数: 指向纬度坐标的指针，其中纬度坐标以秒为单位
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
	X_3 = x / 1000000.00  - 3 ;      % 以兆米（1000000）为单位
	% 对于克拉索夫斯基椭球，计算Bf0
	Bf0 = 27.11115372595 + 9.02468257083 * X_3 - 0.00579740442 * X_3^2 ...
		- 0.00043532572 * X_3^3 + 0.00004857285 * X_3^4 ... 
		+ 0.00000215727 * X_3^5 - 0.00000019399 * X_3^6 ;
	tf = tan(Bf0*pi/180);       %  tf = tg(Bf),注意这里将Bf转换成以弧度为单位
	etf = b_e2 * cos(Bf0*pi/180)^2;   %  etf = e'**2 * cos(Bf) ** 2
	nf = y * sqrt( 1 + etf ) / b_c;     %  n = y * sqrt( 1 + etf ** 2) / c
	% 计算纬度，注意这里计算出来的结果是以度为单位的
	t_B0 = Bf0 - (1.0+etf) * tf / pi * (90.0 * nf^2 ...
		- 7.5 * (5.0 + 3 * tf^2 + etf - 9 * etf * tf^2) * nf^4 ...
		+ 0.25 * (61 + 90 * tf^2 + 45 * tf^4) * nf^6);
	% 计算经差，注意这里计算出来的结果是以度为单位的
	t_l0 = (180 * nf - 30 * ( 1 + 2 * tf^2 + etf ) * nf^3 ...           
		+ 1.5 * (5 + 28 * tf^2 + 24 * tf^4) * nf^5)  ...       
		/ ( pi * cos(Bf0*pi/180) ) ;
	l0 = (t_l0 * 3600.0);       %  将经差转成秒
	
	if (LP == -1000)

		LongSecond = ((DH * DH_width - DH_width/2) * 3600.0 + l0);  % 根据带号计算出以秒为单位的绝对经度，返回指针

	else

		LongSecond = LP * 3600.0 + l0;  % 根据带号计算出以秒为单位的绝对经度，返回指针
    end
	%----------------------------------
	LatSecond = t_B0 * 3600.0;     %  将纬差转成秒，并返回指针
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	t;     %  t=tgB
% 	L;     %  中央经线的经度
% 	l0;    %  经差
%     wd_hd;  %  将jd、wd转换成以弧度为单位
% 	et2;    %  et2 = (e' ** 2) * (cosB ** 2)
% 	N;     %  N = C / sqrt(1 + et2)
% 	X;     %  克拉索夫斯基椭球中子午弧长
% 	m;     %  m = cosB * PI/180 * l0
% 	tsin,tcos;   %  sinB,cosB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x y]=GeoToGauss(jd, wd)
DH=20;
DH_width=6;
b_e2=0.0067385254147;
b_c=6399698.90178271;

wd_hd = wd / 3600.0 * pi / 180.0 ;    % 将以秒为单位的纬度转换成弧度

% 	% 如果不设中央经线（缺省参数: -1000），则计算中央经线，
% 	% 否则，使用传入的中央经线，不再使用带号和带宽参数
% 	%L = (DH - 0.5) * DH_width ;      % 计算中央经线的经度
% %	if (LP == -1000)
% %	{
L = (DH - 0.5) * DH_width ;      % 计算中央经线的经度
% %	}
% %	else
% %	{
% %		L = LP ;
% %	}
%
l0 = jd/3600.0 - L  ;       % 计算经差
tsin = sin(wd_hd);        % 计算sinB
tcos = cos(wd_hd);        % 计算cosB
% 	% 计算克拉索夫斯基椭球中子午弧长X
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
    - 58.0 * et2 * (t^2) ) * (m^5) / 120.0 )+500000;    %注意+500000
end
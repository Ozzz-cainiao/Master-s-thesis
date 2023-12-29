#include "mex.h"
#include "math.h"

//功能函数
void SolTrack(double TimeDelay[3],double Buoy_point[9],double zs,double track[5],double SoundSpeed,int BuoyNum)
{
    //新的
    double x0[3],y0[3],z0[3];
    double d[3],r[3],A,B,C,D,E,d21,d31,g11,g21,g12,g22;
    double j1,j2,h1,h2,p,px,py,qx,qy,aa,bb,cc,v,d1,d2;
    double dis_bo_tr;
    double x_differ=fabs(Buoy_point[0]-Buoy_point[3]);
    double y_differ=fabs(Buoy_point[1]-Buoy_point[4]);
    bool rotate;
    if(BuoyNum==2)
    {
        x0[0]=Buoy_point[0];
        x0[1]=Buoy_point[3];

        y0[0]=Buoy_point[1];
        y0[1]=Buoy_point[4];

        z0[0]=Buoy_point[2];
        z0[1]=Buoy_point[5];

        d[0]=x0[0]*x0[0]+y0[0]*y0[0]+z0[0]*z0[0];
        d[1]=x0[1]*x0[1]+y0[1]*y0[1]+z0[1]*z0[1];

        r[0]=(TimeDelay[0]*SoundSpeed)*(TimeDelay[0]*SoundSpeed);
        r[1]=(TimeDelay[1]*SoundSpeed)*(TimeDelay[1]*SoundSpeed);

        if(x_differ<y_differ)
        {

            A=-1*(x0[1]-x0[0])/(y0[1]-y0[0]);
            B=(2*(z0[0]-z0[1])*zs+d[1]-r[1]-d[0]+r[0])/(2*(y0[1]-y0[0]));
            C=1+A*A;
            D=-2*(x0[0]+A*y0[0]-A*B);
            E=d[0]-r[0]+B*B-2*B*y0[0]-2*z0[0]*zs+zs*zs;
            rotate=false;
        }
        else
        {
            A=-1*(y0[1]-y0[0])/(x0[1]-x0[0]);
            B=(2*(z0[0]-z0[1])*zs+d[1]-r[1]-d[0]+r[0])/(2*(x0[1]-x0[0]));
            C=1+A*A;
            D=-2*(y0[0]+A*x0[0]-A*B);
            E=d[0]-r[0]+B*B-2*B*x0[0]-2*z0[0]*zs+zs*zs;
            rotate=true;
        }
        if(D*D-4*E*C>=0)
        {
            if(!rotate)
            {
                track[0]=(-1*D+sqrt(D*D-4*C*E))/(2*C);
                track[1]=A*track[0]+B;
                track[2]=(-1*D-sqrt(D*D-4*C*E))/(2*C);
                track[3]=A*track[2]+B;
            }
            else
            {
                track[1]=(-1*D+sqrt(D*D-4*C*E))/(2*C);
                track[0]=A*track[1]+B;
                track[3]=(-1*D-sqrt(D*D-4*C*E))/(2*C);
                track[2]=A*track[3]+B;
            }
        }
        else
        {
            track[0]=0xffff;
            track[1]=0xffff;
            track[2]=0xffff;
            track[3]=0xffff;
        }

    }
    if(BuoyNum==3)
    {
        x0[0]=Buoy_point[0];
        x0[1]=Buoy_point[3];
        x0[2]=Buoy_point[6];
        y0[0]=Buoy_point[1];
        y0[1]=Buoy_point[4];
        y0[2]=Buoy_point[7];
        z0[0]=Buoy_point[2];
        z0[1]=Buoy_point[5];
        z0[2]=Buoy_point[8];
        d[0]=x0[0]*x0[0]+y0[0]*y0[0]+z0[0]*z0[0];
        d[1]=x0[1]*x0[1]+y0[1]*y0[1]+z0[1]*z0[1];
        d[2]=x0[2]*x0[2]+y0[2]*y0[2]+z0[2]*z0[2];
        r[0]=(TimeDelay[0]*SoundSpeed)*(TimeDelay[0]*SoundSpeed);
        r[1]=(TimeDelay[1]*SoundSpeed)*(TimeDelay[1]*SoundSpeed);
        r[2]=(TimeDelay[2]*SoundSpeed)*(TimeDelay[2]*SoundSpeed);
        if(x_differ<y_differ)
        {

            A=-1*(x0[1]-x0[0])/(y0[1]-y0[0]);
            B=(2*(z0[0]-z0[1])*zs+d[1]-r[1]-d[0]+r[0])/(2*(y0[1]-y0[0]));
            C=1+A*A;
            D=-2*(x0[0]+A*y0[0]-A*B);
            E=d[0]-r[0]+B*B-2*B*y0[0]-2*z0[0]*zs+zs*zs;
            rotate=false;
        }
        else
        {
            A=-1*(y0[1]-y0[0])/(x0[1]-x0[0]);
            B=(2*(z0[0]-z0[1])*zs+d[1]-r[1]-d[0]+r[0])/(2*(x0[1]-x0[0]));
            C=1+A*A;
            D=-2*(y0[0]+A*x0[0]-A*B);
            E=d[0]-r[0]+B*B-2*B*x0[0]-2*z0[0]*zs+zs*zs;
            rotate=true;
        }
        v=D*D-4*E*C;
        if(v>=0)
        {
            if(!rotate)
            {
                track[0]=(-1*D+sqrt(D*D-4*C*E))/(2*C);
                track[1]=A*track[0]+B;
                track[2]=(-1*D-sqrt(D*D-4*C*E))/(2*C);
                track[3]=A*track[2]+B;
            }
            else
            {
                track[1]=(-1*D+sqrt(D*D-4*C*E))/(2*C);
                track[0]=A*track[1]+B;
                track[3]=(-1*D-sqrt(D*D-4*C*E))/(2*C);
                track[2]=A*track[3]+B;
            }
            if(fabs(sqrt((x0[2]-track[0])*(x0[2]-track[0])+(y0[2]-track[1])*(y0[2]-track[1])+(z0[2]-zs)*(z0[2]-zs))-sqrt(r[2])) > fabs(sqrt((x0[2]-track[2])*(x0[2]-track[2])+(y0[2]-track[3])*(y0[2]-track[3])+(z0[2]-zs)*(z0[2]-zs))-sqrt(r[2]))    )
            {
                track[0]=track[2];
                track[1]=track[3];
                track[2]=0xffff;
                track[3]=0xffff;
            }
            else
            {

                track[2]=0xffff;
                track[3]=0xffff;
            }
        }
        else
        {
            track[0]=0xffff;
            track[1]=0xffff;
            track[2]=0xffff;
            track[3]=0xffff;
        }
    }

    if(BuoyNum==4)
    {

        x0[0]=Buoy_point[0];
        x0[1]=Buoy_point[3];
        x0[2]=Buoy_point[6];
        y0[0]=Buoy_point[1];
        y0[1]=Buoy_point[4];
        y0[2]=Buoy_point[7];
        z0[0]=Buoy_point[2];
        z0[1]=Buoy_point[5];
        z0[2]=Buoy_point[8];
        r[0]=sqrt(x0[0]*x0[0]+y0[0]*y0[0]+z0[0]*z0[0]);
        r[1]=sqrt(x0[1]*x0[1]+y0[1]*y0[1]+z0[1]*z0[1]);
        r[2]=sqrt(x0[2]*x0[2]+y0[2]*y0[2]+z0[2]*z0[2]);
        d[0]=TimeDelay[0]*SoundSpeed;
        d[1]=TimeDelay[1]*SoundSpeed;
        d[2]=TimeDelay[2]*SoundSpeed;

        d21=d[1]-d[0];
        d31=d[2]-d[0];
        g11=2*(x0[1]-x0[0]);
        g21=2*(x0[2]-x0[0]);
        g12=2*(y0[1]-y0[0]);
        g22=2*(y0[2]-y0[0]);
        //j1=r[1]*r[1]-r[0]*r[0]-d21*d21;
        //j2=r[2]*r[2]-r[0]*r[0]-d31*d31;
        j1=r[1]*r[1]-r[0]*r[0]-d21*d21-2*(z0[1]-z0[0])*zs;//Lin
        j2=r[2]*r[2]-r[0]*r[0]-d31*d31-2*(z0[2]-z0[0])*zs;//Lin
        h1=2*d21;
        h2=2*d31;
        p=g11*g22-g21*g12;
        px=(h2*g12-h1*g22)/p;
        py =(h1*g21-h2*g11)/p;
        qx =(j1*g22-j2*g12)/p;
        qy =(j2*g11-j1*g21)/p;
        aa =px *px+py *py-1;
        bb =2*(px *qx -px *x0[0]+py *qy -py *y0[0]);
        cc =(x0[0]-qx )*(x0[0]-qx )+(y0[0]-qy )*(y0[0]-qy )+(z0[0]-zs)*(z0[0]-zs);
        v =(bb *bb-4*aa *cc );
        if(v>=0)
        {
            d1 =(-bb -sqrt(bb *bb-4*aa *cc ))/(2*aa );
            d2 =(-bb +sqrt(bb *bb-4*aa *cc ))/(2*aa );

            if(d1>0 && d2<0)
            {
                track[0] =px *d1 +qx ;
                track[1] =py *d1 +qy ;
                track[2]=0xffff;
                track[3]=0xffff;
            }
            else if(d2>0 && d1<0)
            {
                track[0] =px *d2 +qx ;
                track[1] =py *d2 +qy ;
                track[2]=0xffff;
                track[3]=0xffff;
            }
            else if(d1>0 && d2>0)
            {
                track[0] =px *d1 +qx ;
                track[1] =py *d1 +qy ;
                track[2] =px *d2 +qx ;
                track[3] =py *d2 +qy ;
            }
            dis_bo_tr=sqrt((x0[0]-track[0])*(x0[0]-track[0])+(y0[0]-track[1])*(y0[0]-track[1])+(z0[0]-zs)*(z0[0]-zs));
            track[4]=dis_bo_tr/SoundSpeed;
        }
        else
        {
            track[0]=0xffff;
            track[1]=0xffff;
            track[2]=0xffff;
            track[3]=0xffff;
        }
    }
}

/*下面这个mexFunction的目的是使MATLAB知道如何调用这个timestwo函数*/
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )

/* nlhs是MATLAB命令行方式下输出参数的个数；
   *plhs[]是MATLAB命令行方式下的输出参数；
   nrhs是MATLAB命令行方式下输入参数的个数；
   *prhs[]是MATLAB命令行方式下的输入参数； */

{
    size_t mrows,ncols;
    /* Check for proper number of arguments. */
    if(nrhs!=5)
    {
        mexErrMsgTxt("Five input required.");
    }
    else if(nlhs>1)
    {
        mexErrMsgTxt("Too many output arguments");
    }

    /* 在MATLAB命令行方式下，本MEX文件的调用格式是
     * track[5] = SolTrack(TimeDelay[3], Buoy_point[9], zs, SoundSpeed, BuoyNum)
     * 输入参数（x）个数＝5，输出参数（y）个数＝1，所以在程序一
     * 开始就检查nrhs是否＝1以及nlhs是否>1（因为MATLAB有一个缺省输出参数ans，所以nlhs可以=0） */
    /* The input must be a noncomplex scalar double.*/

    for(int i = 0; i < nrhs; i++)
    {
        //判断输入数据类型是否为double
        if( !mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) )
        {
            mexErrMsgTxt("Input must be a noncomplex scalar double.");
        }

        //判断输入数据纬度
        mrows = mxGetM(prhs[i]); /* 获得输入矩阵的行数 */
        ncols = mxGetN(prhs[i]); /* 获得输入矩阵的列数 */

        size_t defaultRows=1,defaultCols=1;
        switch(i)
        {
        case 0: defaultCols = 3;break;
        case 1: defaultCols = 9;break;
        default: break;
        }

        if(mrows != defaultRows || ncols != defaultCols)
        {
            mexErrMsgTxt("Input must be a noncomplex scalar double.");
        }
    }

    /* 为输出创佳一个矩阵，显然这个矩阵也应该是1x5的 */
    plhs[0] = mxCreateDoubleMatrix(1, 5, mxREAL);
    /* 调用C++函数timestwo(y,x) */
    SolTrack(mxGetPr(prhs[0]), mxGetPr(prhs[1]), *mxGetPr(prhs[2]), mxGetPr(plhs[0]),
             *mxGetPr(prhs[3]), ((int)((*mxGetPr(prhs[4]))+0.5)));
}

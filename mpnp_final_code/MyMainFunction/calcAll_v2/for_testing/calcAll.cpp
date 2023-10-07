//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Sat Sep 23 09:43:19 2023
// Arguments:
// "-B""macro_default""-W""cpplib:calcAll,all,version=1.0""-T""link:lib""-d""E:\
// MPNP\MyMainFunction\calcAll_v2\for_testing""-v""E:\MPNP\MyMainFunction\calcAl
// l.m"
//

#define EXPORTING_calcAll 1
#include "calcAll.h"

static HMCRINSTANCE _mcr_inst = NULL; /* don't use nullptr; this may be either C or C++ */

#if defined( _MSC_VER) || defined(__LCC__) || defined(__MINGW64__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define NOMINMAX
#include <windows.h>
#undef interface

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern C block */
#endif

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern C block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_calcAll_C_API
#define LIB_calcAll_C_API /* No special import/export declaration */
#endif

LIB_calcAll_C_API 
bool MW_CALL_CONV calcAllInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst)
        return true;
    if (!mclmcrInitialize())
        return false;
    if (!GetModuleFileName(GetModuleHandle("calcAll"), path_to_dll, _MAX_PATH))
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream(path_to_dll);
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_calcAll_C_API 
bool MW_CALL_CONV calcAllInitialize(void)
{
    return calcAllInitializeWithHandlers(mclDefaultErrorHandler, mclDefaultPrintHandler);
}

LIB_calcAll_C_API 
void MW_CALL_CONV calcAllTerminate(void)
{
    if (_mcr_inst)
        mclTerminateInstance(&_mcr_inst);
}

LIB_calcAll_C_API 
void MW_CALL_CONV calcAllPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_calcAll_C_API 
bool MW_CALL_CONV mlxCalcAll(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "calcAll", nlhs, plhs, nrhs, prhs);
}

LIB_calcAll_CPP_API 
void MW_CALL_CONV calcAll(int nargout, mwArray& outLoctionCAX, mwArray& outLoctionCAY, 
                          mwArray& outLoctionSPCX, mwArray& outLoctionSPCY, const 
                          mwArray& arrR, const mwArray& pNum, const mwArray& node, const 
                          mwArray& num, const mwArray& t_obs, const mwArray& T)
{
    mclcppMlfFeval(_mcr_inst, "calcAll", nargout, 4, 6, &outLoctionCAX, &outLoctionCAY, &outLoctionSPCX, &outLoctionSPCY, &arrR, &pNum, &node, &num, &t_obs, &T);
}


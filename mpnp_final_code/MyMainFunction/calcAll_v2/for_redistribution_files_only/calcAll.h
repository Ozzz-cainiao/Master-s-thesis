//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Sat Sep 23 09:43:19 2023
// Arguments:
// "-B""macro_default""-W""cpplib:calcAll,all,version=1.0""-T""link:lib""-d""E:\
// MPNP\MyMainFunction\calcAll_v2\for_testing""-v""E:\MPNP\MyMainFunction\calcAl
// l.m"
//

#ifndef calcAll_h
#define calcAll_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_calcAll_C_API 
#define LIB_calcAll_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_calcAll_C_API 
bool MW_CALL_CONV calcAllInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_calcAll_C_API 
bool MW_CALL_CONV calcAllInitialize(void);

extern LIB_calcAll_C_API 
void MW_CALL_CONV calcAllTerminate(void);

extern LIB_calcAll_C_API 
void MW_CALL_CONV calcAllPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_calcAll_C_API 
bool MW_CALL_CONV mlxCalcAll(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_calcAll
#define PUBLIC_calcAll_CPP_API __declspec(dllexport)
#else
#define PUBLIC_calcAll_CPP_API __declspec(dllimport)
#endif

#define LIB_calcAll_CPP_API PUBLIC_calcAll_CPP_API

#else

#if !defined(LIB_calcAll_CPP_API)
#if defined(LIB_calcAll_C_API)
#define LIB_calcAll_CPP_API LIB_calcAll_C_API
#else
#define LIB_calcAll_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_calcAll_CPP_API void MW_CALL_CONV calcAll(int nargout, mwArray& outLoctionCAX, mwArray& outLoctionCAY, mwArray& outLoctionSPCX, mwArray& outLoctionSPCY, const mwArray& arrR, const mwArray& pNum, const mwArray& node, const mwArray& num, const mwArray& t_obs, const mwArray& T);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

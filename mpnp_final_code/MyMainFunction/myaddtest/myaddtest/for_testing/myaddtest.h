//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Thu Sep 21 14:46:02 2023
// Arguments:
// "-B""macro_default""-W""cpplib:myaddtest,all,version=1.0""-T""link:lib""-d""E
// :\MPNP\MyMainFunction\myaddtest\myaddtest\for_testing""-v""E:\MPNP\MyMainFunc
// tion\myadd.m"
//

#ifndef myaddtest_h
#define myaddtest_h 1

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
#ifndef LIB_myaddtest_C_API 
#define LIB_myaddtest_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_myaddtest_C_API 
bool MW_CALL_CONV myaddtestInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_myaddtest_C_API 
bool MW_CALL_CONV myaddtestInitialize(void);

extern LIB_myaddtest_C_API 
void MW_CALL_CONV myaddtestTerminate(void);

extern LIB_myaddtest_C_API 
void MW_CALL_CONV myaddtestPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_myaddtest_C_API 
bool MW_CALL_CONV mlxMyadd(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_myaddtest
#define PUBLIC_myaddtest_CPP_API __declspec(dllexport)
#else
#define PUBLIC_myaddtest_CPP_API __declspec(dllimport)
#endif

#define LIB_myaddtest_CPP_API PUBLIC_myaddtest_CPP_API

#else

#if !defined(LIB_myaddtest_CPP_API)
#if defined(LIB_myaddtest_C_API)
#define LIB_myaddtest_CPP_API LIB_myaddtest_C_API
#else
#define LIB_myaddtest_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_myaddtest_CPP_API void MW_CALL_CONV myadd(int nargout, mwArray& c, const mwArray& a, const mwArray& b);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

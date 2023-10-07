//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Thu Sep 21 11:51:27 2023
// Arguments: "-B""macro_default""-W""cpplib:addtest""-T""link:lib""myadd.m""-C"
//

#ifndef addtest_h
#define addtest_h 1

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
#ifndef LIB_addtest_C_API 
#define LIB_addtest_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_addtest_C_API 
bool MW_CALL_CONV addtestInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_addtest_C_API 
bool MW_CALL_CONV addtestInitialize(void);

extern LIB_addtest_C_API 
void MW_CALL_CONV addtestTerminate(void);

extern LIB_addtest_C_API 
void MW_CALL_CONV addtestPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_addtest_C_API 
bool MW_CALL_CONV mlxMyadd(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_addtest
#define PUBLIC_addtest_CPP_API __declspec(dllexport)
#else
#define PUBLIC_addtest_CPP_API __declspec(dllimport)
#endif

#define LIB_addtest_CPP_API PUBLIC_addtest_CPP_API

#else

#if !defined(LIB_addtest_CPP_API)
#if defined(LIB_addtest_C_API)
#define LIB_addtest_CPP_API LIB_addtest_C_API
#else
#define LIB_addtest_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_addtest_CPP_API void MW_CALL_CONV myadd(int nargout, mwArray& c, const mwArray& a, const mwArray& b);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

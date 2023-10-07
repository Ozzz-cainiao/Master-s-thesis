//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Fri Sep 22 16:59:37 2023
// Arguments:
// "-B""macro_default""-W""cpplib:testMulti,all,version=1.0""-T""link:lib""-d""E
// :\MPNP\MyMainFunction\testMulti1\for_testing""-v""E:\MPNP\MyMainFunction\test
// Multi.m"
//

#ifndef testMulti_h
#define testMulti_h 1

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
#ifndef LIB_testMulti_C_API 
#define LIB_testMulti_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_testMulti_C_API 
bool MW_CALL_CONV testMultiInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_testMulti_C_API 
bool MW_CALL_CONV testMultiInitialize(void);

extern LIB_testMulti_C_API 
void MW_CALL_CONV testMultiTerminate(void);

extern LIB_testMulti_C_API 
void MW_CALL_CONV testMultiPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_testMulti_C_API 
bool MW_CALL_CONV mlxTestMulti(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_testMulti
#define PUBLIC_testMulti_CPP_API __declspec(dllexport)
#else
#define PUBLIC_testMulti_CPP_API __declspec(dllimport)
#endif

#define LIB_testMulti_CPP_API PUBLIC_testMulti_CPP_API

#else

#if !defined(LIB_testMulti_CPP_API)
#if defined(LIB_testMulti_C_API)
#define LIB_testMulti_CPP_API LIB_testMulti_C_API
#else
#define LIB_testMulti_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_testMulti_CPP_API void MW_CALL_CONV testMulti(int nargout, mwArray& cc, mwArray& dd, const mwArray& aa, const mwArray& bb);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

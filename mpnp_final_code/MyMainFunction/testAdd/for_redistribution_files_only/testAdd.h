//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Fri Sep 22 09:35:14 2023
// Arguments:
// "-B""macro_default""-W""cpplib:testAdd,all,version=1.0""-T""link:lib""-d""E:\
// MPNP\MyMainFunction\testAdd\for_testing""-v""E:\MPNP\MyMainFunction\testAdd.m
// "
//

#ifndef testAdd_h
#define testAdd_h 1

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
#ifndef LIB_testAdd_C_API 
#define LIB_testAdd_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_testAdd_C_API 
bool MW_CALL_CONV testAddInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_testAdd_C_API 
bool MW_CALL_CONV testAddInitialize(void);

extern LIB_testAdd_C_API 
void MW_CALL_CONV testAddTerminate(void);

extern LIB_testAdd_C_API 
void MW_CALL_CONV testAddPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_testAdd_C_API 
bool MW_CALL_CONV mlxTestAdd(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_testAdd
#define PUBLIC_testAdd_CPP_API __declspec(dllexport)
#else
#define PUBLIC_testAdd_CPP_API __declspec(dllimport)
#endif

#define LIB_testAdd_CPP_API PUBLIC_testAdd_CPP_API

#else

#if !defined(LIB_testAdd_CPP_API)
#if defined(LIB_testAdd_C_API)
#define LIB_testAdd_CPP_API LIB_testAdd_C_API
#else
#define LIB_testAdd_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_testAdd_CPP_API void MW_CALL_CONV testAdd(int nargout, mwArray& z, const mwArray& x, const mwArray& y);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

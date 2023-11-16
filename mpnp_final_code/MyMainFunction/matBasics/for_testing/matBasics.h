//
// MATLAB Compiler: 8.5 (R2022b)
// Date: Thu Sep 21 21:01:53 2023
// Arguments:
// "-B""macro_default""-W""cpplib:matBasics,all,version=1.0""-T""link:lib""-d""E
// :\MPNP\MyMainFunction\matBasics\for_testing""-v""E:\MPNP\MyMainFunction\testD
// ll\matAbs.m""E:\MPNP\MyMainFunction\testDll\matAdd.m""E:\MPNP\MyMainFunction\
// testDll\matLoadDataFile.m"
//

#ifndef matBasics_h
#define matBasics_h 1

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
#ifndef LIB_matBasics_C_API 
#define LIB_matBasics_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_matBasics_C_API 
bool MW_CALL_CONV matBasicsInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_matBasics_C_API 
bool MW_CALL_CONV matBasicsInitialize(void);

extern LIB_matBasics_C_API 
void MW_CALL_CONV matBasicsTerminate(void);

extern LIB_matBasics_C_API 
void MW_CALL_CONV matBasicsPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_matBasics_C_API 
bool MW_CALL_CONV mlxMatAbs(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

extern LIB_matBasics_C_API 
bool MW_CALL_CONV mlxMatAdd(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

extern LIB_matBasics_C_API 
bool MW_CALL_CONV mlxMatLoadDataFile(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                     *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_matBasics
#define PUBLIC_matBasics_CPP_API __declspec(dllexport)
#else
#define PUBLIC_matBasics_CPP_API __declspec(dllimport)
#endif

#define LIB_matBasics_CPP_API PUBLIC_matBasics_CPP_API

#else

#if !defined(LIB_matBasics_CPP_API)
#if defined(LIB_matBasics_C_API)
#define LIB_matBasics_CPP_API LIB_matBasics_C_API
#else
#define LIB_matBasics_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_matBasics_CPP_API void MW_CALL_CONV matAbs(int nargout, mwArray& C, const mwArray& A);

extern LIB_matBasics_CPP_API void MW_CALL_CONV matAdd(int nargout, mwArray& C, const mwArray& A, const mwArray& B);

extern LIB_matBasics_CPP_API void MW_CALL_CONV matLoadDataFile(int nargout, mwArray& C, const mwArray& filename);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif

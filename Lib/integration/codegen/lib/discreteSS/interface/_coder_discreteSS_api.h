/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_discreteSS_api.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 28-Jan-2022 11:14:57
 */

#ifndef _CODER_DISCRETESS_API_H
#define _CODER_DISCRETESS_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T discreteSS(real_T x0, real_T u[2], real_T A, real_T B, real_T C,
                  real_T D);

void discreteSS_api(const mxArray *const prhs[6], const mxArray **plhs);

void discreteSS_atexit(void);

void discreteSS_initialize(void);

void discreteSS_terminate(void);

void discreteSS_xil_shutdown(void);

void discreteSS_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_discreteSS_api.h
 *
 * [EOF]
 */

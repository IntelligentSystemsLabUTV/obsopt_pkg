/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: discreteSS.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 28-Jan-2022 11:14:57
 */

/* Include Files */
#include "discreteSS.h"

/* Function Definitions */
/*
 * Arguments    : double x0
 *                const double u[2]
 *                double A
 *                double B
 *                double C
 *                double D
 * Return Type  : double
 */
double discreteSS(double x0, const double u[2], double A, double B, double C,
                  double D)
{
  /*  */
  return C * (A * x0 + B * u[0]) + D * u[1];
}

/*
 * File trailer for discreteSS.c
 *
 * [EOF]
 */

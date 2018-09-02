/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:14 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_BaseRotX.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t2977;
  double t3002;
  double t2534;
  double t3061;
  double t3093;
  double t3158;
  t2977 = Cos(var1[4]);
  t3002 = Sin(var1[3]);
  t2534 = Cos(var1[3]);
  t3061 = Sin(var1[4]);
  t3093 = Cos(var1[5]);
  t3158 = Sin(var1[5]);

  p_output1(0)=t2534*t2977;
  p_output1(1)=t2977*t3002;
  p_output1(2)=-1.*t3061;
  p_output1(3)=-1.*t3002*t3093 + t2534*t3061*t3158;
  p_output1(4)=t2534*t3093 + t3002*t3061*t3158;
  p_output1(5)=t2977*t3158;
  p_output1(6)=t2534*t3061*t3093 + t3002*t3158;
  p_output1(7)=t3002*t3061*t3093 - 1.*t2534*t3158;
  p_output1(8)=t2977*t3093;
}


       
void R_BaseRotX(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
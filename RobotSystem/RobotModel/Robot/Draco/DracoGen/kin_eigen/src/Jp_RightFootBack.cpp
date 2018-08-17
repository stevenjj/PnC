/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:13 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RightFootBack.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t258;
  double t18;
  double t69;
  double t81;
  double t215;
  double t451;
  double t406;
  double t424;
  double t453;
  double t277;
  double t299;
  double t305;
  double t383;
  double t13;
  double t621;
  double t662;
  double t710;
  double t437;
  double t468;
  double t481;
  double t813;
  double t821;
  double t957;
  double t1037;
  double t1057;
  double t1093;
  double t1158;
  double t1313;
  double t1412;
  double t1470;
  double t1701;
  double t1920;
  double t1978;
  double t2050;
  double t2064;
  double t2163;
  double t2238;
  double t2307;
  double t2330;
  double t2374;
  double t2448;
  double t2450;
  double t2461;
  double t2505;
  double t2574;
  double t2581;
  double t2601;
  double t2624;
  double t2653;
  double t2673;
  double t2708;
  double t2759;
  double t2782;
  double t133;
  double t231;
  double t232;
  double t346;
  double t397;
  double t404;
  double t527;
  double t544;
  double t589;
  double t726;
  double t760;
  double t792;
  double t3199;
  double t3201;
  double t3260;
  double t1151;
  double t1188;
  double t1263;
  double t3076;
  double t3094;
  double t3124;
  double t3295;
  double t3297;
  double t3310;
  double t1595;
  double t1621;
  double t1687;
  double t2191;
  double t2286;
  double t2305;
  double t3333;
  double t3341;
  double t3345;
  double t3412;
  double t3436;
  double t3462;
  double t2407;
  double t2411;
  double t2413;
  double t2582;
  double t2610;
  double t2621;
  double t3484;
  double t3578;
  double t3580;
  double t3596;
  double t3632;
  double t3659;
  double t2693;
  double t2694;
  double t2701;
  double t3674;
  double t3697;
  double t3732;
  double t3788;
  double t3820;
  double t3838;
  double t4330;
  double t4367;
  double t4377;
  double t4557;
  double t4576;
  double t4579;
  double t4640;
  double t4643;
  double t4649;
  double t4673;
  double t4679;
  double t4694;
  double t4702;
  double t4720;
  double t4726;
  double t4757;
  double t4768;
  double t4774;
  double t4807;
  double t4815;
  double t4816;
  double t5076;
  double t5089;
  double t5092;
  double t5097;
  double t5125;
  double t5127;
  double t5189;
  double t5216;
  double t5220;
  double t5224;
  double t5230;
  double t5246;
  double t5272;
  double t5297;
  double t5329;
  double t5365;
  double t5410;
  double t5414;
  double t5456;
  double t5486;
  double t5489;
  double t5656;
  double t5666;
  double t5672;
  double t5680;
  double t5701;
  double t5707;
  double t5736;
  double t5745;
  double t5764;
  double t5775;
  double t5776;
  double t5778;
  double t5789;
  double t5802;
  double t5806;
  double t5822;
  double t5839;
  double t5844;
  double t5855;
  double t5865;
  double t5873;
  double t5955;
  double t5969;
  double t5973;
  double t5990;
  double t5994;
  double t6000;
  double t6071;
  double t6082;
  double t6090;
  double t6135;
  double t6140;
  double t6145;
  double t6173;
  double t6196;
  double t6204;
  double t6251;
  double t6269;
  double t6285;
  double t6342;
  double t6370;
  double t6379;
  double t6418;
  double t6420;
  double t6422;
  double t6476;
  double t6478;
  double t6482;
  double t6493;
  double t6502;
  double t6528;
  double t6532;
  double t6545;
  double t6569;
  double t6573;
  double t6576;
  double t6583;
  double t6685;
  double t6691;
  double t6694;
  double t6732;
  double t6739;
  double t6744;
  double t6753;
  double t6758;
  double t6768;
  double t6771;
  double t6773;
  double t6779;
  double t6788;
  double t6793;
  double t6794;
  double t6869;
  double t6870;
  double t6881;
  double t6896;
  double t6899;
  double t6915;
  double t6926;
  double t6931;
  double t6938;
  double t6953;
  double t6959;
  double t6963;
  double t6978;
  double t6991;
  double t6999;
  double t6843;
  double t6848;
  double t6853;
  double t6860;
  double t6861;
  double t7061;
  double t7070;
  double t7075;
  double t7117;
  double t7137;
  double t7093;
  double t7109;
  double t7113;
  double t7156;
  double t7157;
  double t7159;
  double t7170;
  double t7174;
  double t7177;
  double t7180;
  double t7195;
  double t7208;
  double t7217;
  double t7223;
  double t7227;
  double t7349;
  double t7366;
  double t7371;
  double t7324;
  double t7327;
  double t7339;
  double t7393;
  double t7394;
  double t7403;
  double t7411;
  double t7419;
  double t7424;
  double t7439;
  double t7441;
  double t7460;
  double t7468;
  double t7475;
  double t7483;
  double t7654;
  double t7657;
  double t7662;
  double t7710;
  double t7720;
  double t7727;
  double t7735;
  double t7739;
  double t7743;
  double t7571;
  double t7583;
  double t7610;
  double t7611;
  double t7622;
  double t7889;
  double t7907;
  double t7915;
  double t7958;
  double t7961;
  double t7972;
  double t7976;
  double t7977;
  double t7989;
  double t8111;
  double t8112;
  double t8141;
  double t8183;
  double t8193;
  double t8195;
  double t8202;
  double t8228;
  double t8248;
  double t8407;
  double t8411;
  double t8412;
  double t8465;
  double t8474;
  double t8494;
  double t8498;
  double t8505;
  double t8340;
  double t8346;
  double t8364;
  double t8365;
  double t8381;
  double t7926;
  double t7942;
  double t7953;
  double t8573;
  double t8575;
  double t8576;
  double t8604;
  double t8608;
  double t8611;
  double t8618;
  double t8623;
  double t8636;
  double t8655;
  double t8660;
  double t8661;
  double t8668;
  double t8670;
  double t8676;
  double t8166;
  double t8169;
  double t8170;
  double t8760;
  double t8767;
  double t8783;
  double t8813;
  double t8815;
  double t8821;
  double t8828;
  double t8848;
  double t8851;
  double t8860;
  double t8911;
  double t8912;
  double t8930;
  double t8933;
  double t8944;
  double t9073;
  double t9079;
  double t4047;
  double t9000;
  double t9016;
  double t9021;
  double t9037;
  double t9038;
  double t9177;
  double t9181;
  double t9183;
  double t9194;
  double t9196;
  double t9200;
  double t9205;
  double t9273;
  double t9276;
  double t9287;
  double t9302;
  double t9325;
  double t9337;
  double t9351;
  double t9118;
  double t4085;
  double t4106;
  double t9469;
  double t9472;
  double t9482;
  double t9509;
  double t9524;
  double t9208;
  double t9566;
  double t9567;
  double t9569;
  double t9233;
  double t9370;
  double t9652;
  double t9653;
  double t9657;
  double t9414;
  t258 = Sin(var1[3]);
  t18 = Cos(var1[11]);
  t69 = -1.*t18;
  t81 = 1. + t69;
  t215 = Sin(var1[11]);
  t451 = Cos(var1[3]);
  t406 = Cos(var1[5]);
  t424 = Sin(var1[4]);
  t453 = Sin(var1[5]);
  t277 = Cos(var1[12]);
  t299 = -1.*t277;
  t305 = 1. + t299;
  t383 = Sin(var1[12]);
  t13 = Cos(var1[4]);
  t621 = -1.*t451*t406;
  t662 = -1.*t258*t424*t453;
  t710 = t621 + t662;
  t437 = -1.*t406*t258*t424;
  t468 = t451*t453;
  t481 = t437 + t468;
  t813 = t13*t215*t258;
  t821 = t18*t710;
  t957 = t813 + t821;
  t1037 = Cos(var1[13]);
  t1057 = -1.*t1037;
  t1093 = 1. + t1057;
  t1158 = Sin(var1[13]);
  t1313 = -1.*t18*t13*t258;
  t1412 = t215*t710;
  t1470 = t1313 + t1412;
  t1701 = t277*t481;
  t1920 = -1.*t383*t957;
  t1978 = t1701 + t1920;
  t2050 = Cos(var1[14]);
  t2064 = -1.*t2050;
  t2163 = 1. + t2064;
  t2238 = Sin(var1[14]);
  t2307 = t1158*t1470;
  t2330 = t1037*t1978;
  t2374 = t2307 + t2330;
  t2448 = t1037*t1470;
  t2450 = -1.*t1158*t1978;
  t2461 = t2448 + t2450;
  t2505 = Cos(var1[15]);
  t2574 = -1.*t2505;
  t2581 = 1. + t2574;
  t2601 = Sin(var1[15]);
  t2624 = -1.*t2238*t2374;
  t2653 = t2050*t2461;
  t2673 = t2624 + t2653;
  t2708 = t2050*t2374;
  t2759 = t2238*t2461;
  t2782 = t2708 + t2759;
  t133 = -0.022225*t81;
  t231 = -0.086996*t215;
  t232 = 0. + t133 + t231;
  t346 = -0.31508*t305;
  t397 = 0.156996*t383;
  t404 = 0. + t346 + t397;
  t527 = -0.086996*t81;
  t544 = 0.022225*t215;
  t589 = 0. + t527 + t544;
  t726 = -0.156996*t305;
  t760 = -0.31508*t383;
  t792 = 0. + t726 + t760;
  t3199 = -1.*t406*t258;
  t3201 = t451*t424*t453;
  t3260 = t3199 + t3201;
  t1151 = -0.022225*t1093;
  t1188 = 0.38008*t1158;
  t1263 = 0. + t1151 + t1188;
  t3076 = t451*t406*t424;
  t3094 = t258*t453;
  t3124 = t3076 + t3094;
  t3295 = -1.*t451*t13*t215;
  t3297 = t18*t3260;
  t3310 = t3295 + t3297;
  t1595 = -0.38008*t1093;
  t1621 = -0.022225*t1158;
  t1687 = 0. + t1595 + t1621;
  t2191 = -0.86008*t2163;
  t2286 = -0.022225*t2238;
  t2305 = 0. + t2191 + t2286;
  t3333 = t18*t451*t13;
  t3341 = t215*t3260;
  t3345 = t3333 + t3341;
  t3412 = t277*t3124;
  t3436 = -1.*t383*t3310;
  t3462 = t3412 + t3436;
  t2407 = -0.022225*t2163;
  t2411 = 0.86008*t2238;
  t2413 = 0. + t2407 + t2411;
  t2582 = -0.021147*t2581;
  t2610 = 1.34008*t2601;
  t2621 = 0. + t2582 + t2610;
  t3484 = t1158*t3345;
  t3578 = t1037*t3462;
  t3580 = t3484 + t3578;
  t3596 = t1037*t3345;
  t3632 = -1.*t1158*t3462;
  t3659 = t3596 + t3632;
  t2693 = -1.34008*t2581;
  t2694 = -0.021147*t2601;
  t2701 = 0. + t2693 + t2694;
  t3674 = -1.*t2238*t3580;
  t3697 = t2050*t3659;
  t3732 = t3674 + t3697;
  t3788 = t2050*t3580;
  t3820 = t2238*t3659;
  t3838 = t3788 + t3820;
  t4330 = t451*t215*t424;
  t4367 = t18*t451*t13*t453;
  t4377 = t4330 + t4367;
  t4557 = -1.*t18*t451*t424;
  t4576 = t451*t13*t215*t453;
  t4579 = t4557 + t4576;
  t4640 = t277*t451*t13*t406;
  t4643 = -1.*t383*t4377;
  t4649 = t4640 + t4643;
  t4673 = t1158*t4579;
  t4679 = t1037*t4649;
  t4694 = t4673 + t4679;
  t4702 = t1037*t4579;
  t4720 = -1.*t1158*t4649;
  t4726 = t4702 + t4720;
  t4757 = -1.*t2238*t4694;
  t4768 = t2050*t4726;
  t4774 = t4757 + t4768;
  t4807 = t2050*t4694;
  t4815 = t2238*t4726;
  t4816 = t4807 + t4815;
  t5076 = t215*t258*t424;
  t5089 = t18*t13*t258*t453;
  t5092 = t5076 + t5089;
  t5097 = -1.*t18*t258*t424;
  t5125 = t13*t215*t258*t453;
  t5127 = t5097 + t5125;
  t5189 = t277*t13*t406*t258;
  t5216 = -1.*t383*t5092;
  t5220 = t5189 + t5216;
  t5224 = t1158*t5127;
  t5230 = t1037*t5220;
  t5246 = t5224 + t5230;
  t5272 = t1037*t5127;
  t5297 = -1.*t1158*t5220;
  t5329 = t5272 + t5297;
  t5365 = -1.*t2238*t5246;
  t5410 = t2050*t5329;
  t5414 = t5365 + t5410;
  t5456 = t2050*t5246;
  t5486 = t2238*t5329;
  t5489 = t5456 + t5486;
  t5656 = t13*t215;
  t5666 = -1.*t18*t424*t453;
  t5672 = t5656 + t5666;
  t5680 = -1.*t18*t13;
  t5701 = -1.*t215*t424*t453;
  t5707 = t5680 + t5701;
  t5736 = -1.*t277*t406*t424;
  t5745 = -1.*t383*t5672;
  t5764 = t5736 + t5745;
  t5775 = t1158*t5707;
  t5776 = t1037*t5764;
  t5778 = t5775 + t5776;
  t5789 = t1037*t5707;
  t5802 = -1.*t1158*t5764;
  t5806 = t5789 + t5802;
  t5822 = -1.*t2238*t5778;
  t5839 = t2050*t5806;
  t5844 = t5822 + t5839;
  t5855 = t2050*t5778;
  t5865 = t2238*t5806;
  t5873 = t5855 + t5865;
  t5955 = t406*t258;
  t5969 = -1.*t451*t424*t453;
  t5973 = t5955 + t5969;
  t5990 = -1.*t18*t383*t3124;
  t5994 = t277*t5973;
  t6000 = t5990 + t5994;
  t6071 = t215*t1158*t3124;
  t6082 = t1037*t6000;
  t6090 = t6071 + t6082;
  t6135 = t1037*t215*t3124;
  t6140 = -1.*t1158*t6000;
  t6145 = t6135 + t6140;
  t6173 = -1.*t2238*t6090;
  t6196 = t2050*t6145;
  t6204 = t6173 + t6196;
  t6251 = t2050*t6090;
  t6269 = t2238*t6145;
  t6285 = t6251 + t6269;
  t6342 = t406*t258*t424;
  t6370 = -1.*t451*t453;
  t6379 = t6342 + t6370;
  t6418 = -1.*t18*t383*t6379;
  t6420 = t277*t710;
  t6422 = t6418 + t6420;
  t6476 = t215*t1158*t6379;
  t6478 = t1037*t6422;
  t6482 = t6476 + t6478;
  t6493 = t1037*t215*t6379;
  t6502 = -1.*t1158*t6422;
  t6528 = t6493 + t6502;
  t6532 = -1.*t2238*t6482;
  t6545 = t2050*t6528;
  t6569 = t6532 + t6545;
  t6573 = t2050*t6482;
  t6576 = t2238*t6528;
  t6583 = t6573 + t6576;
  t6685 = -1.*t18*t13*t406*t383;
  t6691 = -1.*t277*t13*t453;
  t6694 = t6685 + t6691;
  t6732 = t13*t406*t215*t1158;
  t6739 = t1037*t6694;
  t6744 = t6732 + t6739;
  t6753 = t1037*t13*t406*t215;
  t6758 = -1.*t1158*t6694;
  t6768 = t6753 + t6758;
  t6771 = -1.*t2238*t6744;
  t6773 = t2050*t6768;
  t6779 = t6771 + t6773;
  t6788 = t2050*t6744;
  t6793 = t2238*t6768;
  t6794 = t6788 + t6793;
  t6869 = -1.*t18*t451*t13;
  t6870 = -1.*t215*t3260;
  t6881 = t6869 + t6870;
  t6896 = t1158*t3310;
  t6899 = -1.*t1037*t383*t6881;
  t6915 = t6896 + t6899;
  t6926 = t1037*t3310;
  t6931 = t383*t1158*t6881;
  t6938 = t6926 + t6931;
  t6953 = -1.*t2238*t6915;
  t6959 = t2050*t6938;
  t6963 = t6953 + t6959;
  t6978 = t2050*t6915;
  t6991 = t2238*t6938;
  t6999 = t6978 + t6991;
  t6843 = -0.086996*t18;
  t6848 = -0.022225*t215;
  t6853 = t6843 + t6848;
  t6860 = 0.022225*t18;
  t6861 = t6860 + t231;
  t7061 = t451*t406;
  t7070 = t258*t424*t453;
  t7075 = t7061 + t7070;
  t7117 = -1.*t215*t7075;
  t7137 = t1313 + t7117;
  t7093 = -1.*t13*t215*t258;
  t7109 = t18*t7075;
  t7113 = t7093 + t7109;
  t7156 = t1158*t7113;
  t7157 = -1.*t1037*t383*t7137;
  t7159 = t7156 + t7157;
  t7170 = t1037*t7113;
  t7174 = t383*t1158*t7137;
  t7177 = t7170 + t7174;
  t7180 = -1.*t2238*t7159;
  t7195 = t2050*t7177;
  t7208 = t7180 + t7195;
  t7217 = t2050*t7159;
  t7223 = t2238*t7177;
  t7227 = t7217 + t7223;
  t7349 = t18*t424;
  t7366 = -1.*t13*t215*t453;
  t7371 = t7349 + t7366;
  t7324 = t215*t424;
  t7327 = t18*t13*t453;
  t7339 = t7324 + t7327;
  t7393 = t1158*t7339;
  t7394 = -1.*t1037*t383*t7371;
  t7403 = t7393 + t7394;
  t7411 = t1037*t7339;
  t7419 = t383*t1158*t7371;
  t7424 = t7411 + t7419;
  t7439 = -1.*t2238*t7403;
  t7441 = t2050*t7424;
  t7460 = t7439 + t7441;
  t7468 = t2050*t7403;
  t7475 = t2238*t7424;
  t7483 = t7468 + t7475;
  t7654 = -1.*t383*t3124;
  t7657 = -1.*t277*t3310;
  t7662 = t7654 + t7657;
  t7710 = -1.*t2050*t1158*t7662;
  t7720 = -1.*t1037*t2238*t7662;
  t7727 = t7710 + t7720;
  t7735 = t1037*t2050*t7662;
  t7739 = -1.*t1158*t2238*t7662;
  t7743 = t7735 + t7739;
  t7571 = 0.156996*t277;
  t7583 = t7571 + t760;
  t7610 = -0.31508*t277;
  t7611 = -0.156996*t383;
  t7622 = t7610 + t7611;
  t7889 = -1.*t383*t6379;
  t7907 = -1.*t277*t7113;
  t7915 = t7889 + t7907;
  t7958 = -1.*t2050*t1158*t7915;
  t7961 = -1.*t1037*t2238*t7915;
  t7972 = t7958 + t7961;
  t7976 = t1037*t2050*t7915;
  t7977 = -1.*t1158*t2238*t7915;
  t7989 = t7976 + t7977;
  t8111 = -1.*t13*t406*t383;
  t8112 = -1.*t277*t7339;
  t8141 = t8111 + t8112;
  t8183 = -1.*t2050*t1158*t8141;
  t8193 = -1.*t1037*t2238*t8141;
  t8195 = t8183 + t8193;
  t8202 = t1037*t2050*t8141;
  t8228 = -1.*t1158*t2238*t8141;
  t8248 = t8202 + t8228;
  t8407 = -1.*t1158*t3345;
  t8411 = -1.*t1037*t3462;
  t8412 = t8407 + t8411;
  t8465 = t2238*t8412;
  t8474 = t8465 + t3697;
  t8494 = t2050*t8412;
  t8498 = -1.*t2238*t3659;
  t8505 = t8494 + t8498;
  t8340 = 0.38008*t1037;
  t8346 = t8340 + t1621;
  t8364 = -0.022225*t1037;
  t8365 = -0.38008*t1158;
  t8381 = t8364 + t8365;
  t7926 = t277*t6379;
  t7942 = -1.*t383*t7113;
  t7953 = t7926 + t7942;
  t8573 = t18*t13*t258;
  t8575 = t215*t7075;
  t8576 = t8573 + t8575;
  t8604 = -1.*t1158*t8576;
  t8608 = -1.*t1037*t7953;
  t8611 = t8604 + t8608;
  t8618 = t1037*t8576;
  t8623 = -1.*t1158*t7953;
  t8636 = t8618 + t8623;
  t8655 = t2238*t8611;
  t8660 = t2050*t8636;
  t8661 = t8655 + t8660;
  t8668 = t2050*t8611;
  t8670 = -1.*t2238*t8636;
  t8676 = t8668 + t8670;
  t8166 = t277*t13*t406;
  t8169 = -1.*t383*t7339;
  t8170 = t8166 + t8169;
  t8760 = -1.*t18*t424;
  t8767 = t13*t215*t453;
  t8783 = t8760 + t8767;
  t8813 = -1.*t1158*t8783;
  t8815 = -1.*t1037*t8170;
  t8821 = t8813 + t8815;
  t8828 = t1037*t8783;
  t8848 = -1.*t1158*t8170;
  t8851 = t8828 + t8848;
  t8860 = t2238*t8821;
  t8911 = t2050*t8851;
  t8912 = t8860 + t8911;
  t8930 = t2050*t8821;
  t8933 = -1.*t2238*t8851;
  t8944 = t8930 + t8933;
  t9073 = -1.*t2050*t3580;
  t9079 = t9073 + t8498;
  t4047 = t2505*t3732;
  t9000 = -0.022225*t2050;
  t9016 = -0.86008*t2238;
  t9021 = t9000 + t9016;
  t9037 = 0.86008*t2050;
  t9038 = t9037 + t2286;
  t9177 = t1158*t8576;
  t9181 = t1037*t7953;
  t9183 = t9177 + t9181;
  t9194 = -1.*t2238*t9183;
  t9196 = t9194 + t8660;
  t9200 = -1.*t2050*t9183;
  t9205 = t9200 + t8670;
  t9273 = t1158*t8783;
  t9276 = t1037*t8170;
  t9287 = t9273 + t9276;
  t9302 = -1.*t2238*t9287;
  t9325 = t9302 + t8911;
  t9337 = -1.*t2050*t9287;
  t9351 = t9337 + t8933;
  t9118 = -1.*t2601*t3732;
  t4085 = -1.*t2601*t3838;
  t4106 = t4047 + t4085;
  t9469 = 1.34008*t2505;
  t9472 = t9469 + t2694;
  t9482 = -0.021147*t2505;
  t9509 = -1.34008*t2601;
  t9524 = t9482 + t9509;
  t9208 = -1.*t2601*t9196;
  t9566 = t2050*t9183;
  t9567 = t2238*t8636;
  t9569 = t9566 + t9567;
  t9233 = t2505*t9196;
  t9370 = -1.*t2601*t9325;
  t9652 = t2050*t9287;
  t9653 = t2238*t8851;
  t9657 = t9652 + t9653;
  t9414 = t2505*t9325;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1263*t1470 + t1687*t1978 + t2305*t2374 + t2413*t2461 - 1.*t13*t232*t258 + t2621*t2673 + t2701*t2782 - 1.400132*(t2601*t2673 + t2505*t2782) + 0.043805*(t2505*t2673 - 1.*t2601*t2782) + t404*t481 + t589*t710 + t792*t957 - 0.166996*(t383*t481 + t277*t957);
  p_output1(10)=t1263*t3345 + t1687*t3462 + t2305*t3580 + t2413*t3659 + t2621*t3732 - 0.166996*(t277*t3310 + t3124*t383) + t2701*t3838 - 1.400132*(t2601*t3732 + t2505*t3838) + t3124*t404 + 0.043805*t4106 + t13*t232*t451 + t3260*t589 + t3310*t792;
  p_output1(11)=0;
  p_output1(12)=t13*t404*t406*t451 - 1.*t232*t424*t451 - 0.166996*(t277*t4377 + t13*t383*t406*t451) + t1263*t4579 + t1687*t4649 + t2305*t4694 + t2413*t4726 + t2621*t4774 + t2701*t4816 - 1.400132*(t2601*t4774 + t2505*t4816) + 0.043805*(t2505*t4774 - 1.*t2601*t4816) + t13*t451*t453*t589 + t4377*t792;
  p_output1(13)=t13*t258*t404*t406 - 1.*t232*t258*t424 - 0.166996*(t13*t258*t383*t406 + t277*t5092) + t1263*t5127 + t1687*t5220 + t2305*t5246 + t2413*t5329 + t2621*t5414 + t2701*t5489 - 1.400132*(t2601*t5414 + t2505*t5489) + 0.043805*(t2505*t5414 - 1.*t2601*t5489) + t13*t258*t453*t589 + t5092*t792;
  p_output1(14)=-1.*t13*t232 - 1.*t404*t406*t424 - 0.166996*(-1.*t383*t406*t424 + t277*t5672) + t1263*t5707 + t1687*t5764 + t2305*t5778 + t2413*t5806 + t2621*t5844 + t2701*t5873 - 1.400132*(t2601*t5844 + t2505*t5873) + 0.043805*(t2505*t5844 - 1.*t2601*t5873) - 1.*t424*t453*t589 + t5672*t792;
  p_output1(15)=t1263*t215*t3124 + t3124*t589 + t404*t5973 - 0.166996*(t18*t277*t3124 + t383*t5973) + t1687*t6000 + t2305*t6090 + t2413*t6145 + t2621*t6204 + t2701*t6285 - 1.400132*(t2601*t6204 + t2505*t6285) + 0.043805*(t2505*t6204 - 1.*t2601*t6285) + t18*t3124*t792;
  p_output1(16)=t1263*t215*t6379 + t589*t6379 + t1687*t6422 + t2305*t6482 + t2413*t6528 + t2621*t6569 + t2701*t6583 - 1.400132*(t2601*t6569 + t2505*t6583) + 0.043805*(t2505*t6569 - 1.*t2601*t6583) + t404*t710 - 0.166996*(t18*t277*t6379 + t383*t710) + t18*t6379*t792;
  p_output1(17)=t1263*t13*t215*t406 - 1.*t13*t404*t453 - 0.166996*(t13*t18*t277*t406 - 1.*t13*t383*t453) + t13*t406*t589 + t1687*t6694 + t2305*t6744 + t2413*t6768 + t2621*t6779 + t2701*t6794 - 1.400132*(t2601*t6779 + t2505*t6794) + 0.043805*(t2505*t6779 - 1.*t2601*t6794) + t13*t18*t406*t792;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t1263*t3310 + t13*t451*t6853 + t3260*t6861 - 0.166996*t277*t6881 - 1.*t1687*t383*t6881 + t2305*t6915 + t2413*t6938 + t2621*t6963 + t2701*t6999 - 1.400132*(t2601*t6963 + t2505*t6999) + 0.043805*(t2505*t6963 - 1.*t2601*t6999) + t6881*t792;
  p_output1(34)=t13*t258*t6853 + t6861*t7075 + t1263*t7113 - 0.166996*t277*t7137 - 1.*t1687*t383*t7137 + t2305*t7159 + t2413*t7177 + t2621*t7208 + t2701*t7227 - 1.400132*(t2601*t7208 + t2505*t7227) + 0.043805*(t2505*t7208 - 1.*t2601*t7227) + t7137*t792;
  p_output1(35)=-1.*t424*t6853 + t13*t453*t6861 + t1263*t7339 - 0.166996*t277*t7371 - 1.*t1687*t383*t7371 + t2305*t7403 + t2413*t7424 + t2621*t7460 + t2701*t7483 - 1.400132*(t2601*t7460 + t2505*t7483) + 0.043805*(t2505*t7460 - 1.*t2601*t7483) + t7371*t792;
  p_output1(36)=-0.166996*t3462 + t3124*t7583 + t3310*t7622 + t1687*t7662 + t1037*t2305*t7662 - 1.*t1158*t2413*t7662 + t2621*t7727 + t2701*t7743 - 1.400132*(t2601*t7727 + t2505*t7743) + 0.043805*(t2505*t7727 - 1.*t2601*t7743);
  p_output1(37)=t6379*t7583 + t7113*t7622 + t1687*t7915 + t1037*t2305*t7915 - 1.*t1158*t2413*t7915 - 0.166996*t7953 + t2621*t7972 + t2701*t7989 - 1.400132*(t2601*t7972 + t2505*t7989) + 0.043805*(t2505*t7972 - 1.*t2601*t7989);
  p_output1(38)=t13*t406*t7583 + t7339*t7622 + t1687*t8141 + t1037*t2305*t8141 - 1.*t1158*t2413*t8141 - 0.166996*t8170 + t2621*t8195 + t2701*t8248 - 1.400132*(t2601*t8195 + t2505*t8248) + 0.043805*(t2505*t8195 - 1.*t2601*t8248);
  p_output1(39)=t2305*t3659 + t3345*t8346 + t3462*t8381 + t2413*t8412 + t2701*t8474 + t2621*t8505 + 0.043805*(-1.*t2601*t8474 + t2505*t8505) - 1.400132*(t2505*t8474 + t2601*t8505);
  p_output1(40)=t7953*t8381 + t8346*t8576 + t2413*t8611 + t2305*t8636 + t2701*t8661 + t2621*t8676 + 0.043805*(-1.*t2601*t8661 + t2505*t8676) - 1.400132*(t2505*t8661 + t2601*t8676);
  p_output1(41)=t8170*t8381 + t8346*t8783 + t2413*t8821 + t2305*t8851 + t2701*t8912 + t2621*t8944 + 0.043805*(-1.*t2601*t8912 + t2505*t8944) - 1.400132*(t2505*t8912 + t2601*t8944);
  p_output1(42)=t2701*t3732 + t3580*t9021 + t3659*t9038 + t2621*t9079 - 1.400132*(t4047 + t2601*t9079) + 0.043805*(t2505*t9079 + t9118);
  p_output1(43)=t8636*t9038 + t9021*t9183 + t2701*t9196 + t2621*t9205 + 0.043805*(t2505*t9205 + t9208) - 1.400132*(t2601*t9205 + t9233);
  p_output1(44)=t8851*t9038 + t9021*t9287 + t2701*t9325 + t2621*t9351 + 0.043805*(t2505*t9351 + t9370) - 1.400132*(t2601*t9351 + t9414);
  p_output1(45)=-1.400132*t4106 + 0.043805*(-1.*t2505*t3838 + t9118) + t3732*t9472 + t3838*t9524;
  p_output1(46)=t9196*t9472 + t9524*t9569 + 0.043805*(t9208 - 1.*t2505*t9569) - 1.400132*(t9233 - 1.*t2601*t9569);
  p_output1(47)=t9325*t9472 + t9524*t9657 + 0.043805*(t9370 - 1.*t2505*t9657) - 1.400132*(t9414 - 1.*t2601*t9657);
}


       
void Jp_RightFootBack(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

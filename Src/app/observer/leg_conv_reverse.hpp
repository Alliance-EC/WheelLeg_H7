#pragma once
//
// File: leg_conv_reverse.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 30-Mar-2024 23:31:54
//

// Include Files
#include <arm_math.h>
#include <cmath>
namespace app::observer {
// STM32优化版
//  Function Definitions
//
//  LEG_CONV_REVERSE
//      T_r = LEG_CONV_REVERSE(T1,T2,PHI1,PHI4)
//
//      T2    T1
//    _ _ ______
//    4* /    1*\
//       \      /
//        \    /
//
//  Arguments    : float T1
//                 float T2
//                 float phi1
//                 float phi4
//                 float T_r[2]  [F; Tp]
//  Return Type  : void
//
void leg_conv_reverse(double T1, double T2, double phi1, double phi4, double T_r[2]) {
    double t2;
    double t21;
    double t22;
    double t23;
    double t3;
    double t4;
    double t41;
    double t45;
    double t5;
    double t50_im_tmp;
    double t7;
    double t8;
    double t9;
    //     This function was generated by the Symbolic Math Toolbox version 23.2.
    //     2025-02-28 07:24:22
    t2  = std::cos(phi1);
    t3  = std::cos(phi4);
    t4  = std::sin(phi1);
    t5  = std::sin(phi4);
    t7  = t2 * 0.1525;
    t8  = t5 * 0.11;
    t9  = t4 * 0.1525;
    t45 = t8 - t9;
    t21 = t45 * t45;
    t22 = (t3 * 0.11 - t7) + 0.17;
    t23 = t22 * t22;
    t45 = (t21 + t23) - 0.0369;
    t5  = std::atan(
             1.0 / ((((t3 * 0.0352 - t2 * 0.0488) + t21) + t23) + 0.0175)
             * ((t5 * 0.0352 - t4 * 0.0488) + std::sqrt((t21 * 0.1024 + t23 * 0.1024) - t45 * t45)))
       * 2.0;
    t2         = std::cos(t5);
    t3         = std::sin(t5);
    t45        = t2 * 0.26;
    t41        = std::sin(phi1 - t5);
    t50_im_tmp = t4 * 0.1525 + t3 * 0.26;
    t21        = std::atan2(t50_im_tmp, (((t7 + t4 * 0.0) + t45) + t3 * 0.0) - 0.085);
    t3         = std::atan(((t9 - t8) + t3 * 0.16) * (-1.0 / (t22 - t2 * 0.16)));
    t23        = t5 - t21;
    t21        = -t21 + t3;
    t3         = std::sin(phi4 - t3);
    t45        = (t7 + t45) - 0.085;
    t5         = 1.0 / t41 * (1.0 / t3);
    t2         = T2 * t41;
    t3 *= T1;
    T_r[0] = t5 * (t2 * std::cos(t21) * 0.1525 + t3 * std::cos(t23) * 0.11) * -59.612518628912071;
    T_r[1] = t5 * std::sqrt(t50_im_tmp * t50_im_tmp + t45 * t45)
           * (t2 * std::sin(t21) * 0.1525 + t3 * std::sin(t23) * 0.11) * -59.612518628912071;
}
//
// File trailer for leg_conv_reverse.cpp
//
// [EOF]
//
} // namespace app::observer
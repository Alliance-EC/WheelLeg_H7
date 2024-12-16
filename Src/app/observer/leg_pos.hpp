#pragma once
//
// File: leg_pos.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 30-Mar-2024 23:08:59
//

// Include Files
#include <arm_math.h>
#include <cmath>
namespace app::observer {
// Function Definitions
//
// LEG_POS
//     POS = LEG_POS(PHI1,PHI4)
//
//  _ _ ______
//   4*/    1*\
//     \      /
//      \    /
//
// Arguments    : double phi1 均为与后侧水平方向
//                double phi4
//                double pos[2] 长度和角度 [l0; theta]
// Return Type  : void
//
void leg_pos(double phi1, double phi4, double pos[2]) {
    double t19;
    double t2;
    double t21;
    double t24;
    double t3;
    double t4;
    double t5;
    double t6;
    double t8;
    //     This function was generated by the Symbolic Math Toolbox version 23.2.
    //     2024-07-15 23:13:01
    t2  = std::cos(phi1);
    t3  = std::cos(phi4);
    t4  = std::sin(phi1);
    t5  = std::sin(phi4);
    t6  = t2 * 0.148;
    t8  = t4 * 0.148;
    t21 = t8 - t5 * 0.148;
    t24 = (t3 * 0.148 - t6) + 0.14;
    t19 = t21 * t21;
    t21 = t24 * t24;
    t24 = t19 + t21;
    t21 = std::atan(
              1.0 / (((t3 * 0.07844 - t2 * 0.07844) + t24) + 0.0742)
              * ((t5 * 0.07844 - t4 * 0.07844)
                 + std::sqrt((t19 * 0.2809 + t21 * 0.2809) - t24 * t24)))
        * 2.0;
    t24    = t8 + std::sin(t21) * 0.265;
    t21    = (t6 + std::cos(t21) * 0.265) - 0.07;
    pos[0] = std::sqrt(t24 * t24 + t21 * t21);
    pos[1] = 1.5707963267948966 - std::atan2(t24, t21);
}

//
// File trailer for leg_pos.cpp
//
// [EOF]
//
} // namespace app
#pragma once

#include <Eigen/Dense>
#include <cstdint>

namespace app {
constexpr double m_b     = 16.98;  // 身体质量
constexpr double eta_l   = 0.2945; // 质心位置系数
constexpr double m_l     = 0.86;
constexpr double m_w     = 0.2;
constexpr double R_l     = 0.22;   // 半轮距
constexpr double gravity = 9.80665;
constexpr double Rw      = 0.07;   // 轮子半径

constexpr double dt = 0.001;       // 执行周期

// clang-format off
const Eigen::Matrix<float, 3, 3> IMU_spin_matrix = [] {
    Eigen::Matrix<float, 3, 3> m;
    m <<  0, 1, 0, 
         -1, 0, 0,
          0, 0, 1;
    return m;
}();
// clang-format on
enum class chassis_mode : uint8_t {
    stop = 0,
    follow,
    sideways_L,
    sideways_R,
    spin,
    balanceless,
    unknown,
    spin_control,
};
enum class board : uint8_t {
    UNKNOWN           = 0,
    DM_MC02           = 1,
    Robomaster_Cboard = 2,
};
enum wheel : uint8_t {
    wheel_L = 1 - 1,
    wheel_R = 2 - 1,
};
enum leg : uint8_t {
    leg_LF = 1 - 1,
    leg_LB = 2 - 1,
    leg_RF = 3 - 1,
    leg_RB = 4 - 1,
};

constexpr board this_board = board::DM_MC02;

// public variables
static struct status_flags {
    bool IsControlling : 1 = false;
    bool IsSpinning    : 1 = false;
static bool IsControlling = false;
    bool stand_jump_cmd  : 1 = false;
    bool moving_jump_cmd : 1 = false;
} status_flag;
static chassis_mode chassis_mode_;
} // namespace app
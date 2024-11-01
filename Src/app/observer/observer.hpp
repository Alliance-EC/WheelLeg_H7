#pragma once
#include "app/system_parameters.hpp"
#include "leg_conv_reverse.hpp"
#include "leg_pos.hpp"
#include "module/DM8009/DM8009.hpp"
#include "module/IMU_EKF/IMU_EKF.hpp"
#include "tool/daemon/daemon.hpp"
#include "tool/filter/OLS.hpp"
#include "tool/filter/low_pass_filter.hpp"
#include "velocity_kalman.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace app::observer {
struct leg_length {
    double L;
    double Ld;
    double Ldd;
    double R;
    double Rd;
    double Rdd;
};
struct support_force {
    double L;
    double R;
};
class observer {
public:
    static observer* GetInstance() {
        static auto instance = new observer();
        return instance;
    }
    void update() {
        dt_ = DWT_GetDeltaT64_Expect(&last_time, app::dt);
        IMU_update();
        leg_update();
        wheel_update();
        support_force_update();
        levitate_detect();

        if ((*chassis_mode_ == chassis_mode::stop)
            || (*chassis_mode_ == chassis_mode::balanceless)) {
            reset_persistent_data();
            status_levitate_ = false;
        }
        // kalman_observer_update();
        auto phi = IMU_->output_vector.Yaw_multi_turn;
        x_states_ << distance_, // 1 x
            velocity_,          // 2 x_d
            phi,                // 3 phi
            imu_gyro_->z(),     // 4 phid
            theta_L_,           // 5 theta_ll
            theta_Ld_,          // 6 theta_lld
            theta_R_,           // 7 theta_lr
            theta_Rd_,          // 8 theta_lrd
            imu_euler_->y(),    // 9 theta_b
            imu_gyro_->y();     // 10 theta_bd
    }
    void Init(
        module::IMU* IMU, std::array<module::DM8009*, 4> DM8009,
        std::array<device::DjiMotor*, 2> M3508, Eigen::Vector<double, 4>* u_mat) {
        IMU_       = IMU;
        DM8009_    = DM8009;
        M3508_     = M3508;
        imu_accel_ = &IMU_->output_vector.accel;
        imu_gyro_  = &IMU_->output_vector.gyro;
        imu_euler_ = &IMU_->output_vector.euler_angle;
        u_mat_     = u_mat;
    }
    // output variables
    Eigen::Matrix<double, 10, 1> x_states_;
    leg_length leg_length_;
    support_force support_force_;
    bool status_levitate_;
    double roll_, roll_d_;
    double leg_length_avg_;

    velocity_kalman velocity_kalman_ = {};

private:
    observer()                                          = default; // 禁止外部构造
    ~observer()                                         = default; // 禁止外部析构
    observer(const observer& observer)                  = delete;  // 禁止外部拷贝构造
    const observer& operator=(const observer& observer) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double theta_L_ = 0, theta_Ld_ = 0, theta_Ldd_ = 0;
    double theta_R_ = 0, theta_Rd_ = 0, theta_Rdd_ = 0;
    double velocity_ = 0.0, distance_ = 0.0;

    module::IMU* IMU_                       = nullptr;
    std::array<module::DM8009*, 4> DM8009_  = {};
    std::array<device::DjiMotor*, 2> M3508_ = {};
    const chassis_mode* chassis_mode_       = &app::chassis_mode_;
    tool::daemon levitate_allow_timer_ =
        tool::daemon(1, std::bind(&observer::reset_levitate_allowance, this));

    double dt_ = app::dt;
    uint32_t last_time;
    bool allow_levitate_              = true;
    bool parking_mode_                = false;
    const Eigen::Vector3f *imu_accel_ = nullptr, *imu_gyro_ = nullptr, *imu_euler_ = nullptr;
    Eigen::Vector<double, 4>* u_mat_ = nullptr;
    // kalman_observer kalman_observer_ = {};

    tool::filter::LowPassFilter angle_Ld_LPF_ = tool::filter::LowPassFilter(100);
    tool::filter::LowPassFilter angle_Rd_LPF_ = tool::filter::LowPassFilter(100);
    tool::filter::LowPassFilter length_L_LPF_ = tool::filter::LowPassFilter(100);
    tool::filter::LowPassFilter length_R_LPF_ = tool::filter::LowPassFilter(100);
    tool::filter::OLS velocity_OLS_           = tool::filter::OLS(50);
    // clang-format on
    void IMU_update() {
        IMU_->Acquire();
        roll_   = imu_euler_->x();
        roll_d_ = imu_gyro_->x();
    }
    void leg_update() {
        static double last_theta_L   = 0.0;
        static double last_theta_R   = 0.0;
        static double last_theta_Ld  = 0.0;
        static double last_theta_Rd  = 0.0;
        static double last_length_L  = 0.0;
        static double last_length_R  = 0.0;
        static double last_length_Ld = 0.0;
        static double last_length_Rd = 0.0;
        double data[2];
        // left
        leg_pos(DM8009_[leg_LF]->get_angle(), DM8009_[leg_LB]->get_angle(), data);

        leg_length_.L = data[0];                                // ll
        leg_length_.L = length_L_LPF_.update(leg_length_.L);

        theta_L_ = data[1] + imu_euler_->y();                   // 5 theta_ll

        theta_Ld_    = (theta_L_ - last_theta_L) / dt_;         // 6 theta_lld
        theta_Ld_    = angle_Ld_LPF_.update(theta_Ld_);
        last_theta_L = theta_L_;

        theta_Ldd_    = (theta_Ld_ - last_theta_Ld) / dt_;
        last_theta_Ld = theta_Ld_;

        leg_length_.Ld = (leg_length_.L - last_length_L) / dt_; // lld
        last_length_L  = leg_length_.L;

        leg_length_.Ldd = (leg_length_.Ld - last_length_Ld) / dt_;
        last_length_Ld  = leg_length_.Ld;

        // right
        leg_pos(DM8009_[leg_RF]->get_angle(), DM8009_[leg_RB]->get_angle(), data);

        leg_length_.R = data[0];                                // lr
        leg_length_.R = length_R_LPF_.update(leg_length_.R);

        theta_R_ = data[1] + imu_euler_->y();                   // 7 theta_lr

        theta_Rd_    = (theta_R_ - last_theta_R) / dt_;         // 8 theta_lrd
        theta_Rd_    = angle_Rd_LPF_.update(theta_Rd_);
        last_theta_R = theta_R_;

        theta_Rdd_    = (theta_Rd_ - last_theta_Rd) / dt_;
        last_theta_Rd = theta_Rd_;

        leg_length_.Rd = (leg_length_.R - last_length_R) / dt_; // lrd
        last_length_R  = leg_length_.R;

        leg_length_.Rdd = (leg_length_.Rd - last_length_Rd) / dt_;
        last_length_Rd  = leg_length_.Rd;

        leg_length_avg_ = (leg_length_.L + leg_length_.R) / 2.0f;
    }

    void wheel_update() {
        // 驱动轮相对于大地的速度
        auto s_dot =
            (M3508_[wheel_L]->get_velocity() + M3508_[wheel_R]->get_velocity()) * Rw / 2.0f;
        //  https://zhuanlan.zhihu.com/p/689921165
        auto v_ll =
            leg_length_.L * std::cos(theta_L_) * theta_Ld_ + leg_length_.Ld * std::sin(theta_L_);
        auto v_lr =
            leg_length_.R * std::cos(theta_R_) * theta_Rd_ + leg_length_.Rd * std::sin(theta_R_);
        auto s_bd     = s_dot + (v_ll + v_lr) / 2;
        double z[2]   = {s_bd, IMU_->output_vector.accel_b.x()};
        auto s_bd_est = velocity_kalman_.update(z);
        velocity_     = s_bd_est;

        if (!status_flag.IsControlling && std::fabs(s_dot) < 1e-2) { // 等待验证
            parking_mode_ = true;
        } else if (status_flag.IsControlling) {
            parking_mode_ = false;
        }

        if (parking_mode_) { // when the robot is not moving, change to distance control
            velocity_kalman_.set_parking();
            distance_ += velocity_ * dt_;
        } else {
            reset_persistent_data();
            velocity_kalman_.set_normal();
        }
    }
    void support_force_update() {
        double data[2];
        leg_conv_reverse(
            DM8009_[leg_LF]->get_torque(), DM8009_[leg_LB]->get_torque(),
            DM8009_[leg_LF]->get_angle(), DM8009_[leg_LB]->get_angle(), data);
        auto F_l       = data[0];
        auto T_l       = data[1];
        auto P_l       = F_l * std::cos(theta_L_) + T_l * std::sin(theta_L_) / leg_length_.L;
        auto z_wl_ddot = IMU_->get_accel_n().z() - leg_length_.Ldd * std::cos(theta_L_)
                       + 2 * leg_length_.Ld * std::sin(theta_L_) * theta_Ld_
                       + leg_length_.L * std::cos(theta_L_) * theta_Ld_ * theta_Ld_
                       + leg_length_.L * std::sin(theta_L_) * theta_Ldd_;
        support_force_.L = P_l + z_wl_ddot * m_w;
        leg_conv_reverse(
            DM8009_[leg_RF]->get_torque(), DM8009_[leg_RB]->get_torque(),
            DM8009_[leg_RF]->get_angle(), DM8009_[leg_RB]->get_angle(), data);
        auto F_r       = data[0];
        auto T_r       = data[1];
        auto P_r       = F_r * std::cos(theta_R_) + T_r * std::sin(theta_R_) / leg_length_.R;
        auto z_wr_ddot = IMU_->get_accel_n().z() - leg_length_.Rdd * std::cos(theta_R_)
                       + 2 * leg_length_.Rd * std::sin(theta_R_) * theta_Rd_
                       + leg_length_.R * std::cos(theta_R_) * theta_Rd_ * theta_Rd_
                       + leg_length_.R * std::sin(theta_R_) * theta_Rdd_;
        support_force_.R = P_r + z_wr_ddot * m_w;
    }
    void levitate_detect() {
        constexpr double force_levitate = 20.0;
        constexpr double force_normal   = 50.0;
        auto support_force_avg          = (support_force_.L + support_force_.R) / 2.0;
        if ((support_force_avg < force_levitate) && allow_levitate_
            && (*chassis_mode_ != chassis_mode::stop)
            && (*chassis_mode_ != chassis_mode::balanceless)) {
            status_levitate_ = true;
        } else if (
            (support_force_avg > force_normal) && (*chassis_mode_ != chassis_mode::stop)
            && (*chassis_mode_ != chassis_mode::balanceless)) {
            if (status_levitate_ == true) {
                allow_levitate_ = false;
                levitate_allow_timer_.reload();
            }
            status_levitate_ = false;
        }

        if (status_levitate_)
            reset_persistent_data();
    }
    void reset_persistent_data() {
        distance_ = 0;
        for (auto& i : M3508_)
            i->reset_angle();
    }
    void reset_levitate_allowance() { allow_levitate_ = true; }
};

} // namespace app::observer

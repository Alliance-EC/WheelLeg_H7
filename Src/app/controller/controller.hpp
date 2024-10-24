#pragma once
#include "LQR_k.hpp"
#include "LegConv.hpp"
#include "app/observer/observer.hpp"
#include "app/system_parameters.hpp"
#include "desire_set.hpp"
#include "device/super_cap/super_cap.hpp"
#include "module/IMU_EKF/IMU_EKF.hpp"
#include "module/referee/status.hpp"
#include "speed_hat.hpp"
#include "tool/pid/pid.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
namespace app::controller {
using namespace tool;
struct control_torque {
    double wheel_L; // velocity target when in balanceless mode
    double wheel_R; // velocity target when in balanceless mode
    double leg_LF;
    double leg_LB;
    double leg_RF;
    double leg_RB;
};
class Controller {
public:
    static Controller* GetInstance() {
        static auto instance = new Controller();
        return instance;
    }

    void update() {
        do {
            super_cap_controller();
            if (*mode_ == chassis_mode::balanceless) {
                velocity_control((*xd_)(1, 0) * 0.5, (*xd_)(2, 0) - (*x_states_)(2, 0));
                break;
            } else if (*mode_ == chassis_mode::stop) {
                stop_all_control();
                break;
            }
            kinematic_controller();
            anti_fall_check();
            leg_controller();
            // wheel_model_hat();
            leg_split_corrector();
            torque_process();
        } while (false);
    }
    void Init(
        module::IMU* IMU, std::array<module::DM8009*, 4> DM8009,
        std::array<device::DjiMotor*, 2> M3508, device::SuperCap* super_cap,
        module::referee::Status* referee) {
        IMU_      = IMU;
        DM8009_   = DM8009;
        M3508_    = M3508;
        imu_euler = &IMU_->output_vector.euler_angle;
        imu_gyro  = &IMU_->output_vector.gyro;
        referee_  = referee;
        SuperCap_ = super_cap;
    }

    control_torque control_torque_ = {};
    Eigen::Vector<double, 4> u_mat = {};

private:
    Controller()                                              = default; // 禁止外部构造
    ~Controller()                                             = default; // 禁止外部析构
    Controller(const Controller& Controller)                  = delete;  // 禁止外部拷贝构造
    const Controller& operator=(const Controller& Controller) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double F_l_ = 0, F_r_ = 0;
    double T_lwl_ = 0, T_lwr_ = 0, T_bll_ = 0, T_blr_ = 0;
    double T_lwl_compensate_ = 0, T_lwr_compensate_ = 0;
    double T_bll_compensate_ = 0, T_blr_compensate_ = 0;
    double wheel_speed_hat_[2]      = {};
    bool about_to_fall_             = false;
    tool::daemon fall_resume_timer_ = tool::daemon(0.5, std::bind(&Controller::fall_resume, this));

    double wheel_compensate_kp_ = 0.25;
    PID pid_roll_               = PID({300, 0, 0, 500, 0.0, 0.0, dt});
    PID pid_roll_d_             = PID({30, 0, 0, 500, 0.0, 0.0, dt});
    PID pid_length_             = PID({10, 0, 0, 10, 100.0, 0.0, dt});
    PID pid_length_d_           = PID({150, 0, 0, 500, 100.0, 0.0, dt});

    PID wheel_L_PID_ = PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});
    PID wheel_R_PID_ = PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});

    PID leg_split_corrector_PID_ = PID({10, 0.0, 0.0005, 4.0, 0.0, 0.0, dt}); // PD

    observer::observer* observer_           = observer::observer::GetInstance();
    DesireSet* desire_                      = controller::DesireSet::GetInstance();
    std::array<module::DM8009*, 4> DM8009_  = {};
    std::array<device::DjiMotor*, 2> M3508_ = {};
    module::IMU* IMU_                       = nullptr;
    device::SuperCap* SuperCap_             = nullptr;
    module::referee::Status* referee_       = nullptr;

    const Eigen::Matrix<double, 10, 1>* xd_       = &desire_->desires.xd;
    const Eigen::Matrix<double, 10, 1>* x_states_ = &observer_->x_states_;
    const observer::leg_length* leg_length_       = &observer_->leg_length_;
    const double* length_desire_                  = &desire_->desires.leg_length;
    const double* roll_desire_                    = &desire_->desires.roll;
    const Eigen::Vector3f *imu_euler = nullptr, *imu_gyro = nullptr;
    const chassis_mode* mode_ = &chassis_mode_;

    void anti_fall_check() {
        if (fabs(imu_euler->y()) > (0.1 + observer_->leg_length_avg_ * 0.5)) {
            about_to_fall_ = true;
            fall_resume_timer_.reload();
        }
        if (observer_->status_levitate_) // 腾空状态不会触发
            about_to_fall_ = false;
    }
    void fall_resume() { about_to_fall_ = false; }

    void super_cap_controller() {
        // Maximum excess power when buffer energy is sufficient.
        constexpr double excess_power_limit = 35;
        //               power_limit_after_buffer_energy_closed_loop =
        constexpr double buffer_energy_control_line = 120; // = referee + excess
        constexpr double buffer_energy_base_line    = 50;  // = referee
        constexpr double buffer_energy_dead_line    = 0;   // = 0

        device::SuperCapControl SuperCap_set_ = {};
        const auto power_limit                = referee_->chassis_power_limit_;
        const auto power                      = referee_->chassis_power_;
        const auto power_buffer               = referee_->buffer_energy_;

        double power_limit_after_buffer_energy_closed_loop =
            power_limit
                * std::clamp(
                    (power_buffer - buffer_energy_dead_line)
                        / (buffer_energy_base_line - buffer_energy_dead_line),
                    0.0, 1.0)
            + excess_power_limit
                  * std::clamp(
                      (power_buffer - buffer_energy_base_line)
                          / (buffer_energy_control_line - buffer_energy_base_line),
                      0.0, 1.0);
        SuperCap_set_.power_limit =
            static_cast<uint8_t>(power_limit_after_buffer_energy_closed_loop);
        /*超电自动开启后在缓冲能量充满时关闭，操作手按键开启的则不会自动关闭*/
        if ((power > power_limit) && (power_buffer / (power - power_limit) < 0.3)) {
            SuperCap_set_.enable = true;
        } else if (power_buffer >= 60.0) {
            SuperCap_set_.enable = false;
        }

        if (desire_->SuperCap_ON_) {
            SuperCap_set_.enable = true;
        }
        if (SuperCap_->Info.supercap_voltage_ < 12.0) {
            SuperCap_set_.enable = false;
        }
        SuperCap_->write_data(SuperCap_set_);
    }
    void kinematic_controller() {
        double lqr_k[40];
        LQR_k(leg_length_->L, leg_length_->R, lqr_k);
        Eigen::Map<Eigen::Matrix<double, 4, 10, Eigen::ColMajor>> LQR_gain(lqr_k);

        Eigen::Matrix<double, 10, 1> e_mat = -1.0 * (*xd_ - *x_states_);
        if (observer_->status_levitate_) { // 腾空状态仅保持腿部竖直
            e_mat(0, 0) = 0;
            e_mat(1, 0) = 0;
            e_mat(2, 0) = 0;
            e_mat(3, 0) = 0;

            e_mat(8, 0) = 0;
            e_mat(9, 0) = 0;
        }
        u_mat  = LQR_gain * e_mat;
        T_lwl_ = u_mat(0, 0);
        T_lwr_ = u_mat(1, 0);
        T_bll_ = u_mat(2, 0);
        T_blr_ = u_mat(3, 0);
    }

    void leg_controller() {
        auto length   = (leg_length_->L + leg_length_->R) / 2.0;
        auto length_d = (leg_length_->Ld + leg_length_->Rd) / 2.0;
        /*重力前馈和侧向力前馈*/
        constexpr auto gravity_ff = [=]() { return (m_b / 2.0 + eta_l * m_l) * gravity; };
        auto inertial_ff          = [=, this]() {
            auto coefficient = length / (2 * R_l) * (*x_states_)(3, 0) * (*x_states_)(1, 0);
            return (m_b / 2.0 + eta_l * m_l) * coefficient;
            // return 0;
        };

        auto length_desire = about_to_fall_ ? 0.12 : *length_desire_;
        auto roll_desire   = observer_->status_levitate_ ? 0.0 : *roll_desire_;

        static PID_params pid_roll_param_storage   = pid_roll_.GetParams();
        static PID_params pid_roll_d_param_storage = pid_roll_d_.GetParams();
        static chassis_mode last_mode              = chassis_mode::stop;
        if (last_mode != chassis_mode::spin_control && *mode_ == chassis_mode::spin_control) {
            pid_roll_.ChangeParams({1500, 0, 0, 500, 0.0, 0.0, dt});
            pid_roll_d_.ChangeParams({45, 0, 0, 500, 0.0, 0.0, dt});
        } else if (
            last_mode == chassis_mode::spin_control && *mode_ != chassis_mode::spin_control) {
            pid_roll_.ChangeParams(pid_roll_param_storage);
            pid_roll_d_.ChangeParams(pid_roll_d_param_storage);
        }

        auto length_out      = pid_length_.update(length_desire, length);
        auto F_length        = pid_length_d_.update(length_out, length_d);
        auto roll_angle_out  = pid_roll_.update(roll_desire, observer_->roll_);
        auto roll_angled_out = pid_roll_d_.update(0, observer_->roll_d_);
        auto F_roll          = roll_angle_out + roll_angled_out;

        F_l_ = F_length + F_roll + gravity_ff() - inertial_ff();
        F_r_ = F_length - F_roll + gravity_ff() + inertial_ff();

        last_mode = *mode_;
    }
    void wheel_model_hat() {
        constexpr double predict_dt = 0.001; // 预测之后多少时间的值
        speed_hat(
            T_bll_, T_blr_, T_lwl_, T_lwr_, predict_dt, leg_length_->L, leg_length_->R,
            (*x_states_)(3, 0), (*x_states_)(1, 0), (*x_states_)(4, 0), (*x_states_)(6, 0),
            wheel_speed_hat_);

        constexpr double slip_start = 2;
        if (std::abs(wheel_speed_hat_[0] - M3508_[wheel_L]->get_velocity()) > slip_start) {
            T_lwl_compensate_ =
                wheel_compensate_kp_ * (wheel_speed_hat_[0] - M3508_[wheel_L]->get_velocity());
        }
        if (std::abs(wheel_speed_hat_[1] - M3508_[wheel_R]->get_velocity()) > slip_start) {
            T_lwr_compensate_ =
                wheel_compensate_kp_ * (wheel_speed_hat_[1] - M3508_[wheel_R]->get_velocity());
        }
    }
    void leg_split_corrector() {
        auto leg_split_correct_torque = leg_split_corrector_PID_.update(
            0, (*x_states_)(4, 0) - (*x_states_)(6, 0), (*x_states_)(5, 0) - (*x_states_)(7, 0));
        T_bll_compensate_ = -leg_split_correct_torque;
        T_blr_compensate_ = +leg_split_correct_torque;
    }
    void torque_process() {
        constexpr double LEG_MOTOR_T_MAX = 40.0f;
        constexpr double LEG_T_MAX       = 15.0f;

        T_lwl_ += T_lwl_compensate_;
        T_lwr_ += T_lwr_compensate_;
        T_bll_ += T_bll_compensate_;
        T_blr_ += T_blr_compensate_;

        if (observer_->status_levitate_) {
            T_lwl_ = -wheel_L_PID_.update(0, M3508_[wheel_L]->get_velocity());
            T_lwr_ = -wheel_R_PID_.update(0, M3508_[wheel_R]->get_velocity());
        }

        constexpr double max_torque_wheel = 5.0f;
        control_torque_.wheel_L           = std::clamp(T_lwl_, -max_torque_wheel, max_torque_wheel);
        control_torque_.wheel_R           = std::clamp(T_lwr_, -max_torque_wheel, max_torque_wheel);
        if (*mode_ != chassis_mode::balanceless) {
            control_torque_.wheel_L *= -1;
            control_torque_.wheel_R *= -1;
        }
        /*虚拟腿扭矩限幅，防止受大冲击时大幅度震荡*/
        T_bll_ = std::clamp(T_bll_, -LEG_T_MAX, LEG_T_MAX);
        T_blr_ = std::clamp(T_blr_, -LEG_T_MAX, LEG_T_MAX);
        /*VMC解算到关节电机扭矩*/
        double leg_T[2];
        leg_conv(F_l_, T_bll_, DM8009_[leg_LF]->get_angle(), DM8009_[leg_LB]->get_angle(), leg_T);
        control_torque_.leg_LF = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_LB = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        leg_conv(F_r_, T_blr_, DM8009_[leg_RF]->get_angle(), DM8009_[leg_RB]->get_angle(), leg_T);
        control_torque_.leg_RF = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_RB = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
    }

    void stop_all_control() {
        control_torque_.wheel_L = nan;
        control_torque_.wheel_R = nan;
        control_torque_.leg_LF  = nan;
        control_torque_.leg_LB  = nan;
        control_torque_.leg_RF  = nan;
        control_torque_.leg_RB  = nan;
    }
    void velocity_control(double x_speed, double angle_error = 0.0) {
        control_torque_.leg_LF = nan;
        control_torque_.leg_LB = nan;
        control_torque_.leg_RF = nan;
        control_torque_.leg_RB = nan;

        double rotation_velocity = std::clamp(10 * angle_error, -3.0, 3.0);
        control_torque_.wheel_L  = (x_speed - rotation_velocity * R_l) / Rw;
        control_torque_.wheel_R  = (x_speed + rotation_velocity * R_l) / Rw;
    }
};
} // namespace app::controller
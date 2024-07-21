#pragma once
#include "LQR_k.hpp"
#include "LegConv.hpp"
#include "app/observer/observer.hpp"
#include "app/system_parameters.hpp"
#include "desire_set.hpp"
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
            // super_cap_controller();
            if (*mode_ == chassis_mode::balanceless) {
                velocity_control((*xd_)(1, 0) * 0.5, (*xd_)(2, 0) - (*x_states_)(2, 0));
                break;
            } else if (*mode_ == chassis_mode::stop) {
                stop_all_control();
                break;
            }
            kinematic_controller();
            leg_controller();
            wheel_model_hat();
            // leg_coordinator();
            torque_process();
        } while (false);
    }
    void Init(
        module::IMU* IMU, std::array<module::DM8009*, 4> DM8009,
        std::array<device::DjiMotor*, 2> M3508, chassis_mode* chassis_mode) {
        IMU_      = IMU;
        DM8009_   = DM8009;
        M3508_    = M3508;
        mode_     = chassis_mode;
        imu_euler = &IMU_->output_vector.euler_angle;
        imu_gyro  = &IMU_->output_vector.gyro;
    }

    control_torque control_torque_ = {};

private:
    Controller()                                              = default; // 禁止外部构造
    ~Controller()                                             = default; // 禁止外部析构
    Controller(const Controller& Controller)                  = delete;  // 禁止外部拷贝构造
    const Controller& operator=(const Controller& Controller) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double F_l = 0, F_r = 0;
    double T_lwl_ = 0, T_lwr_ = 0, T_bll_ = 0, T_blr_ = 0;
    double T_lwl_compensate_ = 0, T_lwr_compensate_ = 0;
    double T_bll_compensate_ = 0, T_blr_compensate_ = 0;
    double wheel_compensate_kp_ = 0.0;
    PID pid_roll_               = PID({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dt});
    PID pid_length_             = PID({0.0, 0.0, 0.0, 100.0, 100.0, 0.0, dt});

    observer::observer* observer_instance   = observer::observer::GetInstance();
    DesireSet* desire_instance              = controller::DesireSet::GetInstance();
    std::array<module::DM8009*, 4> DM8009_  = {};
    std::array<device::DjiMotor*, 2> M3508_ = {};
    module::IMU* IMU_                       = nullptr;

    const Eigen::Matrix<double, 10, 1>* xd_       = &desire_instance->desires.xd;
    const Eigen::Matrix<double, 10, 1>* x_states_ = &observer_instance->x_states_;
    const observer::leg_length* leg_length_       = &observer_instance->leg_length_;
    const double* length_desire_                  = &desire_instance->desires.leg_length;
    const double* roll_desire_                    = &desire_instance->desires.roll;
    const Eigen::Vector3f *imu_euler = nullptr, *imu_gyro = nullptr;
    const chassis_mode* mode_ = nullptr;

    int safe_check() {
        static int mode, last_mode, pitch_count_, leg_angle_count_, count_;
        static double length = 0.12;
        length               = (leg_length_->L + leg_length_->R) / 2.0;
        count_++;
        /*大倾角检测0.5s，高姿倒地自动收腿起身*/
        if (fabs(imu_euler->y()) > (0.1 + length * 2.0)) {
            pitch_count_++;
        }
        if ((fabs((*x_states_)(4, 0)) > (0.2 + length * 2.5))
            || (fabs((*x_states_)(6, 0)) > (0.2 + length * 2.5))) {
            leg_angle_count_++;
        }
        /*状态判断,触发保护后，直到速度稳定才解除*/
        if (count_ >= 500) {
            count_ = 0;
            if (pitch_count_ > 500) {
                pitch_count_ = 0;
                mode         = 1;
            } else if (leg_angle_count_ > 500) {
                leg_angle_count_ = 0;
                mode             = 2;
            } else if (last_mode) {
                if (fabs((*x_states_)(1, 0)) < 0.3) {
                    mode = 0;
                }
            }
        }
        last_mode = mode;
        return mode;
    }
    void kinematic_controller() {
        double lqr_k[40];
        LQR_k(leg_length_->L, leg_length_->R, lqr_k);
        Eigen::Map<Eigen::Matrix<double, 4, 10, Eigen::ColMajor>> LQR_gain(lqr_k);

        Eigen::Matrix<double, 10, 1> e_mat = -1.0 * (*xd_ - *x_states_);
        // if (*status_levitate_) {
        //     e_mat(0, 0) = 0;
        //     e_mat(1, 0) = 0;
        // }
        auto u_mat = LQR_gain * e_mat;
        T_lwl_     = u_mat(0, 0);
        T_lwr_     = u_mat(1, 0);
        T_bll_     = u_mat(2, 0);
        T_blr_     = u_mat(3, 0);
    }

    void leg_controller() {
        auto length = (leg_length_->L + leg_length_->R) / 2.0;
        /*重力前馈和侧向力前馈*/
        auto gravity_ff  = [=]() { return (m_b / 2.0 + eta_l * m_l) * g; };
        auto inertial_ff = [=, this]() {
            auto coefficient = length / (2 * R_l) * (*x_states_)(3, 0) * (*x_states_)(1, 0);
            return (m_b / 2.0 + eta_l * m_l) * coefficient;
            // return 0;
        };

        auto length_desire = *length_desire_;
        // auto roll_desire                     = *status_levitate_ ? 0.0 : *roll_desire_;
        auto roll_desire = *roll_desire_;

        /*roll和腿长PID运算*/
        auto F_length = pid_length_.update(length_desire, length);
        auto F_roll   = pid_roll_.update(roll_desire, imu_euler->x(), imu_gyro->x());

        F_l = F_length + F_roll + gravity_ff() - inertial_ff();
        F_r = F_length - F_roll + gravity_ff() + inertial_ff();
    }
    void wheel_model_hat() {
        double wheel_speed_hat[2];
        constexpr double predict_dt = 0.001; // 预测之后多少时间的值
        speed_hat(
            T_bll_, T_blr_, T_lwl_, T_lwr_, predict_dt, leg_length_->L, leg_length_->R,
            (*x_states_)(3, 0), (*x_states_)(1, 0), (*x_states_)(4, 0), (*x_states_)(6, 0),
            wheel_speed_hat);

        T_lwl_compensate_ =
            wheel_compensate_kp_ * (wheel_speed_hat[0] - M3508_[wheel_L]->get_velocity());
        T_lwr_compensate_ =
            wheel_compensate_kp_ * (wheel_speed_hat[1] - M3508_[wheel_R]->get_velocity());
    }
    void torque_process() {
        constexpr double LEG_MOTOR_T_MAX = 40.0f;
        constexpr double LEG_T_MAX       = 15.0f;

        T_lwl_ += T_lwl_compensate_;
        T_lwr_ += T_lwr_compensate_;
        T_bll_ += T_bll_compensate_;
        T_blr_ += T_blr_compensate_;

        /*功率扭矩计算进行轮电机输出限制*/
        // if (super_cap_fdb_->status == true) {
        //     wheel_power_.setPowerMax(999.0);
        // } else {
        //     wheel_power_.setPowerMax(720.0);
        // }
        // wheel_power_.limitTouque(T_lwl_, motor_fdb_->speed_l, T_lwr_, motor_fdb_->speed_r);
        constexpr double max_torque_wheel = 5.0f;
        control_torque_.wheel_L           = std::clamp(T_lwl_, -max_torque_wheel, max_torque_wheel);
        control_torque_.wheel_R           = std::clamp(T_lwr_, -max_torque_wheel, max_torque_wheel);

        /*虚拟腿扭矩限幅，防止受大冲击时大幅度震荡*/
        T_bll_ = std::clamp(T_bll_, -LEG_T_MAX, LEG_T_MAX);
        T_blr_ = std::clamp(T_blr_, -LEG_T_MAX, LEG_T_MAX);
        /*VMC解算到关节电机扭矩*/
        double leg_T[2];
        leg_conv(F_l, T_bll_, DM8009_[leg_LF]->get_angle(), DM8009_[leg_LB]->get_angle(), leg_T);
        control_torque_.leg_LF = std::clamp(leg_T[0], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        control_torque_.leg_LB = std::clamp(leg_T[1], -LEG_MOTOR_T_MAX, LEG_MOTOR_T_MAX);
        leg_conv(F_r, T_blr_, DM8009_[leg_RF]->get_angle(), DM8009_[leg_RB]->get_angle(), leg_T);
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
        control_torque_.wheel_L  = static_cast<float>((x_speed - rotation_velocity * R_l) / Rw);
        control_torque_.wheel_R  = static_cast<float>((x_speed + rotation_velocity * R_l) / Rw);
    }
};
} // namespace app::controller
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
#include "tool/filter/band_stop_fliter.hpp"
#include "tool/pid/pid.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
bool watch_fall=false;
double watch_wheel_speed;
double lqr_torque;
double limit_torque;
double debug;
namespace app::controller {
using namespace tool;
struct control_torque {
    double wheel_L; // velocity target when in balanceless mode
    double wheel_R; // velocity target when in balanceless mode
    double leg_LF;
    double leg_LB;
    double leg_RB;
    double leg_RF;
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
            jumping_fsm();
            kinematic_controller();
            anti_fall_check();
            watch_fall = about_to_fall_;
            leg_controller();
            wheel_speed_limit();
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

    enum class jump_stage : uint8_t {
        ready_to_jump = 0,
        prepare_to_jump,
        contracting_legs,
        extending_legs,
        contracting_legs_air,
        prepare_landing,
        finish
    };
    enum class climb_stage : uint8_t {
        ready_to_climb = 0,
        detect_step,
        backward_legs,
        extend_legs,
        restand
    };
    double F_l_ = 0, F_r_ = 0;
    double T_lwl_ = 0, T_lwr_ = 0, T_bll_ = 0, T_blr_ = 0;
    double T_lwl_compensate_ = 0, T_lwr_compensate_ = 0;
    double T_bll_compensate_ = 0, T_blr_compensate_ = 0;
    double wheel_speed_hat_[2]      = {};
    bool about_to_fall_             = false;
    bool about_to_fall_last_             = false;
    bool allow_re_stand             = false;
    tool::daemon fall_resume_timer_ = tool::daemon(0.5, std::bind(&Controller::fall_resume, this));
    double wheel_compensate_kp_ = 0.25;
    PID pid_roll_               = PID({300, 0, 0, 500, 0.0, 0.0, dt});
    PID pid_roll_d_             = PID({30, 0, 0, 500, 0.0, 0.0, dt});
    PID pid_length_             = PID({12, 0, 0, 6, 100.0, 0.0, dt});
    PID pid_length_d_           = PID({150, 0, 0, 400, 100.0, 0.0, dt});

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

    Eigen::Matrix<double, 10, 1>* xd_ = &desire_->desires.xd;    // will be modified by jumping_fsm
    const Eigen::Matrix<double, 10, 1>* x_states_ = &observer_->x_states_;
    const observer::leg_length* leg_length_       = &observer_->leg_length_;
    double* length_desire_     = &desire_->desires.leg_length;   // will be modified by jumping_fsm
    //
    double*  s_d_limit=&desire_->power_limit_velocity;

    const double* roll_desire_ = &desire_->desires.roll;
    const Eigen::Vector3f *imu_euler = nullptr, *imu_gyro = nullptr;
    const chassis_mode* mode_ = &chassis_mode_;

    jump_stage jump_stage_ = jump_stage::ready_to_jump;
    climb_stage climb_stage_ = climb_stage::ready_to_climb;

    tool::filter::BandStopFilter T_l_BSF = tool::filter::BandStopFilter(25, 32, 1000, 3);
    tool::filter::BandStopFilter T_r_BSF = tool::filter::BandStopFilter(25, 32, 1000, 3);
    void jumping_fsm() {
        static PID_params pid_length_param_storage   = pid_length_.GetParams();
        static PID_params pid_length_d_param_storage = pid_length_d_.GetParams();
        auto leg_length                              = (leg_length_->L + leg_length_->R) / 2.0;

        switch (jump_stage_) {
        case jump_stage::ready_to_jump: {
            if (status_flag.stand_jump_cmd)
                jump_stage_ = jump_stage::prepare_to_jump;
            if (status_flag.moving_jump_cmd)
                jump_stage_ = jump_stage::contracting_legs;
            break;
        }
        case jump_stage::prepare_to_jump: {
            constexpr double ideal_leg_angle = 0.5;
            (*xd_)(4, 0) = (*xd_)(6, 0) = ideal_leg_angle;
            auto error_L                = ideal_leg_angle - (*x_states_)(4, 0);
            auto error_R                = ideal_leg_angle - (*x_states_)(6, 0);

            if (fabs(error_L) < 0.2 && fabs(error_R) < 0.2) {
                jump_stage_ = jump_stage::contracting_legs;
                break;
            }
        }
        case jump_stage::contracting_legs: {
            *length_desire_             = 0.10;
            observer_->status_levitate_ = false;
            if (leg_length < 0.13) {
                jump_stage_ = jump_stage::extending_legs;
            }
            break;
        }
        case jump_stage::extending_legs: {
            pid_length_.ChangeParams({50, 0, 0, 50, 0, 0});
            pid_length_d_.ChangeParams({200, 0, 0, 1000, 0, 0}); // for fast leg movement
            *length_desire_ = 0.35;

            if (leg_length > 0.32) {
                jump_stage_ = jump_stage::contracting_legs_air;
            }
            break;
        }
        case jump_stage::contracting_legs_air: {
            *length_desire_ = 0.14;
            observer_->status_levitate_ = true;
            if (leg_length < 0.15) {
                jump_stage_ = jump_stage::prepare_landing;
            }
            break;
        }
        case jump_stage::prepare_landing: {
            observer_->status_levitate_ = true;

            if (leg_length < 0.13) {
                pid_length_.ChangeParams(pid_length_param_storage);
                pid_length_d_.ChangeParams(pid_length_d_param_storage); // restore to normal
                jump_stage_ = jump_stage::finish;
            }
            break;
        }
        case jump_stage::finish: {
            observer_->status_levitate_ = false;
            jump_stage_                 = jump_stage::ready_to_jump;
            status_flag.stand_jump_cmd  = false;
            status_flag.moving_jump_cmd = false;
            break;
        }
        default: break;
        }
    }
    //倾角过大，判定要翻倒
    void anti_fall_check() {
        about_to_fall_last_=about_to_fall_;
        if (fabs(imu_euler->y()) > (0.1 + observer_->leg_length_avg_ * 0.5)) {
            about_to_fall_ = true;
            fall_resume_timer_.reload();
        }
        if (observer_->status_levitate_)                                // 腾空状态不会触发
            about_to_fall_ = false;
    }
    void fall_resume() { 
        about_to_fall_ = false;
        }
    void climb_fsm(){
        const double F_x_max=30.0;//摆角受力 30N 认为堵转
        const double theta_b_normal=0.08;
        auto leg_length     = (leg_length_->L + leg_length_->R) / 2.0;
        Eigen::Vector<double, 3> x_error;
        uint8_t direction = 1;
        x_error(0) = std::abs((*xd_)(4, 0) - (*x_states_)(4, 0));
        x_error(1) = std::abs((*xd_)(6, 0) - (*x_states_)(6, 0));
        x_error(2) = std::abs((*xd_)(8, 0) - (*x_states_)(8, 0));
        
        switch (climb_stage_) {
        case climb_stage::ready_to_climb:{
            if (status_flag.set_to_climb==true){
                climb_stage_=climb_stage::detect_step;
            }
            break;
        }
        case climb_stage::detect_step:{
            if (observer_->F_x_.L > F_x_max && observer_->F_x_.R >F_x_max){
                climb_stage_ = climb_stage::backward_legs;
                direction=1;
            }
            else if (observer_->F_x_.L < -F_x_max && observer_->F_x_.R <-F_x_max){
                climb_stage_ = climb_stage::backward_legs;
                direction=-1;
            }
            break;
        }
        case climb_stage::backward_legs:{
            // constexpr double target_leg_angle = 0.5;
            // (*xd_)(4, 0) = (*xd_)(6, 0) = target_leg_angle*direction;
            if (x_error (0)< 0.1 && x_error (1)< 0.1) {
                climb_stage_ = climb_stage::extend_legs;
            }
            break;
        }
        case climb_stage::extend_legs: {
            *length_desire_ = 0.12;
            if (leg_length < 0.13) {
                climb_stage_ = climb_stage::restand;
            }
            break;
        }
        case climb_stage::restand: {
            if (x_error(2) < theta_b_normal) {
                climb_stage_=climb_stage::ready_to_climb;
            }
            break;
        }
    }
    }
    void super_cap_controller() {
        // Maximum excess  when buffer energy is sufficient.
        constexpr double excess_power_limit = 35;
        //电管可通过的最大功率
        constexpr double refree_power_max = 200.0;
        //               power_limit_after_buffer_energy_closed_loop =
        constexpr double buffer_energy_control_line = 120; // = referee + excess
        constexpr double buffer_energy_base_line    = 50;  // = referee
        constexpr double buffer_energy_dead_line    = 0;   // = 0

        device::SuperCapControl SuperCap_set_ = {};
        const auto power_limit                = referee_->chassis_power_limit_;
        const auto power                      = referee_->chassis_power_;
        const auto power_buffer               = referee_->buffer_energy_;

        double power_limit_after_buffer_energy_closed_loop =
            power_limit* std::clamp((power_buffer - buffer_energy_dead_line)
                        / (buffer_energy_base_line - buffer_energy_dead_line),0.0, 1.0)
            + excess_power_limit
                  * std::clamp((power_buffer - buffer_energy_base_line)
                        / (buffer_energy_control_line - buffer_energy_base_line),0.0, 1.0);
        SuperCap_set_.power_limit =static_cast<uint8_t>(power_limit_after_buffer_energy_closed_loop);
        /*超电自动开启后在缓冲能量充满时关闭，操作手按键开启的则不会自动关闭*/
        if ((power > power_limit) && (power_buffer / (power - power_limit) < 0.3)) {
            SuperCap_set_.enable = true;
        } else if (SuperCap_->Info.chassis_power_ > refree_power_max) {
            SuperCap_set_.enable = true;
        } else if (power_buffer >= 60.0) {
            SuperCap_set_.enable = false;
        }
        debug=SuperCap_->Info.chassis_power_;
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
        if (status_flag.IsSpinning)
            LQR_k_spin(leg_length_->L, leg_length_->R, lqr_k);
        else
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

        lqr_torque = T_lwl_;
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
            pid_roll_.ChangeParams({1500, 0, 0, 500, 0, 0});
            pid_roll_d_.ChangeParams({45, 0, 0, 500, 0, 0});
        } else if (last_mode == chassis_mode::spin_control && *mode_ != chassis_mode::spin_control) {
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
        // constexpr double predict_dt = 0.001; // 预测之后多少时间的值
        // speed_hat(
        //     T_bll_, T_blr_, T_lwl_, T_lwr_, predict_dt, leg_length_->L, leg_length_->R,
        //     (*x_states_)(3, 0), (*x_states_)(1, 0), (*x_states_)(4, 0), (*x_states_)(6, 0),
        //     wheel_speed_hat_);

        // constexpr double slip_start = 2;
        // if (std::abs(wheel_speed_hat_[0] - M3508_[wheel_L]->get_velocity()) > slip_start) {
        //     T_lwl_compensate_ =
        //         wheel_compensate_kp_ * (wheel_speed_hat_[0] - M3508_[wheel_L]->get_velocity());
        // }
        // if (std::abs(wheel_speed_hat_[1] - M3508_[wheel_R]->get_velocity()) > slip_start) {
        //     T_lwr_compensate_ =
        //         wheel_compensate_kp_ * (wheel_speed_hat_[1] - M3508_[wheel_R]->get_velocity());
        // }
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

        if (observer_->status_levitate_ || (climb_stage_ != climb_stage::restand && climb_stage_ != climb_stage::ready_to_climb)) {
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
    void wheel_speed_limit(){
        double wheel_speed_max = *s_d_limit / Rw;
        const double kp =2;
        watch_wheel_speed=wheel_speed_max;
        if (M3508_[0]->get_velocity() > wheel_speed_max+2) {
            T_lwl_ -= kp * (M3508_[0]->get_velocity() - wheel_speed_max);
        } else if (M3508_[0]->get_velocity() < -wheel_speed_max - 2){
            T_lwl_ -= kp * (M3508_[0]->get_velocity() + wheel_speed_max);
        }
        
        if (M3508_[1]->get_velocity() > wheel_speed_max + 2) {
            T_lwr_ += kp * (M3508_[1]->get_velocity() - wheel_speed_max);
            limit_torque =kp * (M3508_[1]->get_velocity() - wheel_speed_max);
        } else if (M3508_[1]->get_velocity() < -wheel_speed_max - 2) {
            T_lwr_ += kp * (M3508_[1]->get_velocity() + wheel_speed_max);
            limit_torque = kp * (M3508_[1]->get_velocity() + wheel_speed_max);
        } else {
            limit_torque = 0;
        }
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
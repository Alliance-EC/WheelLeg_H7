#pragma once
#include "app/system_parameters.hpp"
#include "bsp/dwt/dwt.h"
#include "device/Dji_motor/DJI_motor.hpp"
#include "device/RC/remote_control_data.hpp"
#include "module/IMU/IMU.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <numbers>

namespace app::controller {
struct desire {
    Eigen::Matrix<double, 10, 1> xd = Eigen::Matrix<double, 10, 1>::Zero();
    double roll                     = 0;
    double leg_length               = 0.12;
};
class DesireSet {
public:
    static DesireSet* GetInstance() {
        static auto instance = new DesireSet();
        return instance;
    }
    static constexpr double spinning_velocity = -7.0;
    static constexpr double x_velocity_scale  = 2.5;
    void update() {
        using namespace device;

        do {
            if ((RC_->switch_left == RC_Switch::UNKNOWN || RC_->switch_right == RC_Switch::UNKNOWN)
                || ((RC_->switch_left == RC_Switch::DOWN)
                    && (RC_->switch_right == RC_Switch::DOWN))) {
                reset_all_controls();
                SuperCap_ON_ = false;
                *mode_       = chassis_mode::stop;
                break;
            }

            balanceless_mode_ |= RC_->keyboard.r;
            balanceless_mode_ &= !(RC_->keyboard.ctrl && RC_->keyboard.r);
            if (((RC_->switch_left == RC_Switch::MIDDLE) && (RC_->switch_right == RC_Switch::DOWN))
                || balanceless_mode_) {
                reset_all_controls();
                *mode_ = chassis_mode::balanceless;
            } else if (RC_->switch_right == RC_Switch::MIDDLE) {
                if (!last_keyboard_.c && RC_->keyboard.c) {
                    *mode_ =
                        *mode_ == chassis_mode::spin ? chassis_mode::follow : chassis_mode::spin;
                } else if (RC_->keyboard.a || RC_->keyboard.d) {
                    if (*mode_ != chassis_mode::sideways_L && *mode_ != chassis_mode::sideways_R)
                        *mode_ =
                            RC_->keyboard.a ? chassis_mode::sideways_L : chassis_mode::sideways_R;
                } else if (
                    RC_->keyboard.w || RC_->keyboard.s
                    || (last_switch_right != RC_Switch::MIDDLE)) {
                    *mode_ = chassis_mode::follow;
                }
            } else if (RC_->switch_right == RC_Switch::UP) {
                *mode_ = chassis_mode::spin;
            } else {
                reset_all_controls();
            }

            auto keyboard_xmove = 1.0 * RC_->keyboard.w - RC_->keyboard.s;
            auto keyboard_ymove = 0.5 * RC_->keyboard.a - RC_->keyboard.d;
            auto keyboard_zmove = 0.5 * RC_->keyboard.q - RC_->keyboard.e;
            switch (*mode_) {
            case chassis_mode::follow:
                if (RC_->keyboard.ctrl && (RC_->keyboard.w || RC_->keyboard.s)) {
                    set_states_desire(RC_->joystick_right.x() + keyboard_xmove * 0.35);
                } else {
                    set_states_desire(RC_->joystick_right.x() + keyboard_xmove);
                }
                set_length_desire(RC_->joystick_right.y() + keyboard_zmove);
                break;
            case chassis_mode::sideways_L:
            case chassis_mode::sideways_R:
                if (RC_->keyboard.ctrl && (RC_->keyboard.a || RC_->keyboard.d)) {
                    set_states_desire(RC_->joystick_right.y() + keyboard_ymove * 0.35);
                } else {
                    set_states_desire(RC_->joystick_right.y() + keyboard_ymove);
                }
                set_length_desire(RC_->joystick_right.x() + keyboard_zmove);
                break;
            case chassis_mode::spin: set_states_desire(0, spinning_velocity); break;
            case chassis_mode::balanceless:
                set_states_desire(
                    RC_->joystick_right.x() + keyboard_xmove,
                    RC_->joystick_right.y() + keyboard_ymove);
                break;
            default: break;
            }

            if (RC_->keyboard.f) {
                desires.leg_length = 0.18;
            } else if (RC_->keyboard.g) {
                desires.leg_length = 0.25;
            }
            SuperCap_ON_ = RC_->keyboard.shift;

        } while (false);
        last_switch_right = RC_->switch_right;
        last_keyboard_    = RC_->keyboard;
    }
    void Init(
        module::IMU_output_vector* IMU_output, device::RC_status* RC, device::DjiMotor* GM6020_yaw,
        chassis_mode* mode) {
        IMU_data_   = IMU_output;
        RC_         = RC;
        GM6020_yaw_ = GM6020_yaw;
        mode_       = mode;
    }
    void CanLost() { reset_all_controls(); }

    desire desires    = {};
    bool SuperCap_ON_ = false;

private:
    DesireSet()                                            = default; // 禁止外部构造
    ~DesireSet()                                           = default; // 禁止外部析构
    DesireSet(const DesireSet& DesireSet)                  = delete;  // 禁止外部拷贝构造
    const DesireSet& operator=(const DesireSet& DesireSet) = delete;  // 禁止外部赋值操作

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // rclcpp::TimerBase::SharedPtr rc_watchdog_timer_;
    // InputInterface<msgs::chassis_power> robot_chassis_power_;

    device::DjiMotor* GM6020_yaw_              = nullptr;
    const device::RC_status* RC_               = nullptr;
    const module::IMU_output_vector* IMU_data_ = nullptr;
    chassis_mode* mode_                        = nullptr;

    device::RC_Keyboard last_keyboard_  = {};
    device::RC_Switch last_switch_right = {};
    bool balanceless_mode_              = false;

    void set_states_desire(double x_velocity, double rotation_velocity = 0.0) {
        auto x_d_ref              = x_velocity * x_velocity_scale;
        x_d_ref                   = std::clamp(x_d_ref, -x_velocity_scale, x_velocity_scale);
        static uint32_t last_time = 0;
        auto dt                   = DWT_GetDeltaT64_Expect(&last_time, app::dt);

        desires.xd(0, 0) += x_d_ref * dt; // distance
        if (*mode_ == chassis_mode::balanceless)
            desires.xd(1, 0) = x_d_ref;
        else
            desires.xd(1, 0) = 0;         // velocity

        auto gimbal_yaw_angle = GM6020_yaw_->get_angle();
        switch (*mode_) {                 // yaw
        case chassis_mode::follow:
        case chassis_mode::balanceless:
            desires.xd(2, 0) = IMU_data_->Yaw_multi_turn + (gimbal_yaw_angle - std::numbers::pi);
            break;
        case chassis_mode::sideways_L:
            desires.xd(2, 0) =
                IMU_data_->Yaw_multi_turn + (gimbal_yaw_angle - std::numbers::pi / 2.0);
            break;
        case chassis_mode::sideways_R:
            desires.xd(2, 0) =
                IMU_data_->Yaw_multi_turn + (gimbal_yaw_angle + std::numbers::pi / 2.0);
            break;
        case chassis_mode::spin: desires.xd(2, 0) += rotation_velocity * dt; break;
        default: break;
        }
        for (uint8_t i = 3; i < 10; i++) {
            desires.xd(i, 0) = 0;
        }
    }
    void set_length_desire(double length_speed) {
        constexpr double length_speed_scale = 0.00015;
        desires.leg_length += length_speed * length_speed_scale;
        desires.leg_length = std::clamp(desires.leg_length, 0.12, 0.27);
    }
    void set_roll_desire() {
        // constexpr double roll_scale = 0.5;
        // auto roll_control           = *thumb_wheel_ > 0.05 ? *thumb_wheel_ : 0.0;
        // *roldesire.leg_lentgh               = roll_control * roll_scale;
        desires.roll = 0.0;
    }
    void reset_all_controls() {
        desires.xd.setZero();
        desires.roll       = 0;
        desires.leg_length = 0.12;
        *mode_             = chassis_mode::stop;
        SuperCap_ON_       = false;
    }
};

} // namespace app::controller

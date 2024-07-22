#pragma once
#include "app/system_parameters.hpp"
#include "controller.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "module/DM8009/DM8009.hpp"
#include "tool/PID/PID.hpp"


namespace app::controller {
class SendProcess {
public:
    static SendProcess* GetInstance() {
        static auto instance = new SendProcess();
        return instance;
    }
    void Send() {
        switch (*mode_) {
            // motor_fixed();//原双下收腿
            // break;
        case chassis_mode::balanceless:
            motor_fixed(torque_->wheel_L, torque_->wheel_R); // in this case,torque means velocity
            break;
        case chassis_mode::stop:
        case chassis_mode::unknown:
        default:
            DM8009_sender_->write_command(DM8009_[leg_LF], torque_->leg_LF);
            DM8009_sender_->write_command(DM8009_[leg_LB], torque_->leg_LB);
            DM8009_sender_->write_command(DM8009_[leg_RF], torque_->leg_RF);
            DM8009_sender_->write_command(DM8009_[leg_RB], torque_->leg_RB);
            M3508_sender_->write_command(M3508_[wheel_L], torque_->wheel_L);
            M3508_sender_->write_command(M3508_[wheel_R], torque_->wheel_R);
            break;
        };
        M3508_sender_->send();
        DM8009_sender_->send_DM8009();
    }
    void Init(
        std::array<module::DM8009*, 4> DM8009, std::array<device::DjiMotor*, 2> M3508,
        device::DjiMotor_sender* DM8009_sender, device::DjiMotor_sender* m3508_sender,
        chassis_mode* mode) {
        DM8009_        = DM8009;
        M3508_         = M3508;
        DM8009_sender_ = DM8009_sender;
        M3508_sender_  = m3508_sender;
        mode_          = mode;
    }

private:
    SendProcess()                               = default;   // 禁止外部构造
    ~SendProcess()                              = default;   // 禁止外部析构
    SendProcess(const SendProcess& SendProcess) = delete;    // 禁止外部拷贝构造
    const SendProcess& operator=(const SendProcess& SendProcess) = delete; // 禁止外部赋值操作

    device::DjiMotor_sender* DM8009_sender_ = nullptr;
    device::DjiMotor_sender* M3508_sender_  = nullptr;
    std::array<module::DM8009*, 4> DM8009_  = {};
    std::array<device::DjiMotor*, 2> M3508_ = {};

    const chassis_mode* mode_     = nullptr;
    const control_torque* torque_ = &Controller::GetInstance()->control_torque_;
    tool::PID wheel_L_PID_        = tool::PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});
    tool::PID wheel_R_PID_        = tool::PID({0.6, 0.0, 0.0, 4.0, 0.0, 0.0, dt});

    inline void motor_fixed(double wheel_L_speed = 0, double wheel_R_speed = 0) {
        constexpr double angle_offset = 0.1 / 180.0 * std::numbers::pi;
        static_assert(angle_offset > 0 && angle_offset < module::DM8009::angle_offset);
        for (uint8_t i = 0; i < 4; ++i) {
            if (i == leg_LF || i == leg_RF)
                DM8009_[i]->SetAngle(std::numbers::pi + angle_offset);
            else
                DM8009_[i]->SetAngle(0 - angle_offset);
        }
        for (auto motor : DM8009_) {
            DM8009_sender_->write_command(motor, motor->get_calculated_torque());
        }
        M3508_sender_->write_command(
            M3508_[wheel_L], wheel_L_PID_.update(wheel_L_speed, M3508_[wheel_L]->get_velocity()));
        M3508_sender_->write_command(
            M3508_[wheel_R], wheel_R_PID_.update(wheel_R_speed, M3508_[wheel_R]->get_velocity()));
    }
};
} // namespace app::controller
#include "cmsis_os2.h"
#include "controller/desire_set.hpp"
#include "controller/send_process.hpp"
#include "device/Dji_motor/DJI_motor.hpp"
#include "device/RC/remote_control.hpp"
#include "device/super_cap/super_cap.hpp"
#include "module/DM8009/DM8009.hpp"
#include "module/IMU_EKF/IMU_EKF.hpp"
#include "module/can_comm/can_comm.hpp"
#include "module/referee/status.hpp"
#include "observer/observer.hpp"
#include "system_parameters.hpp"
#include "tool/time_counter.hpp"
#include <array>
// #include <vector>
using namespace device;
using namespace module;
namespace app {
static module::IMU* IMU_instance;
static device::remote_control* RC_instance;
static module::referee::Status* referee_instance;
static device::SuperCap* supercap_instance;
static module::CanComm* can_comm_instance;

static std::array<device::DjiMotor*, 2> M3508_instance;
static std::array<module::DM8009*, 4> DM8009_instance;
static device::DjiMotor* GM6020_yaw_instance;
static device::DjiMotor_sender* DM8009_sender_instance;
static device::DjiMotor_sender* m3508_sender_instance;

static auto observer_instance   = observer::observer::GetInstance();
static auto desire_instance     = controller::DesireSet::GetInstance();
static auto controller_instance = controller::Controller::GetInstance();
static auto sender_instance     = controller::SendProcess::GetInstance();
//lqr
static volatile double x_states_watch[10];
static volatile double x_desire_watch[10];
static volatile double wheel_speed_watch[2] = {};
static volatile double torque_watch[6] = {};
static volatile double motor_alive_watch[6] = {};
// 控制力矩
static std::array<double*, 6> torque_array = {
    &controller_instance->control_torque_.leg_LF,  &controller_instance->control_torque_.leg_LB,
    &controller_instance->control_torque_.leg_RB,  &controller_instance->control_torque_.leg_RF,
    &controller_instance->control_torque_.wheel_L, &controller_instance->control_torque_.wheel_R
};
//支持力解算&&离地检测
static std::array<volatile double, 3> support_watch = {};
static std::array<double*, 2> support_array = {
    &observer_instance->support_force_.L, &observer_instance->support_force_.R
};
static std::array<volatile bool, 3> levitate_watch = {};
static std::array<bool*, 3> levitate_array = {
    &observer_instance->status_levitate_, &observer_instance->status_levitate_L_, &observer_instance->status_levitate_R_
};
//超级电容
static std::array<volatile double, 3> supercap_voltage_watch = {};
static std::array<volatile double*, 3> supercap_voltage_array = {
    &supercap_instance->Info.chassis_power_, &supercap_instance->Info.chassis_voltage_, &supercap_instance->Info.supercap_voltage_
};
static volatile bool SuperCap_enable_watch;
static volatile uint16_t power_limit_watch;
// static volatile int dm8009_encoder_watch[4] = {};
void Init() {
    __disable_irq();
    IMU_instance = new module::IMU(
        IMU_params(this_board).set_angle_offset({0, 0, 0}).set_install_spin(IMU_spin_matrix));
    RC_instance       = new device::remote_control(RC_params(this_board));
    referee_instance  = module::referee::Status::GetInstance();
    supercap_instance = new device::SuperCap(SuperCap_params().set_can_instance(&hfdcan1));
    can_comm_instance = new module::CanComm(CanComm_params().set_can_instance(&hfdcan1));

    for (uint8_t i = 0; i < 2; ++i) {
        M3508_instance[i] = new device::DjiMotor(
            DjiMotor_params().set_can_instance(&hfdcan3).set_rx_id(M3508_ID::ID1 + i));
    }
    for (uint8_t i = 0; i < 4; ++i) {
        auto params = module::DM8009_params();
        params.Dji_common.set_can_instance(&hfdcan2).set_rx_id(DM8009_ID::ID1 + i);
        DM8009_instance[i] = new module::DM8009(params);
    }
    GM6020_yaw_instance = new device::DjiMotor(
        DjiMotor_params().set_can_instance(&hfdcan1).set_rx_id(GM6020_ID::ID1));

    m3508_sender_instance = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan3).set_tx_id(M3508_sendID::ID1));
    DM8009_sender_instance = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan2).set_tx_id(DM8009_sendID::ID1));

    auto M3508_config = device::DjiMotorConfig(device::DjiMotorType::M3508)
                            .enable_multi_turn_angle()
                            .set_reduction_ratio(13.0);
    M3508_instance[1]->configure(M3508_config);
    M3508_instance[0]->configure(M3508_config.reverse()); // 先正后反 顺序不要变

    DM8009_instance[0]->configure(5079, true);//6993 max
    DM8009_instance[1]->configure(7957, true);//6040 max 
    DM8009_instance[2]->configure(2892);//4665 max
    DM8009_instance[3]->configure(5863);//4019 max

    GM6020_yaw_instance->configure(
        device::DjiMotorConfig(device::DjiMotorType::GM6020).set_encoder_zero_point(7817));

    desire_instance->Init(
        &IMU_instance->output_vector, &RC_instance->data, GM6020_yaw_instance, DM8009_instance,
        M3508_instance, referee_instance, supercap_instance);
    controller_instance->Init(
        IMU_instance, DM8009_instance, M3508_instance, supercap_instance, referee_instance);
    observer_instance->Init(
        IMU_instance, DM8009_instance, M3508_instance, &controller_instance->u_mat);
    sender_instance->Init(
        DM8009_instance, M3508_instance, DM8009_sender_instance, m3508_sender_instance);
    __enable_irq();
}
static double dt_watch;
extern "C" void main_task_func(void* argument) {
    Init();
    while (true) {
        uint32_t wakeUpTime = osKernelGetTickCount();
        TimeElapse(dt_watch, [&]() {                      // 在release中删除
            desire_instance->update();
            observer_instance->update();
            controller_instance->update();
            sender_instance->Send();
        });
        for (uint8_t i = 0; i < 2; ++i) {
            wheel_speed_watch[i] = M3508_instance[i]->get_velocity();
            motor_alive_watch[i + 4]  = M3508_instance[i]->get_online_states();
            support_watch[i]          = *support_array[i];
        }
        for (uint8_t i = 0; i < 4; ++i) {
            motor_alive_watch[i] = DM8009_instance[i]->get_online_states();
        }
        for (size_t i = 0; i <6; ++i) {
            torque_watch[i] = *torque_array[i];
        }
        support_watch[2] = (support_watch[1]+support_watch[0])/2.0;
        for (uint8_t i = 0; i < 10; ++i) {
            x_states_watch[i] = observer_instance->x_states_[i];
            x_desire_watch[i] = desire_instance->desires.xd[i];
        }
        for (uint8_t i = 0; i < 3; ++i) {
            levitate_watch[i] = *levitate_array[i];
            supercap_voltage_watch[i] = *supercap_voltage_array[i];
        }
        SuperCap_enable_watch = supercap_instance->Info.enabled_;
        // power_limit_watch     = controller_instance.;
        // for (uint8_t i = 0; i < 4; ++i) {
        //     dm8009_encoder_watch[i] = DM8009_instance[i]->calibrate_zero_point();
        // }
        osDelayUntil(wakeUpTime + 1);
    }
}
extern "C" void comm_task_func(void* argument) {
    while (true) {
        ChassisStates data;
        data.leg_length_L = observer_instance->leg_length_.L;
        data.leg_length_R = observer_instance->leg_length_.R;
        data.mode         = chassis_mode_;
        data.chassis_relative_angle =
            desire_instance->desires.xd(2, 0) - observer_instance->x_states_(2, 0);
        can_comm_instance->Send(data);
        constexpr uint32_t refresh_rate = 1000 / 50;
        osDelay(refresh_rate);
    }
}
} // namespace app
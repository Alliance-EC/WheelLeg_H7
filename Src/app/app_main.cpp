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

static chassis_mode chassis_mode_;

static volatile double x_states_watch[10];
static volatile double x_desire_watch[10];
static volatile double wheel_speed_watch[2] = {};
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
            DjiMotor_params().set_can_instance(&hfdcan2).set_rx_id(toU32(M3508_ID::ID1) + i));
    }
    for (uint8_t i = 0; i < 4; ++i) {
        auto params = module::DM8009_params();
        params.Dji_common.set_can_instance(&hfdcan3).set_rx_id(toU32(DM8009_ID::ID1) + i);
        DM8009_instance[i] = new module::DM8009(params);
    }
    GM6020_yaw_instance = new device::DjiMotor(
        DjiMotor_params().set_can_instance(&hfdcan1).set_rx_id(toU32(GM6020_ID::ID1)));

    m3508_sender_instance = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan2).set_tx_id(toU32(M3508_sendID::ID1)));
    DM8009_sender_instance = new device::DjiMotor_sender(
        DjiMotor_params().set_can_instance(&hfdcan3).set_tx_id(toU32(DM8009_sendID::ID1)));

    auto M3508_config = device::DjiMotorConfig(device::DjiMotorType::M3508)
                            .enable_multi_turn_angle()
                            .set_reduction_ratio(268.0 / 17);
    M3508_instance[1]->configure(M3508_config);
    M3508_instance[0]->configure(M3508_config.reverse()); // 顺序不要变

    DM8009_instance[0]->configure(6466, true);
    DM8009_instance[1]->configure(7954, true);
    DM8009_instance[2]->configure(1781);
    DM8009_instance[3]->configure(149);

    GM6020_yaw_instance->configure(
        device::DjiMotorConfig(device::DjiMotorType::GM6020).set_encoder_zero_point(7817));

    desire_instance->Init(
        &IMU_instance->output_vector, &RC_instance->data, GM6020_yaw_instance, DM8009_instance,
        M3508_instance, &chassis_mode_, referee_instance, supercap_instance);
    controller_instance->Init(
        IMU_instance, DM8009_instance, M3508_instance, &chassis_mode_, supercap_instance,
        referee_instance);
    observer_instance->Init(
        IMU_instance, DM8009_instance, M3508_instance, &chassis_mode_, &controller_instance->u_mat);
    sender_instance->Init(
        DM8009_instance, M3508_instance, DM8009_sender_instance, m3508_sender_instance,
        &chassis_mode_);
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

        for (uint8_t i = 0; i < 10; ++i) {
            x_states_watch[i] = observer_instance->x_states_[i];
            x_desire_watch[i] = desire_instance->desires.xd[i];
        }
        for (uint8_t i = 0; i < 2; ++i) {
            wheel_speed_watch[i] = M3508_instance[i]->get_velocity();
        }
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
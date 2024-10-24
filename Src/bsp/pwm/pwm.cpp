#include "pwm.hpp"
#include <unordered_map>

namespace bsp {
std::unordered_map<std::tuple<TIM_HandleTypeDef*, uint32_t>, pwm*, tool::TupleHash>
    pwm::pwm_instances_ = [] {
        std::unordered_map<std::tuple<TIM_HandleTypeDef*, uint32_t>, pwm*, tool::TupleHash> map;
        map.reserve(3);
        return map;
    }();
} // namespace bsp
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    bsp::pwm* instance = bsp::pwm::get_instance(htim, htim->Channel);
    if (instance) {
        instance->OnPulseFinishedCallback();
    }
}
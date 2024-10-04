#include "gpio.hpp"

namespace bsp {
std::unordered_map<uint16_t, gpio*> gpio::gpio_instances_= {};
} // namespace bsp
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    bsp::gpio* instance = bsp::gpio::get_instance(GPIO_Pin);
    if (instance) {
        instance->OnEXTICallback();
    }
}

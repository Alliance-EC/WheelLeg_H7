#pragma once

#include <cstdint>
namespace device {
template <typename T>
constexpr T operator+(T id, uint32_t value) {
    return static_cast<T>(static_cast<uint32_t>(id) + value);
}
enum class M3508_ID : uint16_t {
    ID1 = 0x201,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class DM8009_ID : uint16_t {
    ID1 = 0x301,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class GM6020_ID : uint16_t {
    ID1 = 0x205,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
};
enum class M3508_sendID : uint16_t {
    ID1 = 0x200,
    ID2 = 0x1FF,
};
enum class GM6020_sendID : uint16_t {
    ID1 = 0x1FF,
    ID2 = 0x2FF,
};
enum class DM8009_sendID : uint16_t {
    ID1 = 0x3FE,
    ID2 = 0x4FE,
};
enum class DjiMotorType : uint8_t {
    UNKNOWN        = 0,
    GM6020         = 1,
    GM6020_VOLTAGE = 2,
    M3508          = 3,
    M2006          = 4,
    DM8009         = 5,
};
} // namespace device
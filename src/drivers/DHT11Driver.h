#pragma once

#include "periph/PeriphBase.h"
#include "periph/TIM.h"

class DHT11Driver
{
public:
    enum class ErrorCode
    {
        Success,
        Timeout,
        InvalidChecksum,
    };

    // TODO: use the same pin
    DHT11Driver(Pin input_pin, Pin output_pin, TIM_TypeDef *timer);

    ErrorCode run(float &temperature, float &humidity);

private:
    Pin input_pin_{};
    Pin output_pin_{};
    TIM_TypeDef *timer_{};
};

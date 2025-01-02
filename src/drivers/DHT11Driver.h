#pragma once

#include "periph/PeriphBase.h"
#include "periph/TIM.h"
#include "utils/FixedBitset.h"

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
    void exti_handler();
    void tim_handler();

private:
    FixedBitset<5 * 8> dht_data;

    Pin input_pin_{};
    Pin output_pin_{};
    TIM_TypeDef *timer_{};

    volatile bool listening = false;
    volatile int num_height = 0;
    volatile int num_written_bits = 0;
};

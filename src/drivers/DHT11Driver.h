#pragma once

#include "periph/PeriphBase.h"
#include "periph/TIM.h"
#include "utils/FixedBitset.h"

#include <atomic>

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
    void cleanup();

    void exti_handler();
    void tim_handler();

private:
    FixedBitset<5 * 8> dht_data;

    Pin input_pin_{};
    Pin output_pin_{};
    TIM_TypeDef *timer_{};

    InterruptType exti_interrupt_type_{};
    InterruptType tim_interrupt_type_{};

    std::atomic<bool> listening = false;
    std::atomic<uint32_t> num_height = 0;
    std::atomic<uint32_t> num_written_bits = 0;
};

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

    DHT11Driver(Pin input_pin, TIM_TypeDef *timer);
    ~DHT11Driver();

    ErrorCode run(float &temperature, float &humidity);

private:
    void cleanup();
    void stop();

    void exti_handler();
    void tim_handler();

private:
    static constexpr int NUM_DATA_BITS = 40;

private:
    FixedBitset<NUM_DATA_BITS> dht_data_;

    Pin pin_{};
    TIM_TypeDef *timer_{};

    InterruptType exti_interrupt_type_{};
    InterruptType tim_interrupt_type_{};

    std::atomic<uint32_t> num_rising_edges_ = 0;
    std::atomic<uint32_t> num_written_bits_ = 0;
};

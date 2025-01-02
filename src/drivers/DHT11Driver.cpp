#include "DHT11Driver.h"

#include "Sleep.h"
#include "core/Globals.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/IRQ.h"

namespace
{

void sleep1()
{
    volatile uint32_t t = 4;
    while (--t)
    {}
}


} // namespace

DHT11Driver::DHT11Driver(Pin input_pin, Pin output_pin, TIM_TypeDef *timer)
    : input_pin_(input_pin)
    , output_pin_(output_pin)
    , timer_(timer)
{}

DHT11Driver::ErrorCode DHT11Driver::run(float &temperature, float &humidity)
{
    // Edge detection
    constexpr GPIOPort edge_detection_port = GPIOPort::B;
    constexpr int edge_detection_pin = 5;
    gpio::setPinMode(edge_detection_port, edge_detection_pin, gpio::PinMode::InputFloating);
    exti::setupEXTI(edge_detection_port, edge_detection_pin, exti::TriggerMode::RisingEdges, exti::ENABLE_INTERRUPT);
    const InterruptType exti_interrupt = exti::getInterruptType(edge_detection_pin);
    irq::enableInterrupt(exti_interrupt);

    irq::setHandler(
        exti_interrupt,
        [](void *opaque) {
            auto *self = (DHT11Driver *)opaque;
            self->exti_handler();
        },
        this);

    irq::setHandler(
        InterruptType::TIM2IRQ,
        [](void *opaque) {
            auto *self = (DHT11Driver *)opaque;
            self->tim_handler();
        },
        this);


    num_written_bits = 0;
    gpio::setPinOutput(GPIOPort::C, 13, false);

    gpio::setPinOutput(GPIOPort::B, 12, false);
    utils::sleepMsec(20);

    listening = true;
    num_height = 0;
    gpio::setPinOutput(GPIOPort::B, 12, true);
    utils::sleepMsec(10);
    listening = false;

    gpio::setPinOutput(GPIOPort::C, 13, false);

    const uint32_t start_time_ms = glob::total_msec;
    constexpr uint32_t TIMEOUT_MS = 100;
    const uint32_t end_time = start_time_ms + TIMEOUT_MS;

    while (num_written_bits != 40)
    {
        if (glob::total_msec > end_time)
        {
            return ErrorCode::Timeout;
        }
    }

    const auto flip_byte = [](uint8_t byte) {
        uint8_t result = 0;
        for (int i = 0; i < 8; ++i)
        {
            result |= ((byte >> i) & 1) << (7 - i);
        }
        return result;
    };


    uint8_t data[5];
    dht_data.copyData<uint8_t>(data, 5);
    for (int i = 0; i < 5; ++i)
    {
        data[i] = flip_byte(data[i]);
    }

    const uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4])
    {
        return ErrorCode::InvalidChecksum;
    }

    humidity = (float)data[0] + (float)data[1] / 256.0f;
    temperature = (float)data[2] + (float)data[3] / 256.0f;

    return ErrorCode::Success;
}

void DHT11Driver::exti_handler()
{
    if (EXTI->PR & EXTI_PR_PR5)
    {
        if (listening)
        {
            ++num_height;
            if (num_height >= 3)
            {
                tim::restartTimer(TIM2);
            }
        }
        EXTI->PR = EXTI_PR_PR5;
    }
}

void DHT11Driver::tim_handler()
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        if (num_height >= 3 && num_written_bits < 40)
        {
            gpio::setPinOutput(GPIOPort::C, 13, true);
            sleep1();
            gpio::setPinOutput(GPIOPort::C, 13, false);
            const bool bit = gpio::getPinInput(GPIOPort::B, 5);
            dht_data.set(num_written_bits, bit);
            ++num_written_bits;
        }
    }
}

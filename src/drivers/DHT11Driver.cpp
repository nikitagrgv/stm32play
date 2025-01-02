#include "DHT11Driver.h"

#include "Sleep.h"
#include "core/Globals.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/IRQ.h"


DHT11Driver::DHT11Driver(Pin input_pin, Pin output_pin, TIM_TypeDef *timer)
    : input_pin_(input_pin)
    , output_pin_(output_pin)
    , timer_(timer)
{
    exti_interrupt_type_ = exti::getInterruptType(input_pin_.num);
    tim_interrupt_type_ = tim::getUpdateInterruptType(timer_);
}

DHT11Driver::ErrorCode DHT11Driver::run(float &temperature, float &humidity)
{
    cleanup();

    gpio::setPinMode(input_pin_, gpio::PinMode::InputFloating);
    exti::setupEXTI(input_pin_, exti::TriggerMode::RisingEdges, exti::ENABLE_INTERRUPT);
    irq::enableInterrupt(exti_interrupt_type_);

    num_written_bits = 0;
    gpio::setPinOutput(output_pin_, false);
    utils::sleepMsec(20);

    listening = true;

    irq::setHandler(
        exti_interrupt_type_,
        [](void *opaque) {
            auto *self = (DHT11Driver *)opaque;
            self->exti_handler();
        },
        this);

    irq::setHandler(
        tim_interrupt_type_,
        [](void *opaque) {
            auto *self = (DHT11Driver *)opaque;
            self->tim_handler();
        },
        this);

    num_height = 0;
    gpio::setPinOutput(output_pin_, true);
    utils::sleepMsec(10);
    listening = false;

    const uint32_t start_time_ms = glob::total_msec;
    constexpr uint32_t TIMEOUT_MS = 100;
    const uint32_t end_time = start_time_ms + TIMEOUT_MS;

    while (num_written_bits != 40)
    {
        if (glob::total_msec > end_time)
        {
            cleanup();
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
        cleanup();
        return ErrorCode::InvalidChecksum;
    }

    humidity = (float)data[0] + (float)data[1] / 256.0f;
    temperature = (float)data[2] + (float)data[3] / 256.0f;

    cleanup();
    return ErrorCode::Success;
}

void DHT11Driver::cleanup()
{
    dht_data.clearAll();
    listening = false;
    num_height = 0;
    num_written_bits = 0;

    tim::stopTimer(timer_);
    exti::disableEXTI(input_pin_.num);

    irq::clearHandler(exti_interrupt_type_);
    irq::clearHandler(tim_interrupt_type_);
}

void DHT11Driver::exti_handler()
{
    if (!exti::checkPendingAndClear(input_pin_.num))
    {
        return;
    }

    if (!listening)
    {
        return;
    }

    ++num_height;
    if (num_height >= 3)
    {
        tim::restartTimer(timer_);
    }
}

void DHT11Driver::tim_handler()
{
    if (!tim::checkPendingUpdateAndClear(timer_))
    {
        return;
    }

    if (num_height < 3 || num_written_bits >= 40)
    {
        return;
    }

    const bool bit = gpio::getPinInput(input_pin_);
    dht_data.set(num_written_bits, bit);
    ++num_written_bits;
}

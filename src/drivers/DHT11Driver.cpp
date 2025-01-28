#include "DHT11Driver.h"

#include "Sleep.h"
#include "core/Globals.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/IRQ.h"
#include "utils/BitUtils.h"


DHT11Driver::DHT11Driver(Pin input_pin, TIM_TypeDef *timer)
    : pin_(input_pin)
    , timer_(timer)
{
    exti_interrupt_type_ = exti::getInterruptType(pin_.num);
    tim_interrupt_type_ = tim::getUpdateInterruptType(timer_);
}

DHT11Driver::~DHT11Driver()
{
    cleanup();
}

DHT11Driver::ErrorCode DHT11Driver::run(float &temperature, float &humidity)
{
    cleanup();

    irq::disableInterrupt(exti_interrupt_type_);
    irq::disableInterrupt(tim_interrupt_type_);

    gpio::disablePin(pin_);
    gpio::setPinOutput(pin_, false);
    gpio::configureOutput(pin_, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);

    utils::sleepMsec(timer_, 20);

    constexpr uint32_t frequency = 1'000'000;
    constexpr uint32_t reload_value = 48;
    tim::setupTimer(timer_, frequency, reload_value, tim::SINGLE_SHOT | tim::ENABLE_UPDATE_INTERRUPT);
    irq::setHandlerMethod<&DHT11Driver::tim_handler>(tim_interrupt_type_, this);
    irq::enableInterrupt(tim_interrupt_type_);

    exti::setupEXTI(pin_, exti::TriggerMode::RisingEdges, exti::ENABLE_INTERRUPT);
    irq::setHandlerMethod<&DHT11Driver::exti_handler>(exti_interrupt_type_, this);
    irq::enableInterrupt(exti_interrupt_type_);

    gpio::setPinOutput(pin_, true);
    gpio::configureInput(pin_);

    const uint32_t start_time_ms = glob::total_msec;
    const uint32_t end_time = start_time_ms + TIMEOUT_MS;

    SCOPE_EXIT([&] { cleanup(); });

    while (num_written_bits_ < NUM_DATA_BITS)
    {
        if (glob::total_msec > end_time)
        {
            return ErrorCode::Timeout;
        }
    }

    uint8_t data[NUM_DATA_BITS / 8];
    dht_data_.copyData<uint8_t>(data, NUM_DATA_BITS / 8);
    for (uint8_t &b : data)
    {
        b = utils::flipByte(b);
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

void DHT11Driver::cleanup()
{
    stop();

    dht_data_.clearAll();
    num_rising_edges_ = 0;
    num_written_bits_ = 0;
}

void DHT11Driver::stop()
{
    tim::stopTimer(timer_);
    exti::disableEXTI(pin_.num);

    irq::clearHandler(exti_interrupt_type_);
    irq::clearHandler(tim_interrupt_type_);
}

void DHT11Driver::exti_handler()
{
    if (!exti::checkPendingAndClear(pin_.num))
    {
        return;
    }

    ++num_rising_edges_;
    if (num_rising_edges_ >= 3 && num_written_bits_ < NUM_DATA_BITS)
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

    if (num_rising_edges_ < 2 || num_written_bits_ >= NUM_DATA_BITS)
    {
        return;
    }

    const bool bit = gpio::getPinInput(pin_);
    dht_data_.set(num_written_bits_, bit);
    ++num_written_bits_;

    if (num_written_bits_ >= NUM_DATA_BITS)
    {
        stop();
    }
}

#include "LCD1602Driver.h"

#include "Sleep.h"
#include "periph/I2C.h"


LCD1602Driver::LCD1602Driver(I2C i2c, TIM_TypeDef *timer)
    : i2c_(i2c)
    , timer_(timer)
{}

bool LCD1602Driver::initialize()
{
    i2c::masterTransmitBlocking(i2c_, ADDRESS, 0x00);

    constexpr uint8_t BACKLIGHT_BIT = 1 << 3;

    utils::sleepMsec(100);
    trigger(0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);
    trigger(0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);
    trigger(0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);

    trigger(0b10'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);

    run_command(0b101000, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    run_command(0b1000, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    run_command(0b1, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    run_command(0b110, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);

    run_command(0b1111, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);

    // uint8_t receive_data[6] = {0, 0, 0, 0, 0, 0};
    // masterReceive(i2c, sht31_address, receive_data, 6);
    return true;
}

bool LCD1602Driver::print(char ch)
{
    return run_command(ch, RWMode::Write, RSMode::Data);
}

bool LCD1602Driver::print(const char *str)
{
    while (*str)
    {
        print(*str);
        ++str;
    }
    return true;
}

bool LCD1602Driver::trigger(uint8_t data)
{
    utils::sleepUsec(timer_, DELAY_US);

    constexpr uint8_t enable_mask = 1 << 2;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

    if (!i2c::masterTransmitBlocking(i2c_, ADDRESS, data | enable_mask))
    {
        return false;
    }

    utils::sleepUsec(timer_, DELAY_US);

    if (i2c::masterTransmitBlocking(i2c_, ADDRESS, data & enable_clear_mask))
    {
        return false;
    }

    return true;
}

bool LCD1602Driver::run_command(uint8_t data, RWMode rw, RSMode rs)
{
    uint8_t base_mask = 0;
    base_mask |= (uint8_t)rw << 1;
    base_mask |= (uint8_t)rs << 0;
    base_mask |= (uint8_t)backlight_ << 3;

    const uint8_t data_mask_high = data & 0xF0;
    const uint8_t data_mask_low = (data << 4) & 0xF0;

    const uint8_t high = data_mask_high | base_mask;
    const uint8_t low = data_mask_low | base_mask;

    if (!trigger(high))
    {
        return false;
    }
    if (!trigger(low))
    {
        return false;
    }
    return true;
}

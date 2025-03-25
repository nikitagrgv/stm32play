#include "LCD1602Driver.h"

#include "Sleep.h"
#include "periph/I2C.h"

namespace
{

constexpr uint32_t REGISTER_SELECT_BIT_POS = 0;
constexpr uint32_t READ_WRITE_BIT_POS = 1;
constexpr uint32_t ENABLE_BIT_POS = 2;
constexpr uint32_t BACKLIGHT_BIT_POS = 3;

} // namespace

LCD1602Driver::LCD1602Driver(I2C i2c, TIM_TypeDef *timer)
    : i2c_(i2c)
    , timer_(timer)
{}

bool LCD1602Driver::initialize()
{
    if (!put_data(0x00))
    {
        return false;
    }

    // Setup 4-bit mode
    utils::sleepMsec(15);
    trigger(0b11'0000);
    utils::sleepMsec(5);
    trigger(0b11'0000);
    utils::sleepMsec(1);
    trigger(0b11'0000);
    utils::sleepMsec(1);
    trigger(0b10'0000);
    utils::sleepMsec(1);

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

bool LCD1602Driver::returnHome()
{
    if (!trigger(0b10'0000))
    {
        return false;
    }
    utils::sleepMsec(2);
    return true;
}

bool LCD1602Driver::put_data(uint8_t data)
{
    utils::sleepUsec(timer_, DELAY_US);
    return i2c::masterTransmitBlocking(i2c_, ADDRESS, data, TIMEOUT_MS);
}

bool LCD1602Driver::trigger(uint8_t data)
{
    constexpr uint8_t enable_mask = 1 << ENABLE_BIT_POS;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

    if (backlight_)
    {
        data |= 1 << BACKLIGHT_BIT_POS;
    }

    if (!put_data(data | enable_mask))
    {
        return false;
    }

    if (!put_data(data & enable_clear_mask))
    {
        return false;
    }

    return true;
}

bool LCD1602Driver::run_command(uint8_t data, RWMode rw, RSMode rs)
{
    uint8_t base_mask = 0;
    base_mask |= (uint8_t)rw << READ_WRITE_BIT_POS;
    base_mask |= (uint8_t)rs << REGISTER_SELECT_BIT_POS;

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

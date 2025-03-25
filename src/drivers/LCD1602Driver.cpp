#include "LCD1602Driver.h"

#include "Sleep.h"
#include "periph/I2C.h"

#include <ctime>

namespace
{

constexpr uint8_t REGISTER_SELECT_BIT_POS = 0;
constexpr uint8_t READ_WRITE_BIT_POS = 1;
constexpr uint8_t ENABLE_BIT_POS = 2;
constexpr uint8_t BACKLIGHT_BIT_POS = 3;

constexpr uint8_t NUM_LINES_BIT_POS = 3;
constexpr uint8_t FONT_BIT_POS = 2;

constexpr uint8_t DISPLAY_ON_BIT_POS = 2;
constexpr uint8_t CURSOR_ON_BIT_POS = 1;
constexpr uint8_t CURSOR_BLINKING_ON_BIT_POS = 0;

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

    // Initialization sequence
    utils::sleepMsec(15);
    trigger(0b11'0000);
    utils::sleepMsec(5);
    trigger(0b11'0000);
    utils::sleepMsec(1);
    trigger(0b11'0000);
    utils::sleepMsec(1);
    // Set 4-bit mode
    trigger(0b10'0000);
    utils::sleepMsec(1);

    update_function_set();
    update_display_control();
    run_command(0b1, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    run_command(0b110, RWMode::Write, RSMode::Command);
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

bool LCD1602Driver::setBacklightEnabled(bool backlight)
{
    if (backlight_ == backlight)
    {
        return true;
    }
    backlight_ = backlight;
    return update_backlight();
}

bool LCD1602Driver::setLinesMode(LinesMode lines_mode)
{
    if (lines_mode_ == lines_mode)
    {
        return true;
    }
    lines_mode_ = lines_mode;
    return update_function_set();
}

bool LCD1602Driver::setFont(Font font)
{
    if (font_ == font)
    {
        return true;
    }
    font_ = font;
    return update_function_set();
}

bool LCD1602Driver::setDisplayEnabled(bool enabled)
{
    if (display_enabled_ == enabled)
    {
        return true;
    }
    display_enabled_ = enabled;
    return update_display_control();
}

bool LCD1602Driver::setCursorEnabled(bool enabled)
{
    if (cursor_enabled_ == enabled)
    {
        return true;
    }
    cursor_enabled_ = enabled;
    return update_display_control();
}

bool LCD1602Driver::setCursorBlinkingEnabled(bool enabled)
{
    if (cursor_blinking_enabled_ == enabled)
    {
        return true;
    }
    cursor_blinking_enabled_ = enabled;
    return update_display_control();
}

bool LCD1602Driver::returnHome()
{
    if (!run_command(0b10, RWMode::Write, RSMode::Command))
    {
        return false;
    }
    short_delay();
    return true;
}

bool LCD1602Driver::clearDisplay()
{
    if (!run_command(0b1, RWMode::Write, RSMode::Command))
    {
        return false;
    }
    short_delay();
    return true;
}

bool LCD1602Driver::put_data(uint8_t data)
{
    if (backlight_)
    {
        data |= 1 << BACKLIGHT_BIT_POS;
    }
    short_delay();
    return i2c::masterTransmitBlocking(i2c_, ADDRESS, data, TIMEOUT_MS);
}

bool LCD1602Driver::trigger(uint8_t data)
{
    constexpr uint8_t enable_mask = 1 << ENABLE_BIT_POS;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

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

void LCD1602Driver::short_delay()
{
    utils::sleepUsec(timer_, DELAY_US);
}

bool LCD1602Driver::update_backlight()
{
    return put_data(0xFF);
}

bool LCD1602Driver::update_function_set()
{
    uint8_t data = 1 << 5;
    data |= (uint8_t)font_ << FONT_BIT_POS;
    data |= (uint8_t)lines_mode_ << NUM_LINES_BIT_POS;
    if (!run_command(data, RWMode::Write, RSMode::Command))
    {
        return false;
    }
    short_delay();
    return true;
}

bool LCD1602Driver::update_display_control()
{
    uint8_t data = 1 << 3;
    data |= (uint8_t)display_enabled_ << DISPLAY_ON_BIT_POS;
    data |= (uint8_t)cursor_enabled_ << CURSOR_ON_BIT_POS;
    data |= (uint8_t)cursor_blinking_enabled_ << CURSOR_BLINKING_ON_BIT_POS;
    if (!run_command(data, RWMode::Write, RSMode::Command))
    {
        return false;
    }
    short_delay();
    return true;
}

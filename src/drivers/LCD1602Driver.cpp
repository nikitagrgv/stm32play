#include "LCD1602Driver.h"


LCD1602Driver::LCD1602Driver(I2C i2c, TIM_TypeDef *timer)
    : i2c_(i2c)
    , timer_(timer)
{}

bool LCD1602Driver::initialize()
{
    constexpr uint8_t address = 0x27;

    i2c::masterTransmitBlocking(i2c, address, 0x00);

    constexpr uint8_t BACKLIGHT_BIT = 1 << 3;

    utils::sleepMsec(100);
    triggerLcd(i2c, address, 0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);
    triggerLcd(i2c, address, 0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);
    triggerLcd(i2c, address, 0b11'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);

    triggerLcd(i2c, address, 0b10'0000 | BACKLIGHT_BIT);
    utils::sleepMsec(20);

    runLcdCommand(i2c, address, 0b101000, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    runLcdCommand(i2c, address, 0b1000, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    runLcdCommand(i2c, address, 0b1, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);
    runLcdCommand(i2c, address, 0b110, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);

    runLcdCommand(i2c, address, 0b1111, RWMode::Write, RSMode::Command);
    utils::sleepMsec(20);

    runLcdCommand(i2c, address, 'f', RWMode::Write, RSMode::Data);

    // uint8_t receive_data[6] = {0, 0, 0, 0, 0, 0};
    // masterReceive(i2c, sht31_address, receive_data, 6);
    return true;
}

bool LCD1602Driver::trigger(I2C i2c, uint8_t address, uint8_t data)
{
    utils::sleepUsec(TIM2, FAST_DELAY_US);

    constexpr uint8_t enable_mask = 1 << 2;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

    i2c::masterTransmitBlocking(i2c, address, data | enable_mask);
    utils::sleepUsec(TIM2, FAST_DELAY_US);
    i2c::masterTransmitBlocking(i2c, address, data & enable_clear_mask);
}

bool LCD1602Driver::run_command(I2C i2c, uint8_t address, uint8_t data, RWMode rw, RSMode rs, bool backlight)
{
    uint8_t base_mask = 0;
    base_mask |= (uint8_t)rw << 1;
    base_mask |= (uint8_t)rs << 0;
    base_mask |= (uint8_t)backlight << 3;

    const uint8_t data_mask_high = data & 0xF0;
    const uint8_t data_mask_low = (data << 4) & 0xF0;

    const uint8_t high = data_mask_high | base_mask;
    const uint8_t low = data_mask_low | base_mask;

    triggerLcd(i2c, address, high);
    triggerLcd(i2c, address, low);
}

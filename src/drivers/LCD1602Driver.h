#pragma once

#include "DeviceCMSIS.h"
#include "periph/PeriphBase.h"

class LCD1602Driver
{
public:
    LCD1602Driver(I2C i2c, TIM_TypeDef *timer);

    bool initialize();

    bool print(char ch);
    bool print(const char *str);

private:
    enum class RWMode : uint8_t
    {
        Write = 0,
        Read = 1
    };

    enum class RSMode : uint8_t
    {
        Command = 0,
        Data = 1
    };

    constexpr int DELAY_US = 80;
    constexpr uint8_t ADDRESS = 0x27;

private:
    bool trigger(uint8_t data);
    bool run_command(uint8_t data, RWMode rw, RSMode rs, bool backlight = true);

private:
    I2C i2c_;
    TIM_TypeDef *timer_;
};
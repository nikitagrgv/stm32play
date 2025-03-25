#pragma once

#include "DeviceCMSIS.h"
#include "periph/PeriphBase.h"

class LCD1602Driver
{
public:
    enum class LinesMode : uint8_t
    {
        One = 0,
        Two = 1,
    };

    enum class Font
    {
        Font5x8 = 0,
        Font5x10 = 1,
    };

public:
    LCD1602Driver(I2C i2c, TIM_TypeDef *timer);

    bool initialize();

    bool print(char ch);
    bool print(const char *str);

    bool isBacklightEnabled() const { return backlight_; }
    void setBacklightEnabled(bool backlight);

    LinesMode getLinesMode() const { return lines_mode_; }
    void setLinesMode(LinesMode lines_mode);

    Font getFont() const { return font_; }
    void setFont(Font font);

    bool returnHome();

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

    static constexpr int TIMEOUT_MS = 100;
    static constexpr int DELAY_US = 80;
    static constexpr uint8_t ADDRESS = 0x27;

private:
    bool put_data(uint8_t data);
    bool trigger(uint8_t data);
    bool run_command(uint8_t data, RWMode rw, RSMode rs);

    void short_delay();

    void update_backlight();
    void update_display_control();

private:
    bool backlight_ = true;
    LinesMode lines_mode_ = LinesMode::Two;
    Font font_ = Font::Font5x8;

    I2C i2c_;
    TIM_TypeDef *timer_;
};
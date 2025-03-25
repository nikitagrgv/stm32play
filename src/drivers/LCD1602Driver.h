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

    enum class MoveDirection
    {
        Decrement = 0,
        Increment = 1,
    };

public:
    LCD1602Driver(I2C i2c, TIM_TypeDef *timer);

    bool initialize();

    bool print(char ch);
    bool print(const char *str);

    bool isBacklightEnabled() const { return backlight_; }
    bool setBacklightEnabled(bool backlight);

    LinesMode getLinesMode() const { return lines_mode_; }
    bool setLinesMode(LinesMode lines_mode);

    Font getFont() const { return font_; }
    bool setFont(Font font);

    bool isDisplayEnabled() const { return display_enabled_; }
    bool setDisplayEnabled(bool enabled);

    bool isCursorEnabled() const { return cursor_enabled_; }
    bool setCursorEnabled(bool enabled);

    bool isCursorBlinkingEnabled() const { return cursor_blinking_enabled_; }
    bool setCursorBlinkingEnabled(bool enabled);

    bool returnHome();
    bool clearDisplay();

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

    bool update_backlight();
    bool update_function_set();
    bool update_display_control();

private:
    bool backlight_ = true;

    LinesMode lines_mode_ = LinesMode::Two;
    Font font_ = Font::Font5x8;

    bool display_enabled_{true};
    bool cursor_enabled_{true};
    bool cursor_blinking_enabled_{true};

    MoveDirection cursor_direction_{MoveDirection::Increment};
    bool display_shifting_enabled_{false};

    I2C i2c_;
    TIM_TypeDef *timer_;
};
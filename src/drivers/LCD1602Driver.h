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
        Left = 0,
        Right = 1,
    };

    static constexpr uint8_t SCREEN_WIDTH = 40;

public:
    LCD1602Driver(I2C i2c, TIM_TypeDef *timer);

    bool initialize();

    bool goHome();
    bool clear();

    bool shiftCursorLeft(int distance = 1);
    bool shiftCursorRight(int distance = 1);

    bool shiftDisplayLeft(int distance = 1);
    bool shiftDisplayRight(int distance = 1);

    bool setCursorPosition(int position);

    bool moveCursorToSecondLine();

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

    MoveDirection getCursorMoveDirection() const { return cursor_move_direction_; }
    bool setCursorMoveDirection(MoveDirection direction);

    bool isDisplayShiftEnabled() const { return display_shift_enabled_; }
    bool setDisplayShiftEnabled(bool enabled);

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
    bool run_command_extra_delay(uint8_t data, RWMode rw, RSMode rs);

    void short_delay();

    bool update_backlight();
    bool update_function_set();
    bool update_display_control();
    bool update_entry_mode();

    bool cursor_or_display_shift(MoveDirection direction, bool is_display, int distance);

private:
    bool backlight_ = true;

    LinesMode lines_mode_ = LinesMode::Two;
    Font font_ = Font::Font5x8;

    bool display_enabled_{true};
    bool cursor_enabled_{true};
    bool cursor_blinking_enabled_{true};

    MoveDirection cursor_move_direction_{MoveDirection::Right};
    bool display_shift_enabled_{false};

    I2C i2c_;
    TIM_TypeDef *timer_;
};
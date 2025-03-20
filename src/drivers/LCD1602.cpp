#include "LCD1602.h"

#include "Sleep.h"
#include "periph/GPIO.h"
#include "periph/PeriphBase.h"
#include "periph/RCC.h"

namespace
{

I2C_TypeDef *i2c1 = I2C2;
I2C_TypeDef *i2c0 = I2C2;

bool masterTransmit(I2C_TypeDef *i2c, uint8_t address, const uint8_t *buf, uint32_t num_bytes)
{
    if (num_bytes == 0)
    {
        return true;
    }

    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB))
    {}

    i2c->DR = address << 1 | 0;
    while (!(i2c->SR1 & I2C_SR1_ADDR))
    {}

    (void)i2c->SR2; // Clear ADDR

    for (uint32_t i = 0; i < num_bytes; ++i)
    {
        while (!(i2c->SR1 & I2C_SR1_TXE))
        {}
        i2c->DR = *buf++;
    }

    while (!(i2c->SR1 & I2C_SR1_BTF))
    {}

    i2c->CR1 |= I2C_CR1_STOP;

    return true;
}

void i2c_write_blocking(I2C_TypeDef *i2c, uint8_t address, const uint8_t *data, uint32_t length, bool nostop)
{
    masterTransmit(i2c, address, data, length);
}

void sleep_us(uint64_t us)
{
    utils::sleepUsec(TIM2, us);
}

void sleep_ms(uint64_t ms)
{
    utils::sleepMsec(TIM2, ms);
}

bool setupI2C(I2C_TypeDef *i2c)
{
    i2c->CR1 = 0;
    i2c->CR1 = I2C_CR1_ACK;
    i2c->CR2 = (42 << I2C_CR2_FREQ_Pos);
    i2c->OAR1 = 0;
    i2c->OAR2 = 0;
    i2c->SR1 = 0;
    i2c->CCR = 210;
    i2c->TRISE = 43;
    i2c->FLTR = 0;
    i2c->CR1 |= I2C_CR1_PE;
    return true;
}

} // namespace

void i2c_write_byte(LCD *lcd_inst, uint8_t val)
{
    i2c_write_blocking((lcd_inst->i2c_type == 1) ? i2c1 : i2c0, lcd_inst->addr, &val, 1, false);
}

void lcd_toggle_enable(LCD *lcd_inst, uint8_t val, const uint64_t delay_us)
{
    sleep_us(delay_us);
    i2c_write_byte(lcd_inst, val | LCD_ENABLE_PIN);
    sleep_us(delay_us);
    i2c_write_byte(lcd_inst, val & ~LCD_ENABLE_PIN);
}

void lcd_send_byte(LCD *lcd_inst, uint8_t val, uint8_t mode, uint64_t delay_us)
{
    // Move the data bits in the appropriate place
    uint8_t high = mode | (val & 0xF0) | (lcd_inst->status.backlight_on << 3);
    uint8_t low = mode | ((val << 4) & 0xF0) | (lcd_inst->status.backlight_on << 3);

    // Let's identify the command - check page 24
    // Remember - val has the 8 bits needed for instructions
    // The other bits are set externally
    if (mode == LCD_COMMAND_MODE)
    {
        if (LCD_SETDDRAMADDR & val)
        {
            // Leave as be
        }
        else if (LCD_SETCGRAMADDR & val)
        {
            // TODO
            return;
        }
        else if (LCD_FUNCTIONSET & val)
        {
            // FORBIDDEN
            return;
        }
        // All low shifts are incremented by 4
        // Check header beginning
        else if (LCD_CURSORSHIFT & val)
        {
            low = low | (lcd_inst->status.shift_display << 7);
            low = low | (lcd_inst->status.shift_right << 6);
        }
        else if (LCD_DISPLAYCONTROL & val)
        {
            low = low | (lcd_inst->status.display_on << 6);
            low = low | (lcd_inst->status.cursor_on << 5);
            low = low | (lcd_inst->status.blinking_on << 4);
        }
        else if (LCD_ENTRYMODESET & val)
        {
            low = low | (lcd_inst->status.cursor_dir << 5);
            low = low | (lcd_inst->status.display_shift << 4);
        }
        // The rest have no parameters
    }

    // Send two transfers
    lcd_toggle_enable(lcd_inst, high, delay_us);
    lcd_toggle_enable(lcd_inst, low, delay_us);
}

void lcd_init(LCD *lcd_inst, int address, uint8_t pin_sda, uint8_t pin_scl, i2c_inst_t *i2c_inst)
{
    rcc::enableClocks(rcc::I2C_1);

    constexpr Pin scl_pin{GPIOPort::B, 8};
    constexpr Pin sda_pin{GPIOPort::B, 9};

    gpio::configureAlternate(scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);
    gpio::configureAlternate(sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);

    setupI2C(i2c_inst);

    /* Init the struct */
    lcd_inst->addr = (address == -1) ? 0x27 : (uint8_t)address;
    lcd_inst->i2c_type = (i2c_inst == i2c1);
    lcd_inst->pin_sda = pin_sda;
    lcd_inst->pin_scl = pin_scl;

    lcd_inst->status.cursor_dir = 1; /* LCD_ENTRYLEFT */
    lcd_inst->status.cursor_on = 0;  /* LCD_CURSORON */
    lcd_inst->status.display_on = 1; /* LCD_DISPLAYON */
    lcd_inst->status.display_shift = 0;
    lcd_inst->status.blinking_on = 0; /* LCD_BLINKON */
    lcd_inst->status.backlight_on = 1;
    lcd_inst->status.shift_display = 0;
    lcd_inst->status.shift_right = 0;

    // The initialization is from page 46 of the HD44780U datasheet.

    // Sleeps are for the display, taken from the same page.
    sleep_us(40000);
    lcd_toggle_enable(lcd_inst, LCD_FUNCTIONSET | LCD_8BITMODE, FAST_DELAY);
    sleep_us(4100);
    lcd_toggle_enable(lcd_inst, LCD_FUNCTIONSET | LCD_8BITMODE, FAST_DELAY);
    sleep_us(100);
    lcd_toggle_enable(lcd_inst, LCD_FUNCTIONSET | LCD_8BITMODE, FAST_DELAY);
    lcd_toggle_enable(lcd_inst, LCD_FUNCTIONSET, FAST_DELAY);

    lcd_send_byte(lcd_inst, LCD_ENTRYMODESET, LCD_COMMAND_MODE, FAST_DELAY);
    lcd_send_byte(lcd_inst, LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND_MODE, FAST_DELAY);
    lcd_send_byte(lcd_inst, LCD_DISPLAYCONTROL, LCD_COMMAND_MODE, FAST_DELAY);

    sleep_ms(1);

    lcd_clear(lcd_inst);
}

void lcd_clear(LCD *lcd_inst)
{
    // Watch out, it's slow (2ms)
    lcd_send_byte(lcd_inst, LCD_CLEARDISPLAY, LCD_COMMAND_MODE, SLOW_DELAY);
}

void lcd_char(LCD *lcd_inst, char val)
{
    lcd_send_byte(lcd_inst, val, LCD_CHAR_MODE, FAST_DELAY);
}

void lcd_string(LCD *lcd_inst, const char *s)
{
    while (*s)
    {
        lcd_char(lcd_inst, *s++);
    }
}

void lcd_set_cursor_pos(LCD *lcd_inst, uint8_t line, uint8_t position)
{
    // Cannot move past these boundaries
    if (position >= MAX_CHARS || line >= MAX_LINES)
    {
        return;
    }
    uint8_t val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(lcd_inst, val, LCD_COMMAND_MODE, FAST_DELAY);
}

void lcd_set_display(LCD *lcd_inst, uint8_t set_display)
{
    lcd_inst->status.display_on = set_display;

    lcd_send_byte(lcd_inst, LCD_DISPLAYCONTROL, LCD_COMMAND_MODE, FAST_DELAY);
}

void lcd_show_cursor(LCD *lcd_inst, uint8_t show_cursor)
{
    lcd_inst->status.cursor_on = show_cursor;

    lcd_send_byte(lcd_inst, LCD_DISPLAYCONTROL, LCD_COMMAND_MODE, FAST_DELAY);
}

void lcd_set_blinking(LCD *lcd_inst, uint8_t set_blinking)
{
    lcd_inst->status.blinking_on = set_blinking;

    lcd_send_byte(lcd_inst, LCD_DISPLAYCONTROL, LCD_COMMAND_MODE, FAST_DELAY);
}

void lcd_set_backlight(LCD *lcd_inst, uint8_t set_backlight)
{
    // Turn both backlight and display on/off at the same time
    lcd_inst->status.backlight_on = set_backlight;
    lcd_inst->status.display_on = set_backlight;

    // Make use of display, so we preserve the state of the memory
    lcd_send_byte(lcd_inst, LCD_DISPLAYCONTROL, LCD_COMMAND_MODE, FAST_DELAY);
}

void lcd_shift_display(LCD *lcd_inst, uint8_t move_right, uint8_t amount)
{
    lcd_inst->status.shift_display = 1;
    lcd_inst->status.shift_right = move_right;

    while (amount--)
    {
        lcd_send_byte(lcd_inst, LCD_CURSORSHIFT, LCD_COMMAND_MODE, FAST_DELAY);
    }
}

void lcd_shift_cursor(LCD *lcd_inst, uint8_t move_right, uint8_t amount)
{
    lcd_inst->status.shift_display = 0;
    lcd_inst->status.shift_right = move_right;

    while (amount--)
    {
        lcd_send_byte(lcd_inst, LCD_CURSORSHIFT, LCD_COMMAND_MODE, FAST_DELAY);
    }
}
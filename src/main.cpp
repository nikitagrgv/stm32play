#include "DeviceCMSIS.h"
#include "Print.h"
#include "Sleep.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/DHT11Command.h"
#include "commands/PrintCommand.h"
#include "commands/SHT31Command.h"
#include "core/Globals.h"
#include "debug/Statistic.h"
#include "drivers/DHT11Driver.h"
#include "drivers/LCD1602.h"
#include "low_level/clock.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/I2C.h"
#include "periph/IRQ.h"
#include "periph/PeriphBase.h"
#include "periph/RCC.h"
#include "periph/SysTick.h"
#include "periph/TIM.h"
#include "periph/USART.h"
#include "utils/CRC.h"
#include "utils/DataStream.h"
#include "utils/FixedBitset.h"

#include <cstdlib>
#include <memory>

FixedDataStream<1024> usart1_stream;

CommandBuffer command_buffer;

CommandExecutor command_executor;

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

constexpr int FAST_DELAY_US = 80;

void triggerLcd(I2C i2c, uint8_t address, uint8_t data)
{
    utils::sleepUsec(TIM2, FAST_DELAY_US);

    constexpr uint8_t enable_mask = 1 << 2;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

    i2c::masterTransmitBlocking(i2c, address, data | enable_mask);
    utils::sleepUsec(TIM2, FAST_DELAY_US);
    i2c::masterTransmitBlocking(i2c, address, data & enable_clear_mask);
}

void runLcdCommand(I2C i2c, uint8_t address, uint8_t data, RWMode rw, RSMode rs, bool backlight = true)
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

bool runLcd(I2C i2c)
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

int main()
{
    glob::SYSTEM_CORE_CLOCK = calcSystemCoreClock();
    glob::APB1_PERIPH_CLOCK = calcAPB1PeriphClock();
    glob::APB1_TIMER_CLOCK = calcAPB1TimerClock();
    glob::APB2_PERIPH_CLOCK = calcAPB2PeriphClock();
    glob::APB2_TIMER_CLOCK = calcAPB2TimerClock();

    irq::disableInterrupts();

    rcc::enableClocks(rcc::GPIO_A | rcc::GPIO_B | rcc::GPIO_C | rcc::SYSCFG_OR_AFIO | rcc::TIM_2);

    // Led
    constexpr Pin led_pin{GPIOPort::C, 13};
    gpio::configureOutput(led_pin, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);

    // User key
    constexpr Pin user_key{GPIOPort::A, 0};
    gpio::configureInput(user_key, gpio::PullMode::Up);

    // SysTick
    constexpr uint32_t systick_frequency = 1000;
    systick::setupTimer(systick_frequency, systick::ENABLE_INTERRUPT);
    systick::restartTimer();

    // USART1
    rcc::enableClocks(rcc::USART_1);

    constexpr Pin usart_tx_pin{GPIOPort::A, 9};
    constexpr Pin usart_rx_pin{GPIOPort::A, 10};
    gpio::configureAlternate(usart_tx_pin, 7, gpio::OutputMode::PushPull, gpio::OutputSpeed::High);
    gpio::configureAlternate(usart_rx_pin, 7, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High, gpio::PullMode::Up);

    constexpr uint32_t baudrate = 56'000;
    constexpr uint32_t flags = usart::ENABLE_RECEIVE | usart::ENABLE_TRANSMIT | usart::ENABLE_RECEIVE_INTERRUPT;
    usart::setupUsart(USART::USART_1, baudrate, flags);
    irq::enableInterrupt(InterruptType::USART1IRQ);

    io::setPrintUsart(USART1);

    // I2C1
    rcc::enableClocks(rcc::I2C_1);

    constexpr Pin scl_pin{GPIOPort::B, 8};
    constexpr Pin sda_pin{GPIOPort::B, 9};
    gpio::configureAlternate(scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);
    gpio::configureAlternate(sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);

    i2c::setupI2C(I2C::I2C_1);

    // Handlers
    irq::setHandler(InterruptType::SysTickIRQ, [](void *) {
        //
        ++glob::total_msec;
    });

    irq::setHandler(InterruptType::USART1IRQ, [](void *) {
        if (USART1->SR & USART_SR_RXNE)
        {
            const uint8_t data = USART1->DR;
            usart1_stream.writeByte(data);
            stat::addReadBytesUsart(1);
        }
    });

    // Others
    command_executor.addCommand(std::make_unique<PrintCommand>());

    constexpr Pin dht_pin{GPIOPort::B, 5};
    TIM_TypeDef *dht_timer = TIM2;
    command_executor.addCommand(std::make_unique<DHT11Command>(dht_pin, dht_timer));

    constexpr I2C sht31_i2c = I2C::I2C_1;
    command_executor.addCommand(std::make_unique<SHT31Command>(sht31_i2c));

    gpio::setPinOutput(led_pin, false);

    irq::enableInterrupts();

    io::printSyncFmt("--- Device is ready ---\n");

    bool user_key_state = gpio::getPinInput(user_key);
    uint32_t user_key_last_change_time = glob::total_msec;

    while (true)
    {
        const uint32_t cur_time = glob::total_msec;
        if (cur_time - user_key_last_change_time > 10)
        {
            user_key_last_change_time = cur_time;
            const bool new_user_key_state = gpio::getPinInput(user_key);
            if (new_user_key_state != user_key_state)
            {
                user_key_state = new_user_key_state;
                if (!user_key_state)
                {
                    io::printSyncFmt("User key pressed\n");

                    if (0)
                    {
                        float temperature = 0.0f;
                        float humidity = 0.0f;
                        const bool valid = checkSht31(I2C::I2C_1, temperature, humidity);
                        if (valid)
                        {
                            io::printSyncFmt("T = %f, H = %f\n", temperature, humidity);
                        }
                        else
                        {
                            io::printSyncFmt("SHT31 error\n");
                        }
                    }
                    else
                    {
                        const uint8_t address = 0x27;
                        runLcd(I2C::I2C_1);
                        //
                        // LCD my_lcd;
                        // lcd_init(&my_lcd, address, 4, 5, i2c);
                        //
                        // utils::sleepMsec(1);
                        // runLcdCommand(i2c, address, 'T', RWMode::Write, RSMode::Data);
                        // utils::sleepMsec(1);
                        // runLcdCommand(i2c, address, 'h', RWMode::Write, RSMode::Data);
                        // utils::sleepMsec(1);
                        // runLcdCommand(i2c, address, 'a', RWMode::Write, RSMode::Data);
                        // utils::sleepMsec(1);
                        //
                        // char message[] = "Thanks extremq";
                        // lcd_string(&my_lcd, message);
                    }
                }
            }
        }

        uint8_t byte;
        while (usart1_stream.readByte(byte))
        {
            stat::addReadBytesStream(1);
            command_buffer.writeByte(byte);
        }

        const char *command = command_buffer.getCurrentCommand();
        if (!command)
        {
            continue;
        }

        const bool executed = command_executor.execute(command);
        if (!executed)
        {
            io::printSyncFmt("can't execute: `%s`\n", command);
        }

        command_buffer.flushCurrentCommand();
    }
}

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
#include "drivers/LCD1602Driver.h"
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
    gpio::setPinOutput(led_pin, false);

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

    // Commands
    command_executor.addCommand(std::make_unique<PrintCommand>());

    constexpr Pin dht_pin{GPIOPort::B, 5};
    TIM_TypeDef *dht_timer = TIM2;
    command_executor.addCommand(std::make_unique<DHT11Command>(dht_pin, dht_timer));

    constexpr I2C sht31_i2c = I2C::I2C_1;
    command_executor.addCommand(std::make_unique<SHT31Command>(sht31_i2c));

    irq::enableInterrupts();

    // Display
    LCD1602Driver display{I2C::I2C_1, TIM2};
    const bool display_initialized = display.initialize();
    if (display_initialized)
    {
        display.print("Initialized");
    }
    else
    {
        io::printSyncFmt("LCD1602 Initialization failed\n");
    }

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
                    display.shiftCursorRight(3);

                    // display.print("Hello!");
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

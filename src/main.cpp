#include "DeviceCMSIS.h"
#include "Print.h"
#include "Sleep.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/DHT11Command.h"
#include "commands/PrintCommand.h"
#include "core/Globals.h"
#include "debug/Statistic.h"
#include "drivers/DHT11Driver.h"
#include "low_level/clock.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/IRQ.h"
#include "periph/PeriphBase.h"
#include "periph/RCC.h"
#include "periph/SysTick.h"
#include "periph/TIM.h"
#include "periph/USART.h"
#include "utils/DataStream.h"
#include "utils/FixedBitset.h"

#include <memory>

FixedDataStream<1024> usart1_stream;

CommandBuffer command_buffer;

CommandExecutor command_executor;


int main()
{
    glob::SYSTEM_CORE_CLOCK = calcSystemCoreClock();

    irq::disableInterrupts();

    rcc::enableClocks(rcc::GPIO_A | rcc::GPIO_B | rcc::GPIO_C | rcc::SYSCFG_OR_AFIO | rcc::USART_1 | rcc::TIM_2);

    constexpr Pin led_pin{GPIOPort::C, 13};

    constexpr Pin usart_tx_pin{GPIOPort::A, 9};
    constexpr Pin usart_rx_pin{GPIOPort::A, 10};

    constexpr Pin i2c_sda_pin{GPIOPort::B, 7};
    constexpr Pin i2c_scl_pin{GPIOPort::B, 8};

    gpio::configureOutput(led_pin, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);

#ifdef STM32F103
    gpio::configureAlternateOutput(usart_tx_pin, gpio::OutputMode::PushPull, gpio::OutputSpeed::High);
    gpio::configureAlternateInput(usart_rx_pin, gpio::PullMode::Up);
#elifdef STM32F401
    gpio::configureAlternate(usart_tx_pin, 7, gpio::OutputMode::PushPull, gpio::OutputSpeed::High);
    gpio::configureAlternate(usart_rx_pin, 7, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);
#endif

#ifdef STM32F103
    #error
#elifdef STM32F401
    gpio::configureAlternate(i2c_sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High, gpio::PullMode::Up);
    gpio::configureAlternate(i2c_scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High, gpio::PullMode::Up);
#endif

    // SysTick
    constexpr uint32_t systick_frequency = 1000;
    systick::setupTimer(systick_frequency, systick::ENABLE_INTERRUPT);
    systick::restartTimer();

    // USART1
    constexpr uint32_t baudrate = 56'000;
    constexpr uint32_t flags = usart::ENABLE_RECEIVE | usart::ENABLE_TRANSMIT | usart::ENABLE_RECEIVE_INTERRUPT;
    usart::setupUsart(USART::USART_1, baudrate, flags);
    irq::enableInterrupt(InterruptType::USART1IRQ);

    io::setPrintUsart(USART1);

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

    gpio::setPinOutput(led_pin, false);

    irq::enableInterrupts();

    io::printSyncFmt("--- Device is ready ---\n");

    while (true)
    {
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

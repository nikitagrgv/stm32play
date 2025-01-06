#include "Print.h"
#include "Sleep.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/PrintCommand.h"
#include "debug/Statistic.h"
#include "drivers/DHT11Driver.h"
#include "periph/EXTI.h"
#include "periph/GPIO.h"
#include "periph/IRQ.h"
#include "periph/PeriphBase.h"
#include "periph/SysTick.h"
#include "periph/TIM.h"
#include "periph/USART.h"
#include "utils/DataStream.h"
#include "utils/FixedBitset.h"

#include <memory>
#include <stm32f1xx.h>

FixedDataStream<1024> usart1_stream;

CommandBuffer command_buffer;

CommandExecutor command_executor;


int main()
{
    irq::disableInterrupts();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN
        | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // C13 open drain
    gpio::setPinMode({GPIOPort::C, 13}, gpio::PinMode::GeneralOpenDrain50MHz);

    gpio::setPinMode({GPIOPort::A, 9}, gpio::PinMode::AlternatePushPull50MHz); // USART1 TX

    gpio::setPinMode({GPIOPort::A, 10}, gpio::PinMode::InputPullUpOrDown); // USART1 RX
    gpio::setPinPullUpOrDown({GPIOPort::A, 10}, gpio::PullUpOrDownMode::Up);

    gpio::setPinMode({GPIOPort::B, 12}, gpio::PinMode::GeneralOpenDrain50MHz);

    // SysTick
    constexpr uint32_t systick_frequency = 1000;
    systick::setupTimer(systick_frequency, systick::ENABLE_INTERRUPT);
    systick::restartTimer();

    // USART1
    constexpr uint32_t baudrate = 56'000;
    constexpr uint32_t flags = usart::ENABLE_RECEIVE | usart::ENABLE_TRANSMIT | usart::ENABLE_RECEIVE_INTERRUPT;
    usart::setupUsart(USART1, baudrate, flags);
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

    gpio::setPinOutput({GPIOPort::B, 12}, true);

    struct TestCommand : public ICommand
    {
        const char *name() override { return "go"; }
        bool execute(const char *args) override
        {
            Pin output_pin{GPIOPort::B, 12};
            Pin input_pin{GPIOPort::B, 5};
            TIM_TypeDef *timer = TIM2;
            DHT11Driver dht11{input_pin, output_pin, timer};

            float temperature = 0.0f;
            float humidity = 0.0f;
            const DHT11Driver::ErrorCode code = dht11.run(temperature, humidity);

            if (code == DHT11Driver::ErrorCode::Timeout)
            {
                io::printSyncFmt("DHT11: Timeout\n");
                return true;
            }

            if (code == DHT11Driver::ErrorCode::InvalidChecksum)
            {
                io::printSyncFmt("DHT11: Invalid checksum\n");
                return true;
            }

            if (code != DHT11Driver::ErrorCode::Success)
            {
                io::printSyncFmt("DHT11: Unexpected error\n");
                return true;
            }

            io::printSyncFmt("temp = %f, hum = %f\n", temperature, humidity);

            return true;
        }
    };
    command_executor.addCommand(std::make_unique<TestCommand>());

    struct CheckTimerCommand : public ICommand
    {
        const char *name() override { return "time"; }
        bool execute(const char *args) override
        {
            const uint32_t value = TIM2->CNT;
            io::printSyncFmt("time : %lu\n", value);
            return true;
        }
    };
    command_executor.addCommand(std::make_unique<CheckTimerCommand>());


    struct ResetTimerCommand : public ICommand
    {
        const char *name() override { return "reset"; }
        bool execute(const char *args) override
        {
            tim::restartTimer(TIM2);
            return true;
        }
    };
    command_executor.addCommand(std::make_unique<ResetTimerCommand>());

    gpio::setPinOutput({GPIOPort::C, 13}, false);

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

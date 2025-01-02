#include "Print.h"
#include "Sleep.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/PrintCommand.h"
#include "debug/Statistic.h"
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

void sleep1()
{
    volatile uint32_t t = 4;
    while (--t)
    {}
}

FixedBitset<5 * 8> dht_data;

volatile bool listening = false;
volatile int num_height = 0;
volatile int num_written_bits = 0;

CommandBuffer command_buffer;

CommandExecutor command_executor;


int main()
{
    irq::disableInterrupts();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN
        | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // A0
    gpio::setPinMode(GPIOPort::A, 0, gpio::PinMode::InputFloating);
    exti::setupEXTI(GPIOPort::A, 0, exti::TriggerMode::RisingEdges, exti::ENABLE_INTERRUPT);

    irq::enableInterrupt(InterruptType::EXTI0IRQ);

    irq::setHandler(InterruptType::EXTI0IRQ, [](void *) {
        if (EXTI->PR & EXTI_PR_PR0)
        {
            if (listening)
            {
                ++num_height;
                if (num_height >= 3)
                {
                    tim::restartTimer(TIM2);
                }
            }
            EXTI->PR = EXTI_PR_PR0;
        }
    });

    // C13 open drain
    gpio::setPinMode(GPIOPort::C, 13, gpio::PinMode::GeneralOpenDrain50MHz);

    gpio::setPinMode(GPIOPort::A, 9, gpio::PinMode::AlternatePushPull50MHz); // USART1 TX

    gpio::setPinMode(GPIOPort::A, 10, gpio::PinMode::InputPullUpOrDown); // USART1 RX
    gpio::setPinPullUpOrDown(GPIOPort::A, 10, gpio::PullUpOrDownMode::Up);

    gpio::setPinMode(GPIOPort::B, 12, gpio::PinMode::GeneralOpenDrain50MHz);

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


    // TIM2
    constexpr uint32_t frequency = 1'000'000;
    constexpr uint32_t reload_value = 48;
    tim::setupTimer(TIM2, frequency, reload_value, tim::SINGLE_SHOT | tim::ENABLE_UPDATE_INTERRUPT);
    irq::enableInterrupt(InterruptType::TIM2IRQ);

    irq::setHandler(InterruptType::SysTickIRQ, [](void *) {
        //
        ++glob::total_msec;
    });

    irq::setHandler(InterruptType::TIM2IRQ, [](void *) {
        if (TIM2->SR & TIM_SR_UIF)
        {
            TIM2->SR &= ~TIM_SR_UIF;
            if (num_height >= 3 && num_written_bits < 40)
            {
                gpio::setPinOutput(GPIOPort::C, 13, true);
                sleep1();
                gpio::setPinOutput(GPIOPort::C, 13, false);
                const bool bit = gpio::getPinInput(GPIOPort::A, 0);
                dht_data.set(num_written_bits, bit);
                ++num_written_bits;
            }
        }
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

    gpio::setPinOutput(GPIOPort::B, 12, true);

    struct TestCommand : public ICommand
    {
        const char *name() override { return "go"; }
        bool execute(const char *args) override
        {
            num_written_bits = 0;
            gpio::setPinOutput(GPIOPort::C, 13, false);

            gpio::setPinOutput(GPIOPort::B, 12, false);
            utils::sleepMsec(20);

            listening = true;
            num_height = 0;
            gpio::setPinOutput(GPIOPort::B, 12, true);
            utils::sleepMsec(10);
            listening = false;

            gpio::setPinOutput(GPIOPort::C, 13, false);

            const uint32_t start_time_ms = glob::total_msec;
            constexpr uint32_t TIMEOUT_MS = 100;
            const uint32_t end_time = start_time_ms + TIMEOUT_MS;

            while (num_written_bits != 40)
            {
                if (glob::total_msec > end_time)
                {
                    io::printSyncFmt("DHT11: Timeout\n");
                    return true;
                }
            }

            const auto flip_byte = [](uint8_t byte) {
                uint8_t result = 0;
                for (int i = 0; i < 8; ++i)
                {
                    result |= ((byte >> i) & 1) << (7 - i);
                }
                return result;
            };


            uint8_t data[5];
            dht_data.copyData<uint8_t>(data, 5);
            for (int i = 0; i < 5; ++i)
            {
                data[i] = flip_byte(data[i]);
            }

            const uint8_t checksum = data[0] + data[1] + data[2] + data[3];
            if (checksum != data[4])
            {
                io::printSyncFmt("DHT11: Invalid checksum\n");
                return true;
            }

            float humidity = data[0];
            humidity += data[1] / 256.0f;
            float temperature = data[2];
            temperature += data[3] / 256.0f;

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

    gpio::setPinOutput(GPIOPort::C, 13, false);

    irq::enableInterrupts();

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

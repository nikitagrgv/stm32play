#include "DataStream.h"
#include "Print.h"
#include "Sleep.h"
#include "Statistic.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/PrintCommand.h"
#include "periph/GPIO.h"
#include "periph/TIM.h"
#include "periph/USART.h"

#include <memory>
#include <stm32f1xx.h>

FixedDataStream<1024> usart1_stream;

void sleep1()
{
    volatile uint32_t t = 10;
    while (--t)
    {}
}


FixedDataStream<1024> temp;
volatile bool listening = false;
volatile int num_height = 0;
extern "C"
{
    void USART1_IRQHandler()
    {
        if (USART1->SR & USART_SR_RXNE)
        {
            const uint8_t data = USART1->DR;
            usart1_stream.writeByte(data);
            stat::addReadBytesUsart(1);
        }
    }

    void TIM2_IRQHandler()
    {
        if (TIM2->SR & TIM_SR_UIF)
        {
            TIM2->SR &= ~TIM_SR_UIF;
            gpio::setPinOutput(GPIOC, 13, true);
            sleep1();
            gpio::setPinOutput(GPIOC, 13, false);
            const bool bit = gpio::getPinInput(GPIOA, 0);
            temp.writeByte(bit ? '1' : '0');
        }
    }

    void EXTI0_IRQHandler()
    {
        if (EXTI->PR & EXTI_PR_PR0)
        {
            gpio::setPinOutput(GPIOC, 13, true);
            sleep1();
            gpio::setPinOutput(GPIOC, 13, false);
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
    }
}

CommandBuffer command_buffer;

CommandExecutor command_executor;


int main()
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN
        | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // C13 open drain
    gpio::setPinMode(GPIOC, 13, gpio::PinMode::GeneralOpenDrain50MHz);

    gpio::setPinMode(GPIOA, 9, gpio::PinMode::AlternatePushPull50MHz); // USART1 TX

    gpio::setPinMode(GPIOA, 10, gpio::PinMode::InputPullUpOrDown); // USART1 RX
    gpio::setPinPullUpOrDown(GPIOA, 10, gpio::PullUpOrDownMode::Up);

    gpio::setPinMode(GPIOB, 12, gpio::PinMode::GeneralOpenDrain50MHz);

    // SysTick
    SysTick->LOAD = 8'000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // USART1
    constexpr uint32_t baudrate = 56'000;
    constexpr uint32_t flags = usart::ENABLE_RECEIVE | usart::ENABLE_TRANSMIT | usart::ENABLE_RECEIVE_INTERRUPT;
    usart::setupUsart(USART1, baudrate, flags);
    NVIC_EnableIRQ(USART1_IRQn);

    io::setPrintUsart(USART1);


    // TIM2
    constexpr uint32_t frequency = 1'000'000;
    constexpr uint32_t reload_value = 5;
    tim::setupTimer(TIM2, frequency, reload_value, tim::SINGLE_SHOT | tim::ENABLE_UPDATE_INTERRUPT);
    NVIC_EnableIRQ(TIM2_IRQn);

    // A0
    gpio::setPinMode(GPIOA, 0, gpio::PinMode::InputFloating);
    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0;
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->FTSR &= ~EXTI_FTSR_FT0; // Falling edge
    EXTI->RTSR |= EXTI_RTSR_RT0;  // Rising edge
    NVIC_EnableIRQ(EXTI0_IRQn);


    __enable_irq(); // enable interrupts


    // Others
    command_executor.addCommand(std::make_unique<PrintCommand>());

    gpio::setPinOutput(GPIOB, 12, true);

    struct TestCommand : public ICommand
    {
        const char *name() override { return "go"; }
        bool execute(const char *args) override
        {
            gpio::setPinOutput(GPIOC, 13, false);
            io::printSyncFmt("goo\n");

            gpio::setPinOutput(GPIOB, 12, false);
            utils::sleepMsec(20);

            listening = true;
            num_height = 0;
            gpio::setPinOutput(GPIOB, 12, true);
            utils::sleepMsec(10);
            listening = false;

            gpio::setPinOutput(GPIOC, 13, false);
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

    gpio::setPinOutput(GPIOC, 13, false);


    char GGGG[1024 + 1];
    while (true)
    {
        const int num_read = temp.readData((uint8_t *)GGGG, 1024);
        if (num_read)
        {
            GGGG[num_read] = 0;
            io::printSyncFmt("%s", GGGG);
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

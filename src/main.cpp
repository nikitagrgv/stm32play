#include "DataStream.h"
#include "Print.h"
#include "Sleep.h"
#include "Statistic.h"
#include "commands/CommandBuffer.h"
#include "commands/CommandExecutor.h"
#include "commands/PrintCommand.h"
#include "periph/GPIO.h"
#include "periph/USART.h"

#include <memory>
#include <stm32f1xx.h>

volatile bool led_state = false;
void toggleIndicatorLed()
{
    gpio::setPinOutput(GPIOC, 13, led_state);
    led_state = !led_state;
}

FixedDataStream<1024> usart1_stream;

extern "C"
{
    void USART1_IRQHandler(void)
    {
        if (USART1->SR & USART_SR_RXNE)
        {
            toggleIndicatorLed();
            const uint8_t data = USART1->DR;
            usart1_stream.writeByte(data);
            stat::addReadBytesUsart(1);
        }
    }
}

CommandBuffer command_buffer;

CommandExecutor command_executor;


namespace tim
{

constexpr uint32_t MAX_PRESCALER = 0xFFFF;
constexpr uint32_t MAX_RELOAD_VALUE = 0xFFFF;

enum SetupFlags : uint32_t
{
    SINGLE_SHOT = TIM_CR1_OPM,
};

void setupTimer(TIM_TypeDef *tim, uint32_t frequency, uint32_t reload_value, uint32_t flags = 0)
{
    const uint32_t prescaler = (glob::SYS_FREQUENCY / frequency) - 1;

    MICRO_ASSERT(prescaler <= MAX_PRESCALER);
    MICRO_ASSERT(reload_value <= MAX_RELOAD_VALUE);

    const bool single_shot = flags & SINGLE_SHOT;

    tim->PSC = prescaler;
    tim->ARR = reload_value - single_shot;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG; // Flush ARR and PSC!

    const uint32_t cr1 = flags;
    tim->CR1 = cr1;
}

void runTimer(TIM_TypeDef *tim)
{
    tim->CNT = 0;
    tim->CR1 |= TIM_CR1_CEN;
}

} // namespace tim


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

    io::setPrintUsart(USART1);

    NVIC_EnableIRQ(USART1_IRQn);
    __enable_irq(); // enable interrupts

    command_executor.addCommand(std::make_unique<PrintCommand>());

    gpio::setPinOutput(GPIOB, 12, true);

    struct TestCommand : public ICommand
    {
        const char *name() override { return "go"; }
        bool execute(const char *args) override
        {
            io::printSyncFmt("goo\n");

            gpio::setPinOutput(GPIOB, 12, false);
            utils::sleepMsec(20);
            gpio::setPinOutput(GPIOB, 12, true);

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
            tim::runTimer(TIM2);
            return true;
        }
    };
    command_executor.addCommand(std::make_unique<ResetTimerCommand>());

    constexpr uint32_t frequency = 1'000;
    tim::setupTimer(TIM2, frequency, tim::MAX_RELOAD_VALUE, tim::SINGLE_SHOT);

    tim::runTimer(TIM2);

    while (true)
    {
        io::printSyncFmt("time : %lu\n", TIM2->CNT);
        utils::sleepMsec(50);


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

        toggleIndicatorLed();
    }
}

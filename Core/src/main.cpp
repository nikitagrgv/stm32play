#include "CommandBuffer.h"
#include "DataStream.h"
#include "GPIO.h"
#include "Globals.h"
#include "Print.h"
#include "Statistic.h"
#include "StringUtils.h"
#include "Utils.h"

#include <cassert>
#include <cmath>
#include <stm32f103xb.h>

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

struct HelpCommand
{
    static constexpr const char *name = "help";
    static bool execute(const char *args)
    {
        if (!str_utils::isEmpty(args))
        {
            return false;
        }
        io::printSync("-- help --\n");
        io::printSync("get:\n");
        io::printSync("  stat\n");
        io::printSync("----------\n");
        return true;
    }
};

struct GetCommand
{
    static constexpr const char *name = "get";
    static bool execute(const char *args)
    {
        if (str_utils::isEmpty(args))
        {
            return false;
        }
        if (str_utils::compareTrimmed(args, "stat"))
        {
#ifdef ENABLE_DATA_STATISTIC
            io::printSyncFmt("num read usart = %d\n", stat::getReadBytesUsart());
            io::printSyncFmt("num read stream = %d\n", stat::getReadBytesStream());
#elif
            io::printSync("Statistic is disabled\n");
#endif
            return true;
        }
        return false;
    }
};

class CommandExecutor
{
public:
    bool execute(const char *command)
    {
        command = str_utils::skipStartSpaces(command);

        if (try_execute<HelpCommand>(command))
        {
            return true;
        }
        if (try_execute<GetCommand>(command))
        {
            return true;
        }

        return false;
    }

private:
    template<typename F>
    static bool try_execute(const char *command, const char *cmd_name, F &&func)
    {
        assert(*command != ' '); // spaces must be already skipped
        const char *name_end = str_utils::skipStart(command, cmd_name);
        if (!name_end)
        {
            return false;
        }

        const char *args = str_utils::skipStartSpaces(name_end);
        if (*args != 0 && args == name_end)
        {
            // must be at least one space between command name and args
            return false;
        }

        return func(args);
    }

    template<typename C>
    static bool try_execute(const char *command)
    {
        const char *cmd_name = C::name;
        auto f = &C::execute;
        return try_execute(command, cmd_name, f);
    }
};

CommandExecutor command_executor;

int main()
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN
        | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // C13 open drain
    gpio::setPinMode(GPIOC, 13, gpio::PinMode::GeneralOpenDrain50MHz);

    gpio::setPinMode(GPIOA, 9, gpio::PinMode::AlternatePushPull50MHz); // USART1 TX
    gpio::setPinMode(GPIOA, 10, gpio::PinMode::InputPullUpOrDown);     // USART1 RX
    gpio::setPinPullUpOrDown(GPIOA, 10, gpio::PullUpOrDownMode::Up);

    // SysTick
    SysTick->LOAD = 8'000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // USART1
    constexpr int baudrate = 56'000;
    constexpr int brr_value = 8'000'000 / baudrate; // USARTDIV
    USART1->BRR = brr_value;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    io::setPrintUsart(USART1);

    NVIC_EnableIRQ(USART1_IRQn);
    __enable_irq(); // enable interrupts

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

        toggleIndicatorLed();
    }
}

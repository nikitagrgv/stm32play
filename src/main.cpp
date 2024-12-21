#include "CommandBuffer.h"
#include "DataStream.h"
#include "GPIO.h"
#include "Globals.h"
#include "HeapUsage.h"
#include "Print.h"
#include "Statistic.h"
#include "StringUtils.h"

#include <memory>
#include <stm32f103xb.h>
#include <vector>

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

class ICommand
{
public:
    virtual ~ICommand() = default;

    virtual const char *name() = 0;
    virtual bool execute(const char *args) = 0;
    virtual bool help() { return false; }
};

class PrintCommand final : public ICommand
{
public:
    const char *name() override { return "print"; }

    bool execute(const char *args) override
    {
        if (str_utils::isEmpty(args))
        {
            return false;
        }
        if (str_utils::compareTrimmed(args, "datastat"))
        {
#ifdef ENABLE_DATA_STATISTIC
            io::printSyncFmt("num read usart = %d\n", stat::getReadBytesUsart());
            io::printSyncFmt("num read stream = %d\n", stat::getReadBytesStream());
#elif
            io::printSync("Statistic is disabled\n");
#endif
            return true;
        }
        if (str_utils::compareTrimmed(args, "heapstat"))
        {
            uint32_t used = 0;
            uint32_t total = 0;
            utils::getSbrkUsage(used, total);
            io::printSyncFmt("sbrk usage = %u/%u\n", used, total);
            return true;
        }
        return false;
    }

    bool help() override
    {
        io::printSync("print entries:\n");
        io::printSync("  datastat\n");
        io::printSync("  heapstat\n");
        return true;
    }
};

class CommandExecutor
{
public:
    void addCommand(std::unique_ptr<ICommand> command)
    {
        commands_.push_back(std::move(command));
        commands_.shrink_to_fit();
    }

    bool execute(const char *command)
    {
        command = str_utils::skipStartSpaces(command);

        if (try_execute_help_command(command))
        {
            return true;
        }

        for (const std::unique_ptr<ICommand> &cmd : commands_)
        {
            const char *cmd_name = cmd->name();
            const char *name_end = str_utils::skipStart(command, cmd_name);
            if (!name_end)
            {
                // Name doesn't match
                continue;
            }

            const char *args = str_utils::skipStartSpaces(name_end);
            if (*args != 0 && args == name_end)
            {
                // Name doesn't match
                // Must be at least one space between command name and args
                continue;
            }

            return cmd->execute(args);
        }

        return false;
    }

private:
    bool try_execute_help_command(const char *command)
    {
        const char *name_end = str_utils::skipStart(command, "help");
        if (!name_end)
        {
            return false;
        }

        const char *arg = str_utils::skipStartSpaces(name_end);
        if (*arg != 0 && arg == name_end)
        {
            return false;
        }

        if (str_utils::allAreSpaces(arg))
        {
            io::printSync("commands:\n");
            for (const std::unique_ptr<ICommand> &cmd : commands_)
            {
                io::printSyncFmt("  %s\n", cmd->name());
            }
            return true;
        }

        for (const std::unique_ptr<ICommand> &cmd : commands_)
        {
            const char *cmd_name = cmd->name();
            const char *cmd_name_end = str_utils::skipStart(arg, cmd_name);
            if (!cmd_name_end)
            {
                continue;
            }
            if (!str_utils::allAreSpaces(cmd_name_end))
            {
                continue;
            }
            return cmd->help();
        }
        return false;
    }

private:
    std::vector<std::unique_ptr<ICommand>> commands_;
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

    command_executor.addCommand(std::make_unique<PrintCommand>());

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

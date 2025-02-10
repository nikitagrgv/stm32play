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

    rcc::enableClocks(
        rcc::GPIO_A | rcc::GPIO_B | rcc::GPIO_C | rcc::SYSCFG_OR_AFIO | rcc::USART_1 | rcc::TIM_2 | rcc::I2C_1);

    constexpr Pin led_pin{GPIOPort::C, 13};

    constexpr Pin usart_tx_pin{GPIOPort::A, 9};
    constexpr Pin usart_rx_pin{GPIOPort::A, 10};

    constexpr Pin i2c_sda_pin{GPIOPort::B, 9};
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

    struct I2CCommand : public ICommand
    {
        const char *name() override { return "go"; }
        bool execute(const char *args) override
        {
            io::printSync("gooo!\n");

            // Enable clock for GPIOB
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

            // Configure PB8 and PB9 for Alternate Function mode
            // Clear the mode bits for PB8 and PB9:
            GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
            // Set alternate function mode (binary "10") for PB8 and PB9:
            GPIOB->MODER |= ((2U << (8 * 2)) | (2U << (9 * 2)));

            // Set output type to open-drain for PB8 and PB9:
            GPIOB->OTYPER |= (1U << 8) | (1U << 9);

            // Set the output speed to high speed (binary "11") for PB8 and PB9:
            GPIOB->OSPEEDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
            GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)));

            // Enable internal pull-up resistors for PB8 and PB9 (binary "01"):
            GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
            GPIOB->PUPDR |= ((1U << (8 * 2)) | (1U << (9 * 2)));

            // Configure the alternate function for PB8 and PB9 to AF4 (I2C1).
            // PB8 and PB9 are pins 8 and 9, so use AFR[1] (for pins 8..15):
            GPIOB->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
            GPIOB->AFR[1] |= ((4U << ((8 - 8) * 4)) | (4U << ((9 - 8) * 4)));


            // Disable I2C1 (clear PE) to allow configuration of timing registers
            I2C1->CR1 &= ~I2C_CR1_PE;

            // --- Set CR2: Lower 6 bits must be the APB1 clock frequency in MHz
            I2C1->CR2 = 42; // APB1 = 42 MHz

            // --- Compute and set the CCR value for standard mode:
            // Formula: CCR = F_PCLK / (2 * f_I2C)
            // With F_PCLK = 42 MHz and f_I2C = 100 kHz:
            //    CCR = 42,000,000 / (2 * 100,000) = 210
            I2C1->CCR = 210;

            // --- Set the TRISE register:
            // For standard mode: TRISE = (F_PCLK / 1,000,000) + 1
            // With F_PCLK = 42 MHz: TRISE = 42 + 1 = 43
            I2C1->TRISE = 43;

            // Re-enable the I2C peripheral
            I2C1->CR1 |= I2C_CR1_PE;


            ////////////////////////
            uint8_t slaveAddr = 0x44;
            uint8_t datas[2] = {0x22, 0x20};
            uint8_t *data = datas;
            uint8_t len = 2;

            volatile uint32_t temp;

            // Wait until I2C bus is free (BUSY flag in SR2)
            while (I2C1->SR2 & I2C_SR2_BUSY)
            {}

            // Generate the START condition by setting the START bit in CR1.
            I2C1->CR1 |= I2C_CR1_START;

            // Wait until the SB (start bit) flag is set in SR1.
            while (!(I2C1->SR1 & I2C_SR1_SB))
            {}

            // Send the slave address (7-bit) shifted left by 1, with LSB=0 for write.
            I2C1->DR = (slaveAddr << 1) & ~0x01;

            // Wait for the ADDR flag to be set in SR1 (address sent and acknowledged).
            while (!(I2C1->SR1 & I2C_SR1_ADDR))
            {}
            // Clear ADDR flag by reading SR1 and SR2.
            temp = I2C1->SR1 | I2C1->SR2;
            (void)temp; // Prevent compiler warning about unused variable.

            // Transmit the data bytes.
            while (len--)
            {
                // Wait until TXE (data register empty) flag is set.
                while (!(I2C1->SR1 & I2C_SR1_TXE))
                {}
                // Write data to the DR register.
                I2C1->DR = *data++;
            }

            // Wait until BTF (Byte Transfer Finished) flag is set.
            while (!(I2C1->SR1 & I2C_SR1_BTF))
            {}









            // Generate the STOP condition.
            I2C1->CR1 |= I2C_CR1_STOP;
            ////////////////////


            //
            //
            // I2C1->CR1 &= ~I2C_CR1_PE;  // Disable I2C before configuring
            // I2C1->CR2 = 42;           // Set PCLK frequency in MHz
            // I2C1->CCR = 210;          // Set clock control register for 100 kHz
            // I2C1->TRISE = 43;         // Set maximum rise time
            // I2C1->CR1 |= I2C_CR1_PE;   // Re-enable I2C
            //
            // // Send START condition
            // I2C1->CR1 |= I2C_CR1_START;
            // // Wait until START is cleared by hardware
            // while (I2C1->SR1 & I2C_SR1_SB)
            // {}
            // // Send slave address (e.g., 0x50 for write)
            // I2C1->DR = (0x50 << 1) | 0; // Slave address + write bit

            return true;
        }
    };
    command_executor.addCommand(std::make_unique<I2CCommand>());

    gpio::setPinOutput(led_pin, false);

    irq::enableInterrupts();

    io::printSyncFmt("--- Device is ready ---\n");

    // I2C1->CR2 |= (42 << I2C_CR2_FREQ_Pos); // 42Mhz APB1 clock
    // I2C1->CCR = 210;
    // I2C1->TRISE = 43;
    // I2C1->CR1 |= I2C_CR1_PE;

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

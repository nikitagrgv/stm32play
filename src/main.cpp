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
#include "utils/CRC.h"
#include "utils/DataStream.h"
#include "utils/FixedBitset.h"

#include <memory>

FixedDataStream<1024> usart1_stream;

CommandBuffer command_buffer;

CommandExecutor command_executor;

bool setupI2C(I2C_TypeDef *i2c)
{
    i2c->CR1 = 0;
    i2c->CR1 = I2C_CR1_ACK;
    i2c->CR2 = (42 << I2C_CR2_FREQ_Pos);
    i2c->OAR1 = 0;
    i2c->OAR2 = 0;
    i2c->SR1 = 0;
    i2c->CCR = 210;
    i2c->TRISE = 43;
    i2c->FLTR = 0;
    i2c->CR1 |= I2C_CR1_PE;
    return true;
}

bool masterTransmit(I2C_TypeDef *i2c, uint8_t address, const uint8_t *buf, uint32_t num_bytes)
{
    if (num_bytes == 0)
    {
        return true;
    }

    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB))
    {}

    i2c->DR = address << 1 | 0;
    while (!(i2c->SR1 & I2C_SR1_ADDR))
    {}

    (void)i2c->SR2; // Clear ADDR

    for (uint32_t i = 0; i < num_bytes; ++i)
    {
        while (!(i2c->SR1 & I2C_SR1_TXE))
        {}
        i2c->DR = *buf++;
    }

    while (!(i2c->SR1 & I2C_SR1_BTF))
    {}

    i2c->CR1 |= I2C_CR1_STOP;

    return true;
}

bool masterReceive(I2C_TypeDef *i2c, uint8_t address, uint8_t *buf, uint32_t num_bytes)
{
    if (num_bytes == 0)
    {
        return true;
    }

    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB))
    {}

    i2c->DR = address << 1 | 1;
    while (!(i2c->SR1 & I2C_SR1_ADDR))
    {}

    if (num_bytes == 1)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;

        (void)i2c->SR2; // Clear ADDR

        i2c->CR1 |= I2C_CR1_STOP;

        while (!(i2c->SR1 & I2C_SR1_RXNE))
        {}

        *buf++ = i2c->DR;

        return true;
    }

    if (num_bytes == 2)
    {
        i2c->CR1 |= I2C_CR1_POS;
        i2c->CR1 &= ~I2C_CR1_ACK;

        (void)i2c->SR2; // Clear ADDR

        while (!(i2c->SR1 & I2C_SR1_BTF))
        {}

        i2c->CR1 |= I2C_CR1_STOP;

        *buf++ = i2c->DR;
        *buf++ = i2c->DR;

        return true;
    }

    i2c->CR1 |= I2C_CR1_ACK;

    (void)i2c->SR2; // Clear ADDR

    for (int i = 0; i < num_bytes - 3; ++i)
    {
        while (!(i2c->SR1 & I2C_SR1_RXNE))
        {}
        *buf++ = i2c->DR;
    }

    while (!(i2c->SR1 & I2C_SR1_BTF))
    {}

    i2c->CR1 &= ~I2C_CR1_ACK;

    *buf++ = i2c->DR;

    while (!(i2c->SR1 & I2C_SR1_BTF))
    {}

    i2c->CR1 |= I2C_CR1_STOP;

    *buf++ = i2c->DR;
    *buf++ = i2c->DR;

    return true;
}

bool checkSht31(I2C_TypeDef *i2c, float &temperature, float &humidity)
{
    rcc::enableClocks(rcc::I2C_1);

    constexpr Pin scl_pin{GPIOPort::B, 8};
    constexpr Pin sda_pin{GPIOPort::B, 9};

    gpio::configureAlternate(scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);
    gpio::configureAlternate(sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);

    setupI2C(i2c);

    utils::sleepMsec(1);

    constexpr uint8_t sht31_address = 0x44;
    const uint8_t transmit_data[2] = {0x2C, 0x10};
    masterTransmit(i2c, sht31_address, transmit_data, 2);

    uint8_t receive_data[6] = {0, 0, 0, 0, 0, 0};
    masterReceive(i2c, sht31_address, receive_data, 6);

    constexpr uint8_t crc8_poly = 0x31;
    constexpr uint8_t crc8_init = 0xFF;
    bool valid = true;
    valid &= utils::crc8(receive_data, 2, crc8_poly, crc8_init) == receive_data[2];
    valid &= utils::crc8(receive_data + 3, 2, crc8_poly, crc8_init) == receive_data[5];

    if (valid)
    {
        constexpr float divisor = 2 << 16 - 1;
        const uint16_t temp = (uint16_t)receive_data[0] << 8 | receive_data[1];
        const uint16_t hum = (uint16_t)receive_data[3] << 8 | receive_data[4];
        humidity = 100.0f * (float)hum / divisor;
        temperature = -45.0f + 175.0f * (float)temp / divisor;
    }

    return valid;
}

enum class RWMode
{
    Write = 0,
    Read = 1
};

enum class RSMode
{
    Command = 0,
    Data = 1
};

constexpr int FAST_DELAY_US = 100;

void triggerLcd(I2C_TypeDef *i2c, uint8_t address, uint8_t data)
{
    utils::sleepUsec(TIM2, FAST_DELAY_US);

    constexpr uint8_t enable_mask = 1 << 5;
    constexpr uint8_t enable_clear_mask = ~enable_mask;

    uint8_t transmit_data = data & enable_mask;
    masterTransmit(i2c, address, &transmit_data, 1);

    utils::sleepUsec(TIM2, FAST_DELAY_US);

    transmit_data = data & enable_clear_mask;
    masterTransmit(i2c, address, &transmit_data, 1);
}

void runLcdCommand(I2C_TypeDef *i2c, uint8_t address, uint8_t data, RWMode rw, RSMode rs, bool backlight = true)
{
    uint8_t base_mask = 0;
    base_mask |= (uint8_t)rw << 6;
    base_mask |= (uint8_t)rs << 7;
    base_mask |= (uint8_t)backlight << 4;

    const uint8_t data_high = data >> 4;
    const uint8_t data_low = data & 0x0F;

    triggerLcd(i2c, address, base_mask | data_high);
    triggerLcd(i2c, address, base_mask | data_low);
}

bool runLcd(I2C_TypeDef *i2c)
{
    rcc::enableClocks(rcc::I2C_1);

    constexpr Pin scl_pin{GPIOPort::B, 8};
    constexpr Pin sda_pin{GPIOPort::B, 9};

    gpio::configureAlternate(scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);
    gpio::configureAlternate(sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);

    setupI2C(i2c);

    utils::sleepMsec(1);

    constexpr uint8_t address = 0x27;
    const uint8_t transmit_data[2] = {0x2C, 0x10};
    masterTransmit(i2c, address, transmit_data, 2);

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

    rcc::enableClocks(rcc::GPIO_A | rcc::GPIO_B | rcc::GPIO_C | rcc::SYSCFG_OR_AFIO | rcc::USART_1 | rcc::TIM_2);

    constexpr Pin led_pin{GPIOPort::C, 13};

    constexpr Pin usart_tx_pin{GPIOPort::A, 9};
    constexpr Pin usart_rx_pin{GPIOPort::A, 10};

    constexpr Pin user_key{GPIOPort::A, 0};
    gpio::configureInput(user_key, gpio::PullMode::Up);

    gpio::configureOutput(led_pin, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High);

#ifdef STM32F103
    gpio::configureAlternateOutput(usart_tx_pin, gpio::OutputMode::PushPull, gpio::OutputSpeed::High);
    gpio::configureAlternateInput(usart_rx_pin, gpio::PullMode::Up);
#elifdef STM32F401
    gpio::configureAlternate(usart_tx_pin, 7, gpio::OutputMode::PushPull, gpio::OutputSpeed::High);
    gpio::configureAlternate(usart_rx_pin, 7, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::High, gpio::PullMode::Up);
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
                        const bool valid = checkSht31(I2C1, temperature, humidity);
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
                        runLcd(I2C1);
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

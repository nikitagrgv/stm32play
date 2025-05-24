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
#include "drivers/MQ2/MQ2.h"
#include "drivers/SHT31Driver.h"
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

#include <cstdio>
#include <cstdlib>
#include <memory>

FixedDataStream<1024> usart1_stream;

CommandBuffer command_buffer;

CommandExecutor command_executor;

uint16_t readADC1()
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC))
        ;
    return ADC1->DR;
}

float readADC1Voltage()
{
    constexpr float REF_VOLTAGE = 3.3f;
    constexpr float MAX_VALUE = 1 << 12;

    float voltage = readADC1();
    voltage = voltage * REF_VOLTAGE / MAX_VALUE;
    return voltage;
}

float readVoltageMQ2()
{
    const float divided = readADC1Voltage();
    return divided * 2.0;
}


class MQ2Custom : public MQ2
{
public:
    float getVoltage() const override { return readVoltageMQ2(); }
};


struct MQ2Command : public ICommand
{
    MQ2Command(MQ2Custom *mq2)
        : mq2_(mq2)
    {}

    const char *name() override { return "mq2"; }

    bool execute(const char *args) override
    {
        mq2_->updateRatio();
        const uint32_t value_lpg = mq2_->readLPG();
        const uint32_t value_methane = mq2_->readMethane();
        const uint32_t value_smoke = mq2_->readSmoke();
        const uint32_t value_hydrogen = mq2_->readHydrogen();

        io::printSyncFmt("MQ2: Smoke = %u; LPG = %u; Methane = %u; Hydrogen = %u\n", value_smoke, value_lpg,
            value_methane, value_hydrogen);

        return true;
    }

    MQ2Custom *mq2_{};
};

int main()
{
    glob::SYSTEM_CORE_CLOCK = calcSystemCoreClock();
    glob::APB1_PERIPH_CLOCK = calcAPB1PeriphClock();
    glob::APB1_TIMER_CLOCK = calcAPB1TimerClock();
    glob::APB2_PERIPH_CLOCK = calcAPB2PeriphClock();
    glob::APB2_TIMER_CLOCK = calcAPB2TimerClock();

    irq::disableInterrupts();

    rcc::enableClocks(rcc::GPIO_A | rcc::GPIO_B | rcc::GPIO_C | rcc::SYSCFG_OR_AFIO | rcc::TIM_2 | rcc::TIM_3
        | rcc::TIM_1 | rcc::ADC_1);


    // MQ2 Analog input
    constexpr Pin adc_pin{GPIOPort::A, 5};
    gpio::configureAnalog(adc_pin);

    ADC1->CR2 = 0;
    ADC1->SMPR2 |= (7U << (3 * 5));
    ADC1->SQR3 = 5;
    ADC1->CR2 |= ADC_CR2_ADON;

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
    I2C main_i2c = I2C::I2C_1;
    rcc::enableClocks(rcc::I2C_1);

    constexpr Pin scl_pin{GPIOPort::B, 8};
    constexpr Pin sda_pin{GPIOPort::B, 9};
    gpio::configureAlternate(scl_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);
    gpio::configureAlternate(sda_pin, 4, gpio::OutputMode::OpenDrain, gpio::OutputSpeed::Max, gpio::PullMode::Up);

    i2c::setupI2C(main_i2c);

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
    DHT11Driver dht11{dht_pin, dht_timer};

    command_executor.addCommand(std::make_unique<DHT11Command>(&dht11));

    irq::enableInterrupts();

    MQ2Custom mq2;

    command_executor.addCommand(std::make_unique<MQ2Command>(&mq2));

    // Display
    LCD1602Driver display{main_i2c, TIM3};
    const bool display_initialized = display.initialize();
    if (display_initialized)
    {
        display.print("Initialized");
        utils::sleepMsec(500);
    }
    else
    {
        io::printSyncFmt("LCD1602 Initialization failed\n");
    }

    io::printSyncFmt("--- Device is ready ---\n");

    bool user_key_state = gpio::getPinInput(user_key);
    uint32_t user_key_last_change_time = 0;

    uint32_t last_screen_update_time = 0;
    constexpr uint32_t SCREEN_UPDATE_PERIOD_MS = 500;

    uint32_t mq2_start_time = glob::total_msec;
    uint32_t mq2_end_init_time = mq2_start_time + 60 * 1000;
    bool mq2_inited_force = false;

    bool mq2_calibrated = false;

    constexpr int BUFFER_SIZE = 16 + 1;
    char buffer[BUFFER_SIZE];

    enum class ScreenMode
    {
        DHT11 = 0,
        MQ2 = 1,
    };
    ScreenMode screen_mode = ScreenMode::MQ2;

    uint32_t last_user_key_press = glob::total_msec;
    bool skip_release = false;
    bool user_kay_holding = false;

    while (true)
    {
        bool force_screen_update = false;

        const uint32_t cur_time = glob::total_msec;

        const auto mq2_initialized = [&] {
            return mq2_inited_force || cur_time >= mq2_end_init_time;
        };

        if (screen_mode == ScreenMode::MQ2 && user_kay_holding && (cur_time - last_user_key_press > 1300))
        {
            if (!mq2_initialized())
            {
                mq2_inited_force = true;
                io::printSyncFmt("Force MQ2 initialization\n");
                display.clear();
                display.goHome();
                display.print("MQ2 force init");
                utils::sleepMsec(1000);
            }
            else if (mq2_calibrated)
            {
                mq2_calibrated = false;
            }

            skip_release = true;
            user_kay_holding = false;
        }

        if (cur_time - user_key_last_change_time > 10)
        {
            user_key_last_change_time = cur_time;
            const bool new_user_key_state = !gpio::getPinInput(user_key);
            if (new_user_key_state != user_key_state)
            {
                user_key_state = new_user_key_state;
                if (user_key_state)
                {
                    last_user_key_press = cur_time;
                    user_kay_holding = true;
                }
                else
                {
                    user_kay_holding = false;
                    if (!skip_release)
                    {
                        screen_mode = (ScreenMode)(!(bool)screen_mode);
                        force_screen_update = true;
                    }
                    skip_release = false;
                }
            }
        }

        if (mq2_initialized() && !mq2_calibrated)
        {
            mq2_calibrated = true;
            mq2.calibrate();

            if (screen_mode == ScreenMode::MQ2)
            {
                display.clear();
                display.goHome();
                display.print("MQ2 calibrated");
                utils::sleepMsec(1000);
            }
        }

        if (force_screen_update || cur_time - last_screen_update_time > SCREEN_UPDATE_PERIOD_MS)
        {
            last_screen_update_time = cur_time;

            if (screen_mode == ScreenMode::MQ2)
            {
                if (mq2_initialized())
                {
                    mq2.updateRatio();
                    const uint32_t value_lpg = mq2.readLPG();
                    const uint32_t value_methane = mq2.readMethane();
                    const uint32_t value_smoke = mq2.readSmoke();
                    const uint32_t value_hydrogen = mq2.readHydrogen();

                    display.clear();
                    display.goHome();

                    snprintf(buffer, BUFFER_SIZE, "SMK %3u LPG %3u", value_smoke, value_lpg);
                    display.print(buffer);

                    display.goToSecondLine();

                    snprintf(buffer, BUFFER_SIZE, "MTH %3u HDG %3u", value_methane, value_hydrogen);
                    display.print(buffer);
                }
                else
                {
                    const int remaining = (mq2_end_init_time - cur_time) / 1000;

                    display.clear();
                    display.goHome();
                    snprintf(buffer, BUFFER_SIZE, "MQ2 wait: %ds", remaining);
                    display.print(buffer);
                }
            }
            else
            {
                float temp = 0;
                float hum = 0;
                const DHT11Driver::ErrorCode error_code = dht11.run(temp, hum);

                if (error_code == DHT11Driver::ErrorCode::Success)
                {
                    display.clear();

                    display.goHome();

                    snprintf(buffer, BUFFER_SIZE, "T=%f", temp);
                    display.print(buffer);

                    display.goToSecondLine();

                    snprintf(buffer, BUFFER_SIZE, "H=%f", hum);
                    display.print(buffer);
                }
                else if (error_code == DHT11Driver::ErrorCode::Timeout)
                {
                    display.clear();
                    display.goHome();
                    display.print("DHT11 timeout");
                }
                else if (error_code == DHT11Driver::ErrorCode::InvalidChecksum)
                {
                    display.clear();
                    display.goHome();
                    display.print("DHT11 invalid checksum");
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

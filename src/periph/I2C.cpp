#include "I2C.h"

#include "DeviceCMSIS.h"
#include "core/Globals.h"

namespace
{

I2C_TypeDef *get_i2c_register(I2C i2c)
{
    switch (i2c)
    {
    case I2C::I2C_1: return I2C1;
    case I2C::I2C_2: return I2C2;
    case I2C::I2C_3: return I2C3;
    default: MICRO_ASSERT(0); return nullptr;
    }
}

uint32_t get_max_trise_time_ns(uint32_t i2c_frequency)
{
    if (i2c_frequency <= 100'000)
    {
        return 1000;
    }
    if (i2c_frequency <= 400'000)
    {
        return 300;
    }
    return 120;
}

} // namespace

void i2c::setupI2C(I2C i2c, uint32_t speed)
{
    I2C_TypeDef *i2c_reg = get_i2c_register(i2c);

    const uint32_t bus_frequency = glob::APB1_PERIPH_CLOCK;
    const uint32_t bus_frequency_mhz = bus_frequency / 1'000'000;
    MICRO_ASSERT(bus_frequency_mhz >= 2 && bus_frequency_mhz <= 50);

    const uint32_t ccr = bus_frequency / speed / 2;
    MICRO_ASSERT(ccr >= 4 && ccr < 1 << 12);

    const uint32_t max_trise_time_ns = get_max_trise_time_ns(speed);
    const uint32_t trise = max_trise_time_ns * bus_frequency_mhz / 1'000 + 1;
    MICRO_ASSERT(trise < 1 << 6);

    i2c_reg->CR1 = 0;
    i2c_reg->CR1 = I2C_CR1_ACK;
    i2c_reg->CR2 = bus_frequency_mhz;
    i2c_reg->OAR1 = 0;
    i2c_reg->OAR2 = 0;
    i2c_reg->SR1 = 0;
    i2c_reg->CCR = ccr;
    i2c_reg->TRISE = trise;
    i2c_reg->FLTR = 0;
    i2c_reg->CR1 |= I2C_CR1_PE;
}

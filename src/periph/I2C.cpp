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

} // namespace

void i2c::setupI2C(I2C i2c, uint32_t speed)
{
    I2C_TypeDef *i2c_reg = get_i2c_register(i2c);

    const uint32_t bus_frequency = glob::APB1_PERIPH_CLOCK;
    const uint32_t bus_frequency_mhz = bus_frequency / 1'000'000;

    i2c_reg->CR1 = 0;
    i2c_reg->CR1 = I2C_CR1_ACK;
    i2c_reg->CR2 = (42 << I2C_CR2_FREQ_Pos);
    i2c_reg->OAR1 = 0;
    i2c_reg->OAR2 = 0;
    i2c_reg->SR1 = 0;
    i2c_reg->CCR = 210;
    i2c_reg->TRISE = 43;
    i2c_reg->FLTR = 0;
    i2c_reg->CR1 |= I2C_CR1_PE;
}

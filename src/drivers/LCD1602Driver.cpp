#include "LCD1602Driver.h"

LCD1602Driver::LCD1602Driver(I2C i2c, TIM_TypeDef *timer)
    : i2c_(i2c)
    , timer_(timer)
{}



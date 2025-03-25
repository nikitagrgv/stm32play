#pragma once

#include "DeviceCMSIS.h"
#include "periph/PeriphBase.h"

class LCD1602Driver
{
public:
    LCD1602Driver(I2C i2c, TIM_TypeDef *timer);

private:
    I2C i2c_;
};
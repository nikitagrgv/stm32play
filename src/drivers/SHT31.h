#pragma once

#include "periph/PeriphBase.h"

class SHT31
{
public:
    enum class ErrorCode
    {
        Success,
        Timeout,
        InvalidChecksum,
    };

    explicit SHT31(I2C i2c);
    ~SHT31();

    ErrorCode run(float &temperature, float &humidity);
};
#pragma once

#include "CommandExecutor.h"
#include "periph/PeriphBase.h"

#include "DeviceCMSIS.h"

class SHT31Command final : public ICommand
{
public:
    SHT31Command(I2C i2c);

    const char *name() override { return "dht11"; }
    bool execute(const char *args) override;

private:
    I2C i2c;
};

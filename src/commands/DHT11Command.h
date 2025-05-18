#pragma once

#include "CommandExecutor.h"
#include "DeviceCMSIS.h"
#include "drivers/DHT11Driver.h"
#include "periph/PeriphBase.h"

class DHT11Command final : public ICommand
{
public:
    DHT11Command(DHT11Driver *driver);

    const char *name() override { return "dht11"; }
    bool execute(const char *args) override;

private:
    DHT11Driver *dht11{};
};

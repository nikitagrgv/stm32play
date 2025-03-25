#pragma once

#include "CommandExecutor.h"
#include "periph/PeriphBase.h"

#include "DeviceCMSIS.h"

class SHT31Command final : public ICommand
{
public:
    SHT31Command(const Pin &pin, TIM_TypeDef *timer);

    const char *name() override { return "dht11"; }
    bool execute(const char *args) override;

private:
    Pin pin_;
    TIM_TypeDef *timer_{};
};

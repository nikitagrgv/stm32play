#pragma once

#include "CommandExecutor.h"
#include "periph/PeriphBase.h"

#include <stm32f1xx.h>

class DHT11Command final : public ICommand
{
public:
    DHT11Command(const Pin &pin, TIM_TypeDef *timer);

    const char *name() override { return "dht11"; }
    bool execute(const char *args) override;

private:
    Pin pin_;
    TIM_TypeDef *timer_{};
};

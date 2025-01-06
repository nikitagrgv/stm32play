#pragma once

#include "CommandExecutor.h"

class DHT11Command final : public ICommand
{
public:
    const char *name() override { return "dht11"; }
    bool execute(const char *args) override;
};

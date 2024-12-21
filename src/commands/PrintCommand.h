#pragma once

#include "CommandExecutor.h"

class PrintCommand final : public ICommand
{
public:
    const char *name() override { return "print"; }

    bool execute(const char *args) override;

    bool help() override;
};


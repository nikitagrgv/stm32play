#pragma once

#include <memory>
#include <vector>

class ICommand
{
public:
    virtual ~ICommand() = default;

    virtual const char *name() = 0;
    virtual bool execute(const char *args) = 0;
    virtual bool help() { return false; }
};

class CommandExecutor
{
public:
    void addCommand(std::unique_ptr<ICommand> command);

    bool execute(const char *command);

private:
    bool try_execute_help_command(const char *command);

private:
    std::vector<std::unique_ptr<ICommand>> commands_;
};

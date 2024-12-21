#include "CommandExecutor.h"

#include "Print.h"
#include "StringUtils.h"

void CommandExecutor::addCommand(std::unique_ptr<ICommand> command)
{
    commands_.push_back(std::move(command));
    commands_.shrink_to_fit();
}

bool CommandExecutor::execute(const char *command)
{
    command = str_utils::skipStartSpaces(command);

    if (try_execute_help_command(command))
    {
        return true;
    }

    for (const std::unique_ptr<ICommand> &cmd : commands_)
    {
        const char *cmd_name = cmd->name();
        const char *name_end = str_utils::skipStart(command, cmd_name);
        if (!name_end)
        {
            // Name doesn't match
            continue;
        }

        const char *args = str_utils::skipStartSpaces(name_end);
        if (*args != 0 && args == name_end)
        {
            // Name doesn't match
            // Must be at least one space between command name and args
            continue;
        }

        return cmd->execute(args);
    }

    return false;
}

bool CommandExecutor::try_execute_help_command(const char *command)
{
    const char *name_end = str_utils::skipStart(command, "help");
    if (!name_end)
    {
        return false;
    }

    const char *arg = str_utils::skipStartSpaces(name_end);
    if (*arg != 0 && arg == name_end)
    {
        return false;
    }

    if (str_utils::allAreSpaces(arg))
    {
        io::printSync("commands:\n");
        for (const std::unique_ptr<ICommand> &cmd : commands_)
        {
            io::printSyncFmt("  %s\n", cmd->name());
        }
        return true;
    }

    for (const std::unique_ptr<ICommand> &cmd : commands_)
    {
        const char *cmd_name = cmd->name();
        const char *cmd_name_end = str_utils::skipStart(arg, cmd_name);
        if (!cmd_name_end)
        {
            continue;
        }
        if (!str_utils::allAreSpaces(cmd_name_end))
        {
            continue;
        }
        return cmd->help();
    }
    return false;
}

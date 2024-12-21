#include "PrintCommand.h"

#include "HeapUsage.h"
#include "Print.h"
#include "Statistic.h"
#include "StringUtils.h"

bool PrintCommand::execute(const char *args)
{
    if (str_utils::isEmpty(args))
    {
        return false;
    }
    if (str_utils::compareTrimmed(args, "datastat"))
    {
#ifdef ENABLE_DATA_STATISTIC
        io::printSyncFmt("num read usart = %d\n", stat::getReadBytesUsart());
        io::printSyncFmt("num read stream = %d\n", stat::getReadBytesStream());
#elif
        io::printSync("Statistic is disabled\n");
#endif
        return true;
    }
    if (str_utils::compareTrimmed(args, "heapstat"))
    {
        uint32_t used = 0;
        uint32_t total = 0;
        utils::getSbrkUsage(used, total);
        io::printSyncFmt("sbrk usage = %u/%u\n", used, total);
        return true;
    }
    return false;
}

bool PrintCommand::help()
{
    io::printSync("print entries:\n");
    io::printSync("  datastat\n");
    io::printSync("  heapstat\n");
    return true;
}
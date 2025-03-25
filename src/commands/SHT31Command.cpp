#include "SHT31Command.h"

#include "Print.h"
#include "drivers/SHT31Driver.h"
#include "periph/PeriphBase.h"

SHT31Command::SHT31Command(I2C i2c)
    : i2c_(i2c)
{}

bool SHT31Command::execute(const char *args)
{
    SHT31Driver sht31{i2c_};

    float temperature = 0.0f;
    float humidity = 0.0f;
    const SHT31Driver::ErrorCode code = sht31.run(temperature, humidity);

    if (code == SHT31Driver::ErrorCode::Timeout)
    {
        io::printSyncFmt("SHT31: Timeout\n");
        return true;
    }

    if (code == SHT31Driver::ErrorCode::InvalidChecksum)
    {
        io::printSyncFmt("SHT31: Invalid checksum\n");
        return true;
    }

    if (code != SHT31Driver::ErrorCode::Success)
    {
        io::printSyncFmt("SHT31: Unexpected error\n");
        return true;
    }

    io::printSyncFmt("SHT31: T = %f, H = %f\n", temperature, humidity);

    return true;
}

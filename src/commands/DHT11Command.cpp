#include "DHT11Command.h"

#include "DeviceCMSIS.h"
#include "Print.h"
#include "drivers/DHT11Driver.h"
#include "periph/PeriphBase.h"

DHT11Command::DHT11Command(DHT11Driver *driver)
    : dht11(driver)
{}

bool DHT11Command::execute(const char *args)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    DHT11Driver::ErrorCode code = DHT11Driver::ErrorCode::Timeout;

    int attempts = 10;
    while (attempts > 0 && code != DHT11Driver::ErrorCode::Success)
    {
        code = dht11->run(temperature, humidity);
        attempts--;
    }

    if (code == DHT11Driver::ErrorCode::Timeout)
    {
        io::printSyncFmt("DHT11: Timeout\n");
        return true;
    }

    if (code == DHT11Driver::ErrorCode::InvalidChecksum)
    {
        io::printSyncFmt("DHT11: Invalid checksum\n");
        return true;
    }

    if (code != DHT11Driver::ErrorCode::Success)
    {
        io::printSyncFmt("DHT11: Unexpected error\n");
        return true;
    }

    io::printSyncFmt("DHT11: T = %f; H = %f\n", temperature, humidity);

    return true;
}

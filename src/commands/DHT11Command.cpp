#include "DHT11Command.h"

#include "Print.h"
#include "drivers/DHT11Driver.h"
#include "periph/PeriphBase.h"

#include "DeviceCMSIS.h"

DHT11Command::DHT11Command(const Pin &pin, TIM_TypeDef *timer)
    : pin_(pin)
    , timer_(timer)
{}

bool DHT11Command::execute(const char *args)
{
    DHT11Driver dht11{pin_, timer_};

    float temperature = 0.0f;
    float humidity = 0.0f;
    const DHT11Driver::ErrorCode code = dht11.run(temperature, humidity);

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

    io::printSyncFmt("T = %f, H = %f\n", temperature, humidity);

    return true;
}

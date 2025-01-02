#include "DHT11Driver.h"

DHT11Driver::DHT11Driver(Pin input_pin, Pin output_pin, TIM_TypeDef *timer)
    : input_pin_(input_pin)
    , output_pin_(output_pin)
    , timer_(timer)
{}

DHT11Driver::ErrorCode DHT11Driver::run(float &temperature, float &humidity)
{


    return ErrorCode::Success;
}

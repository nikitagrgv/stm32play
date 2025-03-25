#pragma once

#includ3

class SHT31
{
public:
    enum class ErrorCode
    {
        Success,
        Timeout,
        InvalidChecksum,
    };

    SHT31(Pin input_pin, TIM_TypeDef *timer);
    ~SHT31();

    ErrorCode run(float &temperature, float &humidity);
}
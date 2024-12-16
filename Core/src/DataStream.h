#pragma once

#include <cstdint>

class DataStream
{
    static constexpr int SIZE = 1024;

public:
    DataStream();

    void writeByte(uint8_t byte);

    bool readByte(uint8_t &byte);

    int readData(uint8_t *data, int max_len);

    int readDataThrottling(uint8_t *data, int max_len, int throttling_msec);

private:
    volatile uint8_t buffer_[SIZE]{}; // TODO: must be external
    volatile uint8_t *buffer_begin_{};
    volatile uint8_t *buffer_end_{};

    volatile uint8_t *cur_{};
    volatile uint8_t *begin_{};
};

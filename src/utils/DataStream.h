#pragma once

#include <cstdint>

class DataStream
{
public:
    DataStream(volatile uint8_t *buf_begin, volatile uint8_t *buf_end);

    void writeByte(uint8_t byte);

    bool readByte(uint8_t &byte);

    int readData(uint8_t *data, int max_len);

    int readDataThrottling(uint8_t *data, int max_len, int throttling_msec);

private:
    volatile uint8_t *buf_begin_{};
    volatile uint8_t *buf_end_{};

    volatile uint8_t *cur_{};
    volatile uint8_t *begin_{};
};

template<int BUF_SIZE>
class FixedDataStream : public DataStream
{
public:
    FixedDataStream();

private:
    volatile uint8_t buffer_[BUF_SIZE]{};
};

// Not actually safe
template<int BUF_SIZE>
FixedDataStream<BUF_SIZE>::FixedDataStream()
    : DataStream(buffer_, buffer_ + BUF_SIZE)
{}

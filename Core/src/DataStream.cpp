#include "DataStream.h"

#include "Globals.h"
#include "Utils.h"

DataStream::DataStream()
{
    buffer_begin_ = buffer_;
    buffer_end_ = buffer_begin_ + SIZE;
    begin_ = buffer_begin_;
    cur_ = buffer_begin_;
}

void DataStream::writeByte(uint8_t byte)
{
    volatile uint8_t *next_cur = cur_ + 1;
    if (next_cur == buffer_end_)
    {
        next_cur = buffer_begin_;
    }

    if (next_cur == begin_)
    {
        // overflow
        return;
    }

    *cur_ = byte;
    cur_ = next_cur;
}

bool DataStream::readByte(uint8_t &byte)
{
    if (begin_ == cur_)
    {
        // Nothing to read
        return false;
    }

    volatile uint8_t *next_begin = begin_ + 1;
    if (next_begin == buffer_end_)
    {
        next_begin = buffer_begin_;
    }

    byte = *begin_;
    begin_ = next_begin;
    return true;
}

int DataStream::readData(uint8_t *data, int max_len)
{
    int num_read = 0;
    bool read = true;
    while (read && num_read < max_len)
    {
        read = readByte(data[num_read]);
        num_read += read;
    }
    return num_read;
}

int DataStream::readDataThrottling(uint8_t *data, int max_len, int throttling_msec)
{
    int total_num_read = 0;
    uint8_t *cur_buff_pos = data;
    const uint8_t *buffer_end = data + max_len;
    while (buffer_end - cur_buff_pos > 0)
    {
        int num_read = readData(cur_buff_pos, buffer_end - cur_buff_pos);
        if (num_read == 0)
        {
            utils::sleepMsec(throttling_msec);
            num_read = readData(cur_buff_pos, buffer_end - cur_buff_pos);
            if (num_read == 0)
            {
                break;
            }
        }
        total_num_read += num_read;
        cur_buff_pos = data + total_num_read;
    }
    return total_num_read;
}
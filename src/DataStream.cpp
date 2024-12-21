#include "DataStream.h"

#include "Globals.h"
#include "Utils.h"

DataStream::DataStream(volatile uint8_t *buf_begin, volatile uint8_t *buf_end)
{
    buf_begin_ = buf_begin;
    buf_end_ = buf_end;
    begin_ = buf_begin_;
    cur_ = buf_begin_;
}

void DataStream::writeByte(uint8_t byte)
{
    volatile uint8_t *next_cur = cur_ + 1;
    if (next_cur == buf_end_)
    {
        next_cur = buf_begin_;
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
    if (next_begin == buf_end_)
    {
        next_begin = buf_begin_;
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

    {
        total_num_read = readData(cur_buff_pos, buffer_end - cur_buff_pos);
        if (total_num_read == 0)
        {
            return 0;
        }
        cur_buff_pos = data + total_num_read;
    }

    while (buffer_end - cur_buff_pos > 0)
    {
        int num_read = readData(cur_buff_pos, buffer_end - cur_buff_pos);
        if (num_read == 0)
        {
            str_utils::sleepMsec(throttling_msec);
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
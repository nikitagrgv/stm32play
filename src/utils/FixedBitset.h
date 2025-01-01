#pragma once

#include <cstdint>

template<int N>
class FixedBitset
{
private:
    using Chunk = uint32_t;
    enum
    {
        CHUNK_SIZE_BITS = sizeof(Chunk) * 8,
        NUM_CHUNKS = (N + CHUNK_SIZE_BITS - 1) / CHUNK_SIZE_BITS,
        NUM_BYTES = NUM_CHUNKS * CHUNK_SIZE_BITS,
    };

public:
    FixedBitset() { clearAll(); }

    bool get(int n) const
    {
        const int idx = n / CHUNK_SIZE_BITS;
        const int bit = n % CHUNK_SIZE_BITS;
        return (data_[idx] >> bit) & 1;
    }

    void set(int n)
    {
        const int idx = n / CHUNK_SIZE_BITS;
        const int bit = n % CHUNK_SIZE_BITS;
        data_[idx] |= 1 << bit;
    }

    void reset(int n)
    {
        const int idx = n / CHUNK_SIZE_BITS;
        const int bit = n % CHUNK_SIZE_BITS;
        data_[idx] &= ~((Chunk)1 << bit);
    }

    void set(int n, bool value)
    {
        if (value)
        {
            set(n);
        }
        else
        {
            reset(n);
        }
    }

    void clearAll()
    {
        for (Chunk &item : data_)
        {
            item = 0;
        }
    }

    template<typename T>
    T getData(int n)
    {
        T *values = (T *)data_;
        return values[n];
    }

    template<typename T>
    void copyData(T *buf, int num_values)
    {
        T *values = (T *)data_;
        for (int i = 0; i < num_values; ++i)
        {
            buf[i] = values[i];
        }
    }

private:
    Chunk data_[NUM_CHUNKS]{};
};

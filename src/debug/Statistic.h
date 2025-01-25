#pragma once

#include "core/Base.h"

namespace stat
{

#ifdef ENABLE_DATA_STATISTIC
extern volatile int NUM_READ_USART;
extern volatile int NUM_READ_STREAM;
#endif

FORCE_INLINE void addReadBytesUsart(int num)
{
#ifdef ENABLE_DATA_STATISTIC
    NUM_READ_USART += num;
#endif
}

FORCE_INLINE int getReadBytesUsart()
{
#ifdef ENABLE_DATA_STATISTIC
    return NUM_READ_USART;
#elif
    return 0;
#endif
}

FORCE_INLINE void addReadBytesStream(int num)
{
#ifdef ENABLE_DATA_STATISTIC
    NUM_READ_STREAM += num;
#endif
}

FORCE_INLINE int getReadBytesStream()
{
#ifdef ENABLE_DATA_STATISTIC
    return NUM_READ_STREAM;
#elif
    return 0;
#endif
}

} // namespace stat

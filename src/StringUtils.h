#pragma once

#include "core/Base.h"

namespace str_utils
{

FORCE_INLINE bool isEmpty(const char *str)
{
    return !str || *str == 0;
}

FORCE_INLINE const char *skipStart(const char *str, const char *start)
{
    while (*start)
    {
        if (*str != *start) // *str == 0 also handled by this
        {
            return nullptr;
        }
        ++str;
        ++start;
    }
    return str;
}

FORCE_INLINE const char *skipStartChars(const char *str, char ch)
{
    while (*str == ch)
    {
        ++str;
    }
    return str;
}

FORCE_INLINE const char *skipStartSpaces(const char *str)
{
    return skipStartChars(str, ' ');
}

FORCE_INLINE bool allOf(const char *str, char ch)
{
    while (*str)
    {
        if (*str != ch)
        {
            return false;
        }
        ++str;
    }
    return true;
}

FORCE_INLINE bool allAreSpaces(const char *str)
{
    return allOf(str, ' ');
}


// compareTrimmed(" abc  ", "abc") -> true
// compareTrimmed(" abcd ", "abc") -> false
// compareTrimmed(" abc d", "abc") -> false
inline bool compareTrimmed(const char *str, const char *base)
{
    str = skipStartSpaces(str);

    str = skipStart(str, base);
    if (!str)
    {
        return false;
    }

    return allAreSpaces(str);
}

} // namespace str_utils
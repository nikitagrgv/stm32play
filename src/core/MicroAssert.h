#pragma once

#include <cmsis_compiler.h>

#ifdef NDEBUG
    #define MICRO_ASSERT(__e) ((void)0)
#else
    #define MICRO_ASSERT(__e) ((__e) ? (void)0 : micro_assert_failed(__FILE__, __LINE__, __PRETTY_FUNCTION__, #__e))
#endif

void micro_assert_failed(const char *file, int line, const char *func, const char *expr) __NO_RETURN;

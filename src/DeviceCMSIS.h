#pragma once

#ifdef STM32F103
    #include <stm32f1xx.h>
#elifdef STM32F401
    #include <stm32f4xx.h>
#else
    #error "Unknown device"
#endif

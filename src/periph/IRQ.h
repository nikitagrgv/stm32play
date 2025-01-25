#pragma once

#include "PeriphBase.h"

namespace irq
{

using HandlerFunc = void (*)(void *opaque);

void setHandler(InterruptType type, HandlerFunc func, void *opaque = nullptr);
void clearHandler(InterruptType type);

void setInterruptEnabled(InterruptType type, bool enabled);
void disableInterrupt(InterruptType type);
void enableInterrupt(InterruptType type);

void setInterruptsEnabled(bool enabled);
void disableInterrupts();
void enableInterrupts();

template<auto Func, typename T>
FORCE_INLINE void setHandlerMethod(InterruptType type, T *self = nullptr)
{
    irq::setHandler(
        type,
        [](void *opaque) {
            auto *s = static_cast<T *>(opaque);
            (s->*Func)();
        },
        self);
}

} // namespace irq

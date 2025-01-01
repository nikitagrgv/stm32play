#include "Handlers.h"

#include "core/Base.h"
#include "core/Globals.h"

namespace
{

struct FuncWithOpaque
{
    hnd::HandlerFunc func{};
    void *opaque{};
};

FuncWithOpaque handlers[(int)hnd::HandlerType::NUM_HANDLERS];

FORCE_INLINE void call_handler(hnd::HandlerType type)
{
    const FuncWithOpaque &fwo = handlers[(int)type];
    if (!fwo.func)
    {
        return;
    }
    (*fwo.func)(fwo.opaque);
}

} // namespace

extern "C"
{
    void SysTick_Handler()
    {
        call_handler(hnd::HandlerType::SysTickHandler);
    }

    void USART1_IRQHandler()
    {
        call_handler(hnd::HandlerType::USART1Handler);
    }

    void TIM2_IRQHandler()
    {
        call_handler(hnd::HandlerType::TIM2Handler);
    }

    void EXTI0_IRQHandler()
    {
        call_handler(hnd::HandlerType::EXTI0Handler);
    }
}

void hnd::setHandler(HandlerType type, HandlerFunc func, void *opaque)
{
    FuncWithOpaque &fwo = handlers[(int)type];
    fwo.func = func;
    fwo.opaque = opaque;
}

void hnd::clearHandler(HandlerType type)
{
    setHandler(type, nullptr, nullptr);
}
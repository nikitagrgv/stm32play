#pragma once

#include <utility>

#define FORCE_INLINE __inline__


#define CONCATENATE_IMPL(A, B) A##B
#define CONCATENATE(A, B)      CONCATENATE_IMPL(A, B)

template<typename F>
struct ScopeExit
{
    ~ScopeExit() { f(); }
    F f;
};

template<typename F>
ScopeExit<F> makeScopeExit(F &&f)
{
    return {std::forward<F>(f)};
}

#define SCOPE_EXIT(lambda) auto CONCATENATE(scope_exit_, __LINE__) = makeScopeExit(lambda);

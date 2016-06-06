#include "timer.h"

Timer::Timer()
{
    start = std::chrono::high_resolution_clock::now();
}

#ifdef DESKTOP_BUILD
    __int64 Timer::diff()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
    }
#elif ANDROID_BUILD
    __int64_t Timer::diff()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
    }
#endif

void Timer::reset()
{
    start = std::chrono::high_resolution_clock::now();
}

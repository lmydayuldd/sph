#include "timer.h"

template <class T>
Timer::Timer()
{
    start = std::chrono::high_resolution_clock::now();
}

__int64 Timer::diff()
{
    return std::chrono::duration_cast<T>(std::chrono::high_resolution_clock::now() - start).count();
}

void Timer::reset()
{
    start = std::chrono::high_resolution_clock::now();
}

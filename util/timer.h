#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#ifdef ANDROID_BUILD
#include <cinttypes>
#endif

class Timer
{
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

public:
    Timer();

#ifdef DESKTOP_BUILD
    __int64 diff();
#elif ANDROID_BUILD
    __int64_t diff();
#endif
    void reset();
};

#endif // TIMER_H

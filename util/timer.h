#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <cinttypes>

class Timer
{
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

public:
    Timer()
    {
        start = std::chrono::high_resolution_clock::now();
    }

//    __int64_t diff()
//    {
//        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
//    }
    __int64 diff()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
    }

    void reset()
    {
        start = std::chrono::high_resolution_clock::now();
    }
};

#endif // TIMER_H

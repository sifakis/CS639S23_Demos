#pragma once

#include <chrono>
#include <cstring>
#include <iostream>

struct Timer
{
    using clock_t = std::chrono::high_resolution_clock;
    using time_point_t = std::chrono::time_point<clock_t>;

    time_point_t mStartTime;
    time_point_t mStopTime;

    void Start()
    {
        mStartTime = clock_t::now();
    }

    void Stop(const std::string& msg)
    {
        mStopTime = clock_t::now();
        std::chrono::duration<double, std::milli> elapsedTime = mStopTime - mStartTime;
        std::cout << "[" << msg << elapsedTime.count() << "ms]" << std::endl;
    }
    
};

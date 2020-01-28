#pragma once
#include <chrono>

class Timer{
    
    private:
    std::chrono::time_point<std::chrono::system_clock> start;

    public:
    void tick();
    int tockMicroSeconds();
    int tockMilliSeconds();
};
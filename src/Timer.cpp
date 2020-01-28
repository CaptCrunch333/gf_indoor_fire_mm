#include "Timer.hpp"

void Timer::tick(){
    start=std::chrono::system_clock::now();
}
int Timer::tockMicroSeconds(){
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start).count();
}
int Timer::tockMilliSeconds(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
}
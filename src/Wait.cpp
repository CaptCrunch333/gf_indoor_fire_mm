#include "Wait.hpp"

void Wait::perform(){
    std::cout<< "WAITING TIME" << std::endl;
    Timer _wait_timer;
    _wait_timer.tick();
    while (_wait_timer.tockMilliSeconds()<wait_time_ms)
    {}
    
}

void Wait::receive_msg_data(DataMessage* t_msg){}

Wait::Wait(){}
Wait::~Wait(){}

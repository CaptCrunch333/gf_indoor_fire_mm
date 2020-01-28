#pragma once
#include "FlightElement.hpp"
#include "IntegerMsg.hpp"

class SetMissionState : public FlightElement{

private:
	IntegerMsg m_output_msg;

public:
    void perform();
    void setMS(int);
    int getMS();
    void receive_msg_data(DataMessage* t_msg);
    
    SetMissionState();
    ~SetMissionState();
};
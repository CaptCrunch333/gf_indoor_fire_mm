#pragma once
#include "MsgReceiver.hpp"

class Condition :public msg_receiver {
private:
	
public:
    virtual bool isConditionMet()=0;

    virtual void receive_msg_data(DataMessage* t_msg)=0;
    
};
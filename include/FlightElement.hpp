#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "ROSUnit.hpp"

class FlightElement : public msg_emitter, public msg_receiver{

public:

    virtual void perform() = 0;
    virtual void receive_msg_data(DataMessage* t_msg) = 0;

    FlightElement();
    ~FlightElement();
};
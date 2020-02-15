#pragma once
#include "MsgEmitter.hpp"
#include "logger.hpp"
#include "ROSUnit.hpp"

class FlightElement : public msg_emitter, public msg_receiver{

public:

    virtual void perform() = 0;
    virtual void receive_msg_data(DataMessage* t_msg) = 0;
    virtual void set_perform_msg(std::string);

    FlightElement();
    ~FlightElement();

protected:

    virtual void print_info();

private:

    std::string m_completion_msg = "Flight Element Performed - This is standard MSG";

};
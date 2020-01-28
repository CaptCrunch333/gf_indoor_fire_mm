#pragma once
#include "FlightElement.hpp"

class SendMessage : public FlightElement{

private:
    DataMessage* m_output_msg;
    
public:
    void perform();
    void receive_msg_data(DataMessage*);

    SendMessage(DataMessage*);
    ~SendMessage();

};
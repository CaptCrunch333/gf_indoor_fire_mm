#pragma once
#include "FlightElement.hpp"
#include "IntegerMsg.hpp"
#include "MissionStateManager.hpp"

class ChangeInternalState : public FlightElement {

private:
    GFMMState m_new_state;
    
public:
    void perform();
    void receive_msg_data(DataMessage*);

    ChangeInternalState(GFMMState);
    ~ChangeInternalState();
};
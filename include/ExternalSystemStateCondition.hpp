#pragma once
#include "Condition.hpp"
#include "common_types.hpp"
#include "IntegerMsg.hpp"

class ExternalSystemStateCondition: public Condition {

private:
	bool _isConditionMet = false;
    int m_desired_state;
    int m_actual_state = -1;

public:

    bool isConditionMet();

    void receive_msg_data(DataMessage* t_msg);

    ExternalSystemStateCondition(int);

};

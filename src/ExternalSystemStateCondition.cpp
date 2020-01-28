#include "ExternalSystemStateCondition.hpp"

ExternalSystemStateCondition::ExternalSystemStateCondition(int t_desired_state){
    m_desired_state = t_desired_state;
}

bool ExternalSystemStateCondition::isConditionMet(){
	return (m_desired_state == m_actual_state);
}

void ExternalSystemStateCondition::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* int_msg = (IntegerMsg*)t_msg;
        m_actual_state = int_msg->data;
    }
}
    

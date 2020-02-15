#include "InternalSystemStateCondition.hpp"

InternalSystemStateCondition::InternalSystemStateCondition(GFMMState t_check_state) {
    m_check_state = t_check_state;
}

bool InternalSystemStateCondition::isConditionMet(){
	return (mainMissionStateManager.getMissionState() == m_check_state);
}

void InternalSystemStateCondition::receive_msg_data(DataMessage* t_msg){

}
    

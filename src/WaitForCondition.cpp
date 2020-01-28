#include "WaitForCondition.hpp"

WaitForCondition::WaitForCondition(Condition* t_wait_condition){
    m_wait_condition = t_wait_condition;
}

void WaitForCondition::perform(){

    while (!m_wait_condition->isConditionMet())
    {
        sleep(1);
    }
    std::cout<< "CONDITION MATCHED" << std::endl;
}
void WaitForCondition::receive_msg_data(DataMessage* t_msg){
    
}

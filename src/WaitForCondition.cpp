#include "WaitForCondition.hpp"

WaitForCondition::WaitForCondition(Condition* t_wait_condition){
    m_wait_condition = t_wait_condition;
}

void WaitForCondition::perform(){

    while (!m_wait_condition->isConditionMet())
    {
        sleep(1);
        //m_met = false;
    }
    // if(!m_met) {
    //     this->print_info();
    //     m_met = true;
    // }
    this->print_info();
}
void WaitForCondition::receive_msg_data(DataMessage* t_msg){
    
}

#include "ChangeInternalState.hpp"

ChangeInternalState::ChangeInternalState(GFMMState t_new_state) {
    m_new_state = t_new_state;
}

ChangeInternalState::~ChangeInternalState() {

}

void ChangeInternalState::perform() {
    mainMissionStateManager.updateMissionState(m_new_state);
    std::cout << "Current state: " << (int)m_new_state << std::endl;
}

void ChangeInternalState::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* int_msg = (IntegerMsg*)t_msg;

        if(int_msg->data == (int)m_new_state){ //TODO remove the if or not? just makes the update to be called once instead of 7x 
            mainMissionStateManager.updateMissionState(static_cast<GFMMState>(m_new_state));
        }
    }
}
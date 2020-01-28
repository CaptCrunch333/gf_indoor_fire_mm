#include "SetMissionState.hpp"
#include <iostream>

SetMissionState::SetMissionState() {

}

SetMissionState::~SetMissionState() {

}

void SetMissionState::setMS(int t_ms){ 
    m_output_msg.data = t_ms;
}

int SetMissionState::getMS(){
    return m_output_msg.data; 
}

void SetMissionState::perform(){
    this->emit_message((DataMessage*)&m_output_msg);
}

void SetMissionState::receive_msg_data(DataMessage* t_msg){
    
}
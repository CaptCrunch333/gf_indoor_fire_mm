#include "SendMessage.hpp"

SendMessage::SendMessage(DataMessage* t_msg) {
    m_output_msg = t_msg;
}   

SendMessage::~SendMessage() {

}

void SendMessage::perform(){
    std::cout << "SENDING MESSAGE" << std::endl;
    this->emit_message(m_output_msg);
}

void SendMessage::receive_msg_data(DataMessage*){

}

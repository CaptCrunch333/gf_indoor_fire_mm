#include "QuaternionMessage.hpp"

QuaternionMessage::QuaternionMessage() {
}

QuaternionMessage::~QuaternionMessage() {

}

msg_type QuaternionMessage::getType(){
    return _type;
}

const int QuaternionMessage::getSize()
{
    return sizeof(this);
}

Quaternion QuaternionMessage::getData(){
    return _data;
}

void QuaternionMessage::setQuaternionMessage(Quaternion t_data) {
    _type = msg_type::QUATERNION;
    _data = t_data;
}

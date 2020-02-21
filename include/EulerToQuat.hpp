#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "QuaternionMessage.hpp"
#include "Vector3DMessage.hpp"
#include <math.h>

//const int E2Q_Channel = 34;
class EulerToQuat : public msg_emitter, public msg_receiver
{
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int){/*Not Implemented*/}
};
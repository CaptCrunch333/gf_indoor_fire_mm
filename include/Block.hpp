#pragma once
#include "common_types.hpp"
#include <iostream>
#include "DataMessage.hpp"
#include "MsgReceiver.hpp"

class Block : public msg_receiver{   //TODO check msg_emitter msg_receiver

    public:
        
        virtual block_id getID() = 0;
        virtual block_type getType() = 0;
        virtual void switchIn(DataMessage*) = 0;
        virtual DataMessage* switchOut() = 0;
        virtual DataMessage* receive_msg_internal(DataMessage*) = 0; 
        virtual void receive_msg_data(DataMessage* t_msg) = 0;

        Block();
        ~Block();
};
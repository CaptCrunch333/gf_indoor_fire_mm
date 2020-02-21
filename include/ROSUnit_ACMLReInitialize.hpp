#pragma once

#include "ROSUnit.hpp"
#include "geometry_msgs/Pose.h"
//Change the msg type to match your received msg
#include "Vector3DMessage.hpp"
#include "QuaternionMessage.hpp"

class ROSUnit_ACMLReInitialize : public ROSUnit
{
    public:

        ROSUnit_ACMLReInitialize(std::string, ros::NodeHandle&);
        ~ROSUnit_ACMLReInitialize();
        //Change the receive_msg_data code to reflect your system
        void receive_msg_data(DataMessage* t_msg);

    private:

        ros::Publisher m_pub;
        geometry_msgs::Pose m_pose;
};
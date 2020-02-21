#include "ROSUnit_ACMLReInitialize.hpp"

ROSUnit_ACMLReInitialize::ROSUnit_ACMLReInitialize(std::string t_name, ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)
{
    m_pub = t_main_handler.advertise<geometry_msgs::Pose>(t_name, 1, true);
}

ROSUnit_ACMLReInitialize::~ROSUnit_ACMLReInitialize()
{

}

void ROSUnit_ACMLReInitialize::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::VECTOR3D)
    {
        Vector3DMessage* t_vec = (Vector3DMessage*) t_msg;
        m_pose.position.x = t_vec->getData().x;
        m_pose.position.y = t_vec->getData().y;
        m_pose.position.z = t_vec->getData().z;
        //m_pub.publish(t_point);
    }
    else if(t_msg->getType() == msg_type::QUATERNION) {
        QuaternionMessage* t_quat_msg = (QuaternionMessage*) t_msg;
        m_pose.orientation.x = t_quat_msg->getData().x;
        m_pose.orientation.y = t_quat_msg->getData().y;
        m_pose.orientation.z = t_quat_msg->getData().z;
        m_pose.orientation.w = t_quat_msg->getData().w;
        m_pub.publish(m_pose);
    }
}
#include "EulerToQuat.hpp"

void EulerToQuat::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::VECTOR3D)
    {
        Vector3DMessage* t_vec_msg = (Vector3DMessage*) t_msg;
        float pitch = t_vec_msg->getData().x;
        float roll = t_vec_msg->getData().y;
        float yaw = t_vec_msg->getData().z;
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;
        QuaternionMessage t_quat_msg;
        t_quat_msg.setQuaternionMessage(q);
        this->emit_message((DataMessage*) &t_quat_msg);
    } 
}
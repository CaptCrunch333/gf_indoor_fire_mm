#include "ros_logger.hpp"

RosLogger::RosLogger()
{

}

RosLogger::~RosLogger()
{
}

void RosLogger::log(const char* t_log, LoggerLevel t_log_level){
    ROS_LOG(getloggerlevel(t_log_level),ROSCONSOLE_DEFAULT_NAME,t_log);
}

void RosLogger::log(const char* t_log, float t_para_1, LoggerLevel t_log_level){
    ROS_LOG(getloggerlevel(t_log_level),ROSCONSOLE_DEFAULT_NAME,t_log, t_para_1);
}

void RosLogger::log(const char* t_log, float t_para_1, float t_para_2, LoggerLevel t_log_level){
    ROS_LOG(getloggerlevel(t_log_level),ROSCONSOLE_DEFAULT_NAME,t_log, t_para_1, t_para_2);
}

void RosLogger::log(const char* t_log, float t_para_1 , float t_para_2, float t_para_3, LoggerLevel t_log_level){
    ROS_LOG(getloggerlevel(t_log_level),ROSCONSOLE_DEFAULT_NAME,t_log, t_para_1, t_para_2, t_para_3);
}

void RosLogger::enableFileLog(LoggerLevel t_loggerlevel)
{
}

void RosLogger::disableFileLog()
{

}

ros::console::levels::Level RosLogger::getloggerlevel(LoggerLevel t_log_level){
    switch (t_log_level){
        case LoggerLevel::Info:
        {
			return ros::console::levels::Info;
        }
        case LoggerLevel::Warning:
        {
			return ros::console::levels::Warn;
        }
        case LoggerLevel::Error:
        {
			return ros::console::levels::Error;
        }

    }
}

void RosLogger::logtofile(const char* t_log, float t_para_1, LoggerLevel t_log_level)
{

}

void RosLogger::logtofile(const char* t_log, float t_para_1, float t_para_2, LoggerLevel t_log_level)
{

}

void RosLogger::logtofile(const char* t_log, float t_para_1 , float t_para_2, float t_para_3, LoggerLevel t_log_level)
{

}
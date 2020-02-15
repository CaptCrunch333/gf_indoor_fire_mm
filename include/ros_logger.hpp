#pragma once
/**
* @file ros_logger.hpp
*
* @brief Logging through ROS_CONSOLER
*
* @ingroup UAVFirmware
* (Note: this needs exactly one @defgroup somewhere)
*
* @author Mohamad Chehadeh
* Contact: mohamad.chehadeh@ku.ac.ae
*
* @version 1.0 26/11/2019
*
**/
#include "logger.hpp"
#include <ros/ros.h>

class RosLogger : public Logger
{
    private:
        ros::console::levels::Level getloggerlevel(LoggerLevel);

    public:

        RosLogger();
        ~RosLogger();
        void log(const char*,LoggerLevel);
        void log(const char*,float,LoggerLevel);
        void log(const char*,float,float,LoggerLevel);
        void log(const char*,float,float,float,LoggerLevel);

        void logtofile(const char*,float,LoggerLevel);
        void logtofile(const char*,float,float,LoggerLevel);
        void logtofile(const char*,float,float,float,LoggerLevel);

        void enableFileLog(LoggerLevel);// Example: if LoggerLevel::Warning is passed only LoggerLevel::Warning and LoggerLevel::Error will be written to file
        void disableFileLog();
};
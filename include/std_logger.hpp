#pragma once
/**
* @file std_logger.hpp
*
* @brief Logging through STL
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
#include <stdio.h>
class StdLogger : public Logger
{
    private:
    LoggerLevel file_logger_level=LoggerLevel::Info;
	void printLogLevel(LoggerLevel);
    public:
    void log(const char*,LoggerLevel);
    void log(const char*,float,LoggerLevel);
    void log(const char*,float,float,LoggerLevel);
    void log(const char*,float,float,float,LoggerLevel);
    void enableFileLog(LoggerLevel);// Example: if LoggerLevel::Warning is passed only LoggerLevel::Warning and LoggerLevel::Error will be written to file
    void disableFileLog();
};
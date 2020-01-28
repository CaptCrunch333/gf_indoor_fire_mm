#pragma once
/**
* @file ros_logger.hpp
*
* @brief Logging abstract singelton
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
enum LoggerLevel {Info,Warning,Error};

class Logger {
    static Logger* assigned_logger;
    public:
    static Logger* getAssignedLogger();

    static void assignLogger(Logger*);
    virtual void log(const char*,LoggerLevel)=0;
    virtual void log(const char*,float,LoggerLevel)=0;
    virtual void log(const char*,float,float,LoggerLevel)=0;
    virtual void log(const char*,float,float,float,LoggerLevel)=0;
    virtual void enableFileLog(LoggerLevel)=0;// Example: if LoggerLevel::Warning is passed only LoggerLevel::Warning and LoggerLevel::Error will be written to file
    virtual void disableFileLog()=0;
};
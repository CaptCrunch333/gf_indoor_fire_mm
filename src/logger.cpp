#include "logger.hpp"

Logger* Logger::assigned_logger=nullptr;

Logger* Logger::getAssignedLogger(){
    return Logger::assigned_logger;
}

void Logger::assignLogger(Logger* t_assigned_logger){
    Logger::assigned_logger =t_assigned_logger;
}
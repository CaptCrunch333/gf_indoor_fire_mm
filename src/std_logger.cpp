#include "std_logger.hpp"

using namespace std;
void StdLogger::log(const char* t_log,LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log);
    printf("\n");
}

void StdLogger::log(const char* t_log,float t_para_1,LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log, t_para_1);
    printf("\n");
}

void StdLogger::log(const char* t_log,float t_para_1,float t_para_2,LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log, t_para_1, t_para_2);
    printf("\n");
}

void StdLogger::log(const char* t_log,float t_para_1 ,float t_para_2,float t_para_3,LoggerLevel t_log_level){
    printLogLevel(t_log_level);
	printf(t_log, t_para_1, t_para_2,t_para_3);
}

void StdLogger::enableFileLog(LoggerLevel){
    //TODO: Implement
}

void StdLogger::disableFileLog(){
    //TODO: Implement
}

void StdLogger::printLogLevel(LoggerLevel t_log_level){
    switch (t_log_level){
        case LoggerLevel::Info:
        {
			printf("Info: ");
            break;
        }
        case LoggerLevel::Warning:
        {
			printf("Warn: ");
            break;
        }
        case LoggerLevel::Error:
        {
			printf("Error: ");
            break;
        }

    }
}
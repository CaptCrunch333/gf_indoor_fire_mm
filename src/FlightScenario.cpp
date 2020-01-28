#include "FlightScenario.hpp"
#include <unistd.h>
pthread_mutex_t FlightScenario::execution_control=PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  FlightScenario::condition_cond  = PTHREAD_COND_INITIALIZER;
bool FlightScenario::_start_locked=true;
FlightScenario::FlightScenario(){

}

void FlightScenario::AddFlightPipeline(FlightPipeline* t_flight_pipeline){
    pthread_t ptid; 
  
    // Creating a new thread 
    pthread_create(&ptid, NULL, &pipelineThreadStarter, (void*)t_flight_pipeline); 
}
void FlightScenario::StartScenario(){
    usleep(100000);
    FlightScenario::_start_locked=false;
}
void FlightScenario::ResetScenario(){

}
//void* FlightScenario::pipelineThreadStarter(void* arg);

void* FlightScenario::pipelineThreadStarter(void* arg){
    Logger::getAssignedLogger()->log("pthread_cond_wait",LoggerLevel::Info);
    //pthread_cond_wait( &condition_cond, &execution_control );
    while (FlightScenario::_start_locked)
    {
        /* code */
    }
    
    Logger::getAssignedLogger()->log("((FlightPipeline*)arg)->execute()",LoggerLevel::Info);
    ((FlightPipeline*)arg)->execute();
    return 0;
}
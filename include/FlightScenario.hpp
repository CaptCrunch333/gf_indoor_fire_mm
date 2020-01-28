#pragma once
#include "FlightPipeline.hpp"
#include "pthread.h"
#include "logger.hpp"
class FlightScenario {
    static pthread_mutex_t execution_control;
    static pthread_cond_t  condition_cond;
    static bool _start_locked;
    public:
    void AddFlightPipeline(FlightPipeline*);
    void StartScenario();
    void ResetScenario();
    static void* pipelineThreadStarter(void* arg);
    FlightScenario();

};
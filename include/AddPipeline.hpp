#pragma once
#include "FlightScenario.hpp"

class AddPipeline : public FlightElement{

public:
    AddPipeline(FlightPipeline*, FlightScenario*);
    void perform();
    void receive_msg_data(DataMessage* t_msg);

private:
    FlightPipeline* m_pipeline;
    FlightScenario* m_scenario;
};
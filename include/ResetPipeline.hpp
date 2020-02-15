#pragma once
#include "FlightScenario.hpp"

class ResetPipeline : public FlightElement{

public:
    ResetPipeline(FlightPipeline*);
    void perform();
    void receive_msg_data(DataMessage* t_msg);

private:
    FlightPipeline* m_pipeline;
};
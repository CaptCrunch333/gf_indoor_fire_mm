#include "AddPipeline.hpp"

AddPipeline::AddPipeline(FlightPipeline* t_pipeline, FlightScenario* t_scenario)
{
    m_pipeline = t_pipeline;
    m_scenario = t_scenario;
}

void AddPipeline::perform()
{
    m_scenario->AddFlightPipeline(m_pipeline);
    this->print_info();
}

void AddPipeline::receive_msg_data(DataMessage*)
{

}
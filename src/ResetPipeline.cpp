#include "ResetPipeline.hpp"

ResetPipeline::ResetPipeline(FlightPipeline* t_pipeline)
{
    m_pipeline = t_pipeline;
}

void ResetPipeline::perform()
{
    m_pipeline->reset_pipeline();
    this->print_info();
}

void ResetPipeline::receive_msg_data(DataMessage*)
{

}
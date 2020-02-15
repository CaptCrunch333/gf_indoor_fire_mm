#include "FlightPipeline.hpp"

FlightPipeline::FlightPipeline() {

}

FlightPipeline::~FlightPipeline() {

}

void FlightPipeline::addElement(FlightElement* t_element){
    _list_of_elements.push_back(t_element);
}

void FlightPipeline::execute(){
    //std::list<FlightElement*>::iterator it;
    // for (it = _list_of_elements.begin(); it != _list_of_elements.end(); ++it){
    //     (*it)->perform();
    // }
    // this->print_done_msg();
    for (m_it = _list_of_elements.begin(); m_it != _list_of_elements.end(); ++m_it){
        (*m_it)->perform();
    }
    this->print_done_msg();
}

void FlightPipeline::reset_pipeline() {
    m_it = _list_of_elements.begin();
}

void FlightPipeline::set_msg(std::string t_msg) {
    m_name_msg = t_msg;
}

void FlightPipeline::print_add_msg() {
    std::string t_msg_to_print = "Added ";
    t_msg_to_print.append(m_name_msg);
    Logger::getAssignedLogger()->log(t_msg_to_print.c_str(), LoggerLevel::Info);
}
void FlightPipeline::print_done_msg() {
    std::string t_msg_to_print = m_name_msg;
    t_msg_to_print.append(" done!");
    Logger::getAssignedLogger()->log(t_msg_to_print.c_str(), LoggerLevel::Info);
}
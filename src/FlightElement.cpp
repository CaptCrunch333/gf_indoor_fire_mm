#include "FlightElement.hpp"

FlightElement::FlightElement(){
    
}

FlightElement::~FlightElement() {

}

void FlightElement::set_perform_msg(std::string t_msg) {
    m_completion_msg = t_msg;
}

void FlightElement::print_info() {
    Logger::getAssignedLogger()->log(m_completion_msg.c_str(), LoggerLevel::Info);
}
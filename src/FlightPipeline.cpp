#include "FlightPipeline.hpp"

FlightPipeline::FlightPipeline() {

}

FlightPipeline::~FlightPipeline() {

}

void FlightPipeline::addElement(FlightElement* t_element){
    _list_of_elements.push_back(t_element);
}

void FlightPipeline::execute(){
    std::list<FlightElement*>::iterator it;
    for (it = _list_of_elements.begin(); it != _list_of_elements.end(); ++it){
        (*it)->perform();
    }
}
#pragma once
#include <list>
#include "FlightElement.hpp"

class FlightPipeline {

private:
    std::list<FlightElement*> _list_of_elements;

public:

    void addElement(FlightElement*);
    void execute();

    FlightPipeline();
    ~FlightPipeline();
};
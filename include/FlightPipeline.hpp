#pragma once
#include <list>
#include "logger.hpp"
#include "FlightElement.hpp"

class FlightPipeline {

private:
    std::list<FlightElement*> _list_of_elements;
    std::string m_name_msg = "Pipeline";

public:

    void addElement(FlightElement*);
    void set_msg(std::string);
    void execute();
    void print_add_msg();
    void print_done_msg();

    FlightPipeline();
    ~FlightPipeline();
};
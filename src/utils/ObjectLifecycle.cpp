#include "utils/ObjectLifecycle.hpp"
#include "utils/Logger.hpp"


x3ds::ObjectLifecycle::ObjectLifecycle(std::vector<std::string> stages) : 
    stages(stages), stage_iterator(stages.begin()) {

}


void x3ds::ObjectLifecycle::iterate() {
    this->stage_iterator++;
}

bool x3ds::ObjectLifecycle::is_complete() {
    return (this->stage_iterator == this->stages.end()); 
}


void x3ds::ObjectLifecycle::set_stage_complete(std::string stage) {
    std::vector<std::string>::iterator it = std::find(this->stages.begin(), this->stages.end(), stage);
    if(it == this->stages.end()) {
        WARNLOG("Programmer warning : Invalid lifecycle state change, state %s unavailable in object's lifecycle.", stage.c_str());
        return;
    }
    this->stage_iterator = it;   
}
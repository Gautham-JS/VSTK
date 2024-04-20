#include "utils/ObjectLifecycle.hpp"
#include "utils/Logger.hpp"

using namespace vstk;

ObjectLifecycle::ObjectLifecycle(std::vector<std::string> stages) : 
    stages(stages), stage_iterator(stages.begin()) {

}


void ObjectLifecycle::iterate() {
    this->stage_iterator++;
}

bool ObjectLifecycle::is_complete() {
    return (this->stage_iterator == this->stages.end()); 
}


void ObjectLifecycle::set_stage_complete(std::string stage) {
    std::vector<std::string>::iterator it = std::find(this->stages.begin(), this->stages.end(), stage);
    if(it == this->stages.end()) {
        WARNLOG("Programmer warning : Invalid lifecycle state change, state %s unavailable in object's lifecycle.", stage.c_str());
        return;
    }
    this->stage_iterator = it;   
}
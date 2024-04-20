#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#ifndef __OBJ_LIFECYCLE_H
#define __OBJ_LIFECYCLE_H

namespace vstk {

    class ObjectLifecycle {
        private:
            std::vector<std::string> stages;
            std::vector<std::string>::iterator stage_iterator;
        public:
            ObjectLifecycle(std::vector<std::string> stages);
            void set_stage_complete(std::string stage);
            void iterate();
            bool is_complete();

    };
}
#endif

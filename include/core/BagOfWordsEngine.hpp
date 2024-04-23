#include <stdio.h>
#include <string>


#ifndef __VSTK_BOW_ENGINE_H
#define __VSTK_BOW_ENGINE_H

namespace vstk {
    class BowEngine {
        public:
            BowEngine();
            BowEngine(std::string vocab_file_path);
    };
}

#endif
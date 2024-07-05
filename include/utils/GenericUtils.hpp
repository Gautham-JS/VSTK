#ifndef __VSTK_GENERIC_UTILS_H_
#define __VSTK_GENERIC_UTILS_H_

#include <stdio.h>
#include <string>
#include <random>
#include <functional>
#include <random>
#include <sstream>

namespace vstk {
    std::string generate_random_string(uint8_t length);

    inline std::string generate_random_string() {
        return generate_random_string(16);
    }
    
    std::string generate_uuid();
}

#endif
#ifndef __VSTK_GENERIC_UTILS_H_
#define __VSTK_GENERIC_UTILS_H_

#include <stdio.h>
#include <assert.h>
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

    template<typename... Args>
    std::string fmt_str(std::string fmt, Args... args) {
        size_t bufferSize = 1000;
        char *buffer = new char[bufferSize];
        int n = sprintf(buffer, fmt.c_str(), args...);
        assert (n >= 0 and n < (int) bufferSize - 1  && "check fmt_str output");
        std::string fmtStr (buffer);
        delete buffer;
        return fmtStr;
    }
    
    std::string generate_uuid();
}

#endif
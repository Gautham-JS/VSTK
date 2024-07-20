#ifndef __VSTK_ASYNC_UTILS_H_
#define __VSTK_ASYNC_UTILS_H_

#include <thread>
#include <string>
#include <atomic>

namespace vstk {
    void set_async(bool is_async);
    std::string get_rt_id();
    void set_rt_id(std::string rt_id);
    bool is_async();

    void set_headless(bool headless);
    bool is_headless();
}

#endif
#include "utils/AsyncUtils.hpp"
#include "utils/Logger.hpp"

using namespace vstk;

static thread_local std::atomic_bool is_async;
static thread_local std::string runtime_id;
static thread_local bool is_headless_rt;

void vstk::set_async(bool is_async) {
    is_async = true;
}

bool vstk::is_async() {
    return is_async;
}

std::string vstk::get_rt_id() {
    return runtime_id;
}

void vstk::set_rt_id(std::string rt_id) {
    runtime_id = rt_id;
}

void vstk::set_headless(bool headless) {
    is_headless_rt = headless;
}

bool vstk::is_headless() {
    return is_headless_rt;
}
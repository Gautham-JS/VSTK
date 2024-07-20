#include "utils/AsyncUtils.hpp"
#include "utils/Logger.hpp"

using namespace vstk;

static thread_local std::string runtime_id;
static std::atomic_bool async;
static std::atomic_bool is_headless_rt;

void vstk::set_async(bool is_async) {
    async.store(is_async);
}

bool vstk::is_async() {
    return async.load();
}

std::string vstk::get_rt_id() {
    return runtime_id;
}

void vstk::set_rt_id(std::string rt_id) {
    runtime_id = rt_id;
}

void vstk::set_headless(bool headless) {
    is_headless_rt.store(headless);
}

bool vstk::is_headless() {
    return is_headless_rt.load();
}
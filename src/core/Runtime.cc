#include <core/Runtime.hpp>
#include "utils/Logger.hpp"

using namespace vstk;

std::string RuntimeManager::start_runtime(VstkConfig config, RuntimePtr_t &rt) {
    INFOLOG("Starting async runtime %s", rt->get_id());
}
#include "io/Persistence.hpp"
#include "utils/Logger.hpp"

using namespace vstk;


std::shared_ptr<StereoImageContextPair> ProcMemoryPersistence::get_reference_stereo_frame(std::string rt_id) {
    ThreadSharedLock lock(static_mtx);
    StereoCtxStore store = stereo_ctx_store;
    auto it = store.find(rt_id);
    
    if(it == store.end()) {
        ERRORLOG("Runtime ID [%s] was not found in memory store, invalid state!");
        return nullptr;
    }
    return std::make_shared<vstk::StereoImageContextPair>(it->second);
}

void ProcMemoryPersistence::set_reference_stereo_frame(std::string rt_id, StereoImageContextPair stereo_pair) {
    ThreadSharedLock lock(static_mtx);
    stereo_ctx_store.erase(rt_id);
    auto it = stereo_ctx_store.insert({rt_id, stereo_pair});
}


void ProcMemoryPersistence::initialize() {
    // ThreadSharedLock lock(static_mtx);
    // stereo_ctx_store = StereoCtxStore();
}

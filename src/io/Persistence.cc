#include "io/Persistence.hpp"
#include "utils/Logger.hpp"

using namespace vstk;


std::shared_ptr<StereoImageContextPair> ProcMemoryPersistence::get_reference_stereo_frame(std::string rt_id) {
    ThreadSharedLock lock(ProcMemoryPersistence::static_class_mtx);
    StereoCtxStore store = ProcMemoryPersistence::stereo_ctx_store;
    auto it = store.find(rt_id);
    
    if(it == store.end()) {
        ERRORLOG("Runtime ID [%s] was not found in memory store, invalid state!");
        return nullptr;
    }
    return std::make_shared<vstk::StereoImageContextPair>(it->second);
}

void ProcMemoryPersistence::set_reference_stereo_frame(std::string rt_id, StereoImageContextPair stereo_pair) {
    ThreadSharedLock lock(ProcMemoryPersistence::static_class_mtx);
    ProcMemoryPersistence::stereo_ctx_store.erase(rt_id);
    auto it = ProcMemoryPersistence::stereo_ctx_store.insert({rt_id, stereo_pair});
}


void ProcMemoryPersistence::initialize() {
    ThreadSharedLock lock(ProcMemoryPersistence::static_class_mtx);
    ProcMemoryPersistence::stereo_ctx_store = StereoCtxStore();
}

#include <core/Runtime.hpp>
#include "utils/Logger.hpp"

using namespace vstk;


static std::unordered_map<std::string, RuntimePtr_t> runtimes;
static std::shared_mutex mtx;

using unique_smutex_lock_t = std::unique_lock<std::shared_mutex>;



void start_pipeline_callback(std::shared_ptr<IPipeline> pipeline) {
    pipeline->initialize();
    pipeline->set_interrupt_flag(false);
    pipeline->start();
}



std::string RuntimeManager::start_runtime(VstkConfig conf) {
    RuntimeFactory rt_factory;
    RuntimePtr_t rt_ptr = rt_factory.build_rt(conf);
    return this->start_runtime(rt_ptr);
}

std::string RuntimeManager::start_runtime(RuntimePtr_t &rt) {
    INFOLOG("Starting async runtime %s", rt->get_id());
    unique_smutex_lock_t(mtx);
    std::string rt_id = rt->start();
    runtimes.insert({rt_id, rt});
    return rt_id;
}

RuntimePtr_t RuntimeManager::get_runtime(std::string rt_id) {
    unique_smutex_lock_t(mtx);
    auto it = runtimes.find(rt_id);
    if(it == runtimes.end()) {
        ERRORLOG("No runtime found in memory with ID %s", rt_id);
        return nullptr;
    }
    return it->second;
}

bool RuntimeManager::is_running(std::string rt_id) {
    unique_smutex_lock_t(mtx);
    auto it = runtimes.find(rt_id);
    if(it == runtimes.end()) {
        WARNLOG("No runtime found in memory with ID %s", rt_id);
        return false;
    }
    return (it->second->get_runtime_state() == RT_RUNNING);
}

int RuntimeManager::stop_runtime(std::string rt_id) {
    unique_smutex_lock_t(mtx);
    auto it = runtimes.find(rt_id);
    if(it == runtimes.end()) {
        WARNLOG("No runtime found in memory with ID %s", rt_id);
        return EXIT_FAILURE;
    }
    int rc = it->second->stop();
    runtimes.erase(rt_id);
    return rc;
}




RT_STATE IRuntime::get_runtime_state() {
    std::future_status f_status = this->rt_descriptor.pipeline_proc.wait_for(std::chrono::seconds(0));
    RT_STATE state = RT_ERROR;
    switch (f_status) {
        case std::future_status::ready : 
            state = RT_COMPLETE;
            break;
        case std::future_status::timeout :
            state = RT_RUNNING;
            break;
        default:
            break;
    }
    return state;
}

std::string IRuntime::get_id() {
    if(this->rt_descriptor.rt_id.empty()) {
        this->rt_descriptor.rt_id = generate_random_string(16);
    }
    return this->rt_descriptor.rt_id;
}

std::string StereoRuntime::start() {
    std::string rt_id = this->get_id();
    INFOLOG("Starting stereo runtime with async ID %s", rt_id);
    if(this->rt_descriptor.state == RT_RUNNING) {
        std::string msg = "Attempt to start an already running runtime, ignoring start call";
        WARNLOG(msg);
        throw std::bad_function_call();
    }

    this->pipeline_ptr = std::make_shared<vstk::StereoPipeline>(conf, this->get_id());
    this->rt_descriptor.pipeline_proc = std::async(start_pipeline_callback, pipeline_ptr);
    this->rt_descriptor.state = RT_RUNNING;

    INFOLOG("Runtime %s active", rt_id);
    return rt_id;
}

int StereoRuntime::stop() {
    INFOLOG("Stopping Runtime ID %s", this->get_id());
    RT_STATE state = rt_descriptor.state;
    if(state != RT_RUNNING && state != RT_COMPLETE) {
        std::string msg = "Attempt to stop a non-running runtime, ignoring stop call";
        WARNLOG(msg);
        throw std::bad_function_call(); 
    }
    this->pipeline_ptr->set_interrupt_flag(true);
    DBGLOG("Dispatched interrupt signal to Runtime ID %s, waiting for the thread to exit...", this->get_id());
    
    vstk::Timer t = vstk::get_timer(this->get_id());
    vstk::start_timer(t);
    this->rt_descriptor.pipeline_proc.get();
    vstk::end_timer(t);
    INFOLOG("Runtime thread successfully stopped in %ld ms", t.elapsed_ms);
    return 0;
}

void StereoRuntime::describe() {
    WARNLOG("Runtime description is unimplemented");
}

RuntimePtr_t RuntimeFactory::build_rt(VstkConfig conf) {
    RuntimePtr_t rt_ptr;
    switch (conf.get_slam_type()) {
        case SLAMType::STEREO :
            rt_ptr = std::make_shared<vstk::StereoRuntime>(conf);
            break;
        case SLAMType::MONO :
            WARNLOG("Monocular SLAM Pipeline is unimplemented");
            throw std::bad_function_call();
            break;
        case SLAMType::RGBD :
            WARNLOG("RGBD SLAM Pipeline is unimplemented");
            throw std::bad_function_call();
            break;
        default:
            WARNLOG("Runtime Builder logic for this SLAM type is unimplmented");
            throw std::bad_function_call();
            exit(EXIT_FAILURE);
            break;
    }
    return rt_ptr;
}


std::vector<std::string> RuntimeManager::get_all_running_rts() {
    INFOLOG("Getting all running runtime IDs");
    unique_smutex_lock_t(mtx);
    std::vector<std::string> rts;
    rts.reserve(runtimes.size());
    auto it = runtimes.begin();
    while(it != runtimes.end()) {
        if(it->first.empty()) continue;

        rts.push_back(it->first);
        it++;
    }
    return rts;
}



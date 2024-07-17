#ifndef __VSTK_CORE_RT_H_
#define __VSTK_CORE_RT_H_

#include <thread>
#include <atomic>
#include <shared_mutex>
#include <future>

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "core/Pipeline.hpp"
#include "config/Config.hpp"
#include "utils/Logger.hpp"
#include "utils/GenericUtils.hpp"

namespace vstk {

    /**
     * Core Model of VSTK : 
     * 2 Main components : 
     *      -> Runtime(conf) : 
     *              asynchronous object that handles one pipeline's runtime, gets assigned a runtime ID on construction, 
     *              holds a thread at class level with the corresponding pipeline process running within.
     * 
     *      -> RuntimeManager() : (This may be a singleton object, TBD) 
     *              asynchronous management class for handling multiple runtimes. 
     *              implements a method "string start_async(conf)" that builds a Runtime Object for that config and starts in a new thread. returns the runtime's ID
     *              also has methods for stop and get_status with rt_id as the arg. 
     *              Runtimes are stored as a static map in the class level safeguarded by a shared mutex 
     *       
     *      Key Objects In a pipeline : 
     *          - IOInterface (Implemets the read, write of data aka sets up the transport layer) 
     *            [IOInterface.hpp]
     *              * RosIO -> reads data from ROS layer, implements an internal MQ that subscriber thread fills up.
     *                  (Async : True, Requires build with ROS libraries & protobuf to work)
     * 
     *              * DiskIO -> reads data from Disk, raw image files are read from filesystem. Just globs directory and reads files.
     *                  (Async : False, does not require any libraries)
     * 
     *              * GrpcIO (TBD) -> reads data from GRPC functions for file upload & RT management.
     *                  (Async : True, Requires build with GRPC & protobuf libraries to work)
     *          
     *          - IPersistentDataStore (Implements read write of transient runtime data, needs to be low latency)
     *            [Persistence.hpp] 
     *              * ProcMemoryPersistence -> Implements an in process async memory storage. Least latency
     *              * RedisPersistence -> Implements a client for redis as a data store, higher latency due to network calls         
     */           

    enum RT_STATE {
        RT_RUNNING,
        RT_COMPLETE,
        RT_ERROR,
        RT_UNINIT,
    };

    typedef struct RuntimeDescriptor_t {
        std::string rt_id;
        std::future<void> pipeline_proc;
        RT_STATE state = RT_UNINIT;
    } RuntimeDescriptor_t;

    

    class IRuntime {
        protected:
            vstk::VstkConfig conf;
            RuntimeDescriptor_t rt_descriptor;
            std::shared_mutex mtx;
            std::atomic_bool interrupt_flag;
            std::shared_ptr<IPipeline> pipeline_ptr;

        public:
            explicit IRuntime(VstkConfig conf) : 
                conf(conf) 
            {} 
            
            virtual void describe() = 0;
            virtual std::string start() = 0;
            virtual int stop() = 0;


            RT_STATE get_runtime_state();
            std::string get_id();


        private:
            inline IPersistentDataStore* build_persistence_layer(VstkConfig conf) {
                return new ProcMemoryPersistence(conf);
            }

            inline IOInterface* build_io_layer(VstkConfig conf) {
                return new FileIO(conf);
            }
    };

    typedef std::shared_ptr<IRuntime> RuntimePtr_t;

    class StereoRuntime final : public IRuntime {
        public:
            explicit StereoRuntime(VstkConfig conf) : IRuntime(conf) {}
            
            std::string start() override;
            int stop() override;
            void describe() override;
    };

    class RuntimeFactory {
        public:
            RuntimePtr_t build_rt(VstkConfig conf);
    };

    class RuntimeManager {
        private:
            inline static std::unordered_map<std::string, RuntimePtr_t> runtimes;
            inline static std::shared_mutex mtx;
            
        public: 
            std::string start_runtime(RuntimePtr_t &rt);
            std::string start_runtime(VstkConfig config);
            RuntimePtr_t get_runtime(std::string rt_id);
            bool is_running(std::string rt_id);
            int stop_runtime(std::string rt_id);
    };

}

#endif
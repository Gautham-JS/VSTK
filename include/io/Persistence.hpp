#ifndef __VSTK_PERSISTENCE_H_
#define __VSTK_PERSISTENCE_H_

#include <stdio.h>
#include <string>
#include <unordered_map>
#include <atomic>
#include <shared_mutex>

#include "features/FeatureExtractor.hpp"
#include "config/Config.hpp"

namespace vstk {

    typedef std::pair<vstk::ImageContextHolder, vstk::ImageContextHolder> StereoImageContextPair;
    typedef std::unordered_map<std::string, vstk::StereoImageContextPair> StereoCtxStore;
    typedef std::shared_lock<std::shared_mutex> ThreadSharedLock;
    typedef std::atomic<StereoCtxStore> AtomicStereoCtxStore;

    static StereoCtxStore stereo_ctx_store;
    static std::shared_mutex static_mtx;

    class IPersistentDataStore {
        protected:
            PersistenceConfig config;
        
        public:
            explicit IPersistentDataStore(PersistenceConfig config) : config(config) {}

            virtual void initialize() = 0;

            // returns the previous reference frame from the appropriate data store. 
            //virtual std::shared_ptr<ImageContextHolder> get_reference_frame(std::string rt_id);
            virtual std::shared_ptr<StereoImageContextPair> get_reference_stereo_frame(std::string rt_id) = 0;
            virtual void set_reference_stereo_frame(std::string rt_id, StereoImageContextPair stereo_pair) = 0;
    };

    class ProcMemoryPersistence : public IPersistentDataStore {
        public:
            explicit ProcMemoryPersistence(PersistenceConfig config) : IPersistentDataStore(config) {}
            explicit ProcMemoryPersistence(VstkConfig vstk_conf) : IPersistentDataStore(*vstk_conf.get_persistence_config().get()) {}
            
            void initialize() override;

            std::shared_ptr<StereoImageContextPair> get_reference_stereo_frame(std::string rt_id) override;
            //std::shared_ptr<ImageContextHolder> get_reference_frame(std::string rt_id) override;
            void set_reference_stereo_frame(std::string rt_id, StereoImageContextPair stereo_pair) override;
    };

    class PersistenceFactory {
        public:
            IPersistentDataStore create(PersistenceConfig config);
    };

}

#endif
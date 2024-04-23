#include <stdio.h>
#include <iostream>

#include "boost/format.hpp"
#include "boost/log/trivial.hpp"

#ifndef __VSTK_LOGGER_H
#define __VSTK_LOGGER_H

#define DBGLOG(format, ...) {vstk::Logger::get().log_debug(format, ##__VA_ARGS__);}
#define INFOLOG(format, ...) {vstk::Logger::get().log_info(format, ##__VA_ARGS__);}
#define ERRORLOG(format, ...) {vstk::Logger::get().log_error(format, ##__VA_ARGS__);}
#define WARNLOG(format, ...) {vstk::Logger::get().log_warn(format, ##__VA_ARGS__);}
namespace vstk {
    
    class Logger {
        private:
            bool is_debug = false;

        protected:
            Logger() {}

        public:
            static Logger& get() {
                static Logger logger;
                return logger;
            }

            Logger(Logger &other) = delete;
            void operator=(const Logger &) = delete;

            void enable_debug() {
                is_debug = true; 
            }

            void disable_debug() {
                is_debug = false; 
            }

            template<typename... Arguments>
            void log_info(const std::string& fmt, const Arguments&... args) {
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(info) << fmtMsg;
            }
            template<typename... Arguments>
            void log_debug(const std::string& fmt, const Arguments&... args) {
                if(!is_debug) {
                    return;
                }
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(debug) << fmtMsg;
            }
            template<typename... Arguments>
            void log_trace(const std::string& fmt, const Arguments&... args) {
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(trace) << fmtMsg;
            }
            template<typename... Arguments>
            void log_error(const std::string& fmt, const Arguments&... args) {
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(error) << fmtMsg;
            }
            template<typename... Arguments>
            void log_warn(const std::string& fmt, const Arguments&... args) {
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(warning) << fmtMsg;
            }
    };
}

#endif
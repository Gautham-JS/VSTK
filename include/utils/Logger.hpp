#include <stdio.h>
#include <iostream>

#include "boost/format.hpp"
#include "boost/log/trivial.hpp"

#ifndef __X3DS_LOGGER_H
#define __X3DS_LOGGER_H

#define DBGLOG(format, ...) {x3ds::Logger::get().log_debug(format, ##__VA_ARGS__);}
#define INFOLOG(format, ...) {x3ds::Logger::get().log_info(format, ##__VA_ARGS__);}
#define ERRORLOG(format, ...) {x3ds::Logger::get().log_error(format, ##__VA_ARGS__);}
#define WARNLOG(format, ...) {x3ds::Logger::get().log_warn(format, ##__VA_ARGS__);}
namespace x3ds {
    
    class Logger {
        protected:
            Logger() {}

        public:
            static Logger& get() {
                static Logger logger;
                return logger;
            }
            Logger(Logger &other) = delete;
            void operator=(const Logger &) = delete;

            template<typename... Arguments>
            void log_info(const std::string& fmt, const Arguments&... args) {
                auto fmtMsg = boost::str((boost::format(fmt) % ... % args));
                BOOST_LOG_TRIVIAL(info) << fmtMsg;
            }
            template<typename... Arguments>
            void log_debug(const std::string& fmt, const Arguments&... args) {
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
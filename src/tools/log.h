#ifndef CYBER_COMMON_LOG_H_
#define CYBER_COMMON_LOG_H_

#include <cstdarg>
#include <string>

#include "glog/logging.h"
#include "glog/raw_logging.h"

// #include "cyber/binary.h"

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

namespace common {

class Binary {
 public:
  static std::string GetName() { return GetNameRef(); }
  static void SetName(const std::string& name) { GetNameRef() = name; }
  static std::string& GetNameRef() {
    static std::string binary_name;
    return binary_name;
  }
};
}  // namespace common

// cyber log deps
// #include <mutex>
// namespace {
//   std::mutex bm;
//   std::string b_name;
// } // namespace

// namespace apollo {
//   namespace cyber {
//     namespace binary {
//       std::string GetName() {
//         std::lock_guard<std::mutex> lock(bm);                             
//         return b_name; 
//       }

//       void SetName(const std::string& name) {                              
//         std::lock_guard<std::mutex> lock(bm);                           
//         b_name = name;                                           
//       }
//     }
//   }
// }  // namespace apollo
// // cyber log deps

#ifndef MODULE_NAME
#define MODULE_NAME common::Binary::GetName().c_str()
#endif

#define ADEBUG_MODULE(module) \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
#define AFATAL ALOG_MODULE(MODULE_NAME, FATAL)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
  ALOG_MODULE_STREAM(log_severity)(module)
#endif

#define ALOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)
#define ALOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ALOG_MODULE(module, severity)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif

#endif  // CYBER_COMMON_LOG_H_

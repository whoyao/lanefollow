//
// Created by huyao on 18-3-6.
// Email: hooyao@sjtu.edu.cn
//

#ifndef COMMON_LOG_H_
#define COMMON_LOG_H_

#include "glog/logging.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define INFO_IF(cond) LOG_IF(INFO, cond)
#define WARN_IF(cond) LOG_IF(WARNING, cond)
#define ERROR_IF(cond) LOG_IF(ERROR, cond)

// LOG_EVERY_N
#define INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)               \
    if (ptr == nullptr) {                 \
        WARN << #ptr << " is nullptr.";  \
        return;                           \
    }

#define RETURN_VAL_IF_NULL(ptr, val)      \
    if (ptr == nullptr) {                 \
        WARN << #ptr << " is nullptr.";  \
        return val;                       \
    }

#define RETURN_IF(condition)                   \
    if (condition) {                           \
        WARN << #condition << " is not met."; \
        return;                                \
    }

#define RETURN_VAL_IF(condition, val)          \
    if (condition) {                           \
        WARN << #condition << " is not met."; \
        return val;                            \
    }

#endif  // COMMON_LOG_H_

/**
 * @file rt_runtime.hpp
 * @brief Real-time threading and scheduling utilities for Linux (POSIX + SCHED_DEADLINE).
 *
 * Provides RAII classes for memory locking, CPU affinity, and thread pools with
 * real-time scheduling policies.
 */
#pragma once
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>
#include <thread>
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <tuple>
#include <algorithm>
#include <utility>
#include <mutex>
#include <functional>

#ifdef __linux__
#include <linux/sched.h> // for SCHED_DEADLINE and (maybe) struct sched_attr
#endif

// ... keep struct sched_attr fallback as before ...

namespace rt {

/**
 * @brief Throw std::runtime_error if condition is true.
 * @param cond Condition to test.
 * @param msg Error message.
 */
inline void throwIf(bool cond, const char* msg);

/**
 * @brief Wraps errno into a std::runtime_error with API name.
 * @param api API function name.
 * @param err Optional errno (defaults to global errno).
 */
inline std::runtime_error sysError(const char* api, int err = errno);

/**
 * @brief Locks all current and future process memory into RAM.
 *
 * Uses RLIMIT_MEMLOCK and mlockall(). RAII style: restored on destruction.
 */
class RtMemoryLock { /* ... */ };

/**
 * @brief A wrapper around cpu_set_t for managing CPU affinity sets.
 */
class CpuSet { /* ... */ };

/**
 * @brief Read list of online CPUs from /sys/devices/system/cpu/online.
 * @return Vector of CPU indices.
 */
inline std::vector<int> onlineCpus();

/**
 * @brief Get the set of CPUs the current process is allowed to run on.
 * @return Vector of CPU indices in affinity mask.
 */
inline std::vector<int> currentAffinityCpus();

/**
 * @brief Intersection of online CPUs and current affinity mask.
 * @return Vector of CPU indices safe to use.
 */
inline std::vector<int> allowedOnlineCpus();

/**
 * @brief Supported real-time scheduling policies.
 */
enum class Policy { FIFO, RR, DEADLINE };

/**
 * @brief Parameters for SCHED_DEADLINE scheduling.
 */
struct DeadlineParams {
    uint64_t runtime_ns;   /**< Guaranteed runtime per period in nanoseconds. */
    uint64_t deadline_ns;  /**< Absolute deadline per period in nanoseconds. */
    uint64_t period_ns;    /**< Period in nanoseconds. */
    uint64_t flags;        /**< Additional SCHED_FLAG_* bits. */
    void validate() const; /**< Validate runtime <= deadline <= period. */
};

/**
 * @brief Real-time thread with CPU affinity and scheduling policy.
 *
 * Template parameter selects the scheduling policy.
 * Supports exception propagation from thread function.
 */
template <Policy P = Policy::FIFO>
class RtThread { /* ... */ };

/**
 * @brief Convenience pool of RtThreads pinned to a set of CPUs.
 *
 * Creates one RtThread per CPU. Supports FIFO, RR, and DEADLINE (with params).
 */
template <Policy P = Policy::FIFO, typename F, typename... Args>
class RtThreadPool { /* ... */ };

} // namespace rt

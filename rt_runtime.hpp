/**
 * @file rt_runtime.hpp
 * @brief Real-time threading and scheduling utilities for Linux (POSIX + optional SCHED_DEADLINE).
 *
 * Provides RAII classes for memory locking, CPU affinity utilities, and thread/pool
 * wrappers that apply FIFO/RR/DEADLINE policies and pin to specific CPUs.
 */
#pragma once

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

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
#include <iterator>
#include <cstdint>

#ifdef __linux__
#include <linux/sched.h> // for SCHED_DEADLINE and (maybe) struct sched_attr
#endif

namespace rt {

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Throw std::runtime_error if condition is true.
 * @param cond Condition to test.
 * @param msg Error message.
 */
inline void throwIf(bool cond, const char* msg) {
    if (cond) throw std::runtime_error(msg);
}

/**
 * @brief Wrap errno into an exception with API name.
 * @param api API function name (for context).
 * @param err Optional errno value (defaults to current errno).
 */
inline std::runtime_error sysError(const char* api, int err = errno) {
    return std::runtime_error(std::string(api) + ": " + std::strerror(err));
}

/* -------------------------------------------------------------------------- */
/* 1.  Process-wide memory lock                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Locks all current and future process pages into RAM (RAII).
 *
 * Raises RLIMIT_MEMLOCK to unlimited for the process lifetime of this object
 * and calls mlockall(MCL_CURRENT|MCL_FUTURE). Both are restored/undone in the
 * destructor (best effort).
 */
class RtMemoryLock {
public:
    RtMemoryLock();
    ~RtMemoryLock();

    RtMemoryLock(const RtMemoryLock&) = delete;
    RtMemoryLock& operator=(const RtMemoryLock&) = delete;
    RtMemoryLock(RtMemoryLock&&) = default;
    RtMemoryLock& operator=(RtMemoryLock&&) = default;
private:
    struct rlimit old_{}; ///< saved memlock rlimit
};

/* -------------------------------------------------------------------------- */
/* 2.  CPU topology / affinity helpers                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief A thin RAII wrapper around cpu_set_t.
 */
class CpuSet {
public:
    CpuSet();
    explicit CpuSet(const std::vector<int>& cpus);

    /** Add a single CPU to the set. */
    void add(int cpu);
    /** Add multiple CPUs to the set. */
    void add(const std::vector<int>& cpus);

    /** Access native cpu_set_t. */
    cpu_set_t* native() { return &set_; }
    /** Access native cpu_set_t (const). */
    const cpu_set_t* native() const { return &set_; }
private:
    cpu_set_t set_{};
};

/**
 * @brief Read list of online CPUs from /sys/devices/system/cpu/online.
 * @return Vector of CPU indices.
 */
std::vector<int> onlineCpus();

/**
 * @brief Get current process CPU affinity as a list of CPUs.
 * @return Vector of CPU indices in the process' mask.
 */
std::vector<int> currentAffinityCpus();

/**
 * @brief Intersection of online CPUs and current affinity mask.
 * @return Vector of CPU indices safe to use.
 */
std::vector<int> allowedOnlineCpus();

/* -------------------------------------------------------------------------- */
/* 3.  Real-time thread (move-only, exception-aware)                          */
/* -------------------------------------------------------------------------- */

/** Supported real-time scheduling policies. */
enum class Policy { FIFO, RR, DEADLINE };

/**
 * @brief Parameters for SCHED_DEADLINE scheduling.
 */
struct DeadlineParams {
    uint64_t runtime_ns  = 100ull * 1000;   /**< Guaranteed runtime per period (ns). */
    uint64_t deadline_ns = 1000ull * 1000;  /**< Absolute deadline per period (ns). */
    uint64_t period_ns   = 1000ull * 1000;  /**< Period (ns). */
    uint64_t flags       = 0;               /**< SCHED_FLAG_* bits (optional). */
    /** Validate runtime <= deadline <= period and non-zero. */
    void validate() const;
};

/**
 * @brief Real-time thread with CPU pinning and policy application.
 *
 * - Template parameter selects the policy: FIFO, RR, or DEADLINE.
 * - Exceptions from the user functor are captured and rethrown on join().
 * - DEADLINE parameters can be supplied via a dedicated constructor.
 */
template <Policy P = Policy::FIFO>
class RtThread {
    static constexpr bool kIsDeadline = (P == Policy::DEADLINE);
public:
    /**
     * @brief Construct and start a thread pinned to `cpu`.
     * @tparam F Callable type.
     * @param cpu CPU index to pin the thread to.
     * @param f Callable.
     * @param args Arguments forwarded to the callable.
     */
    template <typename F, typename... Args>
    RtThread(int cpu, F&& f, Args&&... args) : dl_params_{} {
        launch(cpu, std::forward<F>(f), std::forward<Args>(args)...);
    }

    /**
     * @brief Construct a DEADLINE thread with explicit parameters.
     */
    template <typename F, typename... Args>
    RtThread(int cpu, DeadlineParams dl, F&& f, Args&&... args)
    requires(kIsDeadline)
    : dl_params_(dl) {
        dl_params_.validate();
        launch(cpu, std::forward<F>(f), std::forward<Args>(args)...);
    }

    ~RtThread() { if (thread_.joinable()) thread_.join(); }

    RtThread(RtThread&&)            = default;
    RtThread& operator=(RtThread&&) = default;

    /** Join the thread; rethrows any exception from the callable. */
    void join() {
        if (thread_.joinable()) thread_.join();
        if (exc_) std::rethrow_exception(exc_);
    }

    /**
     * @brief Change priority dynamically (ignored for DEADLINE).
     * @param prio Desired priority; clamped to OS min/max.
     */
    void setPriority(int prio) {
        if constexpr (!kIsDeadline) {
            int pol = policy();
            sched_param sp{};
            int pmin = sched_get_priority_min(pol);
            int pmax = sched_get_priority_max(pol);
            if (pmin != -1 && pmax != -1) prio = std::clamp(prio, pmin, pmax);
            sp.sched_priority = prio;
            int rc = pthread_setschedparam(thread_.native_handle(), pol, &sp);
            if (rc != 0) throw sysError("pthread_setschedparam", rc);
        }
    }

    /** Yield the CPU cooperatively. */
    static void yield() { sched_yield(); }

private:
    std::thread thread_{};
    std::exception_ptr exc_{};
    std::mutex exc_m_{};
    DeadlineParams dl_params_{}; // used only for DEADLINE

    static constexpr int policy() {
        if constexpr (kIsDeadline) {
#ifdef SCHED_DEADLINE
            return SCHED_DEADLINE;
#else
            return SCHED_FIFO; // compile-time fallback
#endif
        }
        if constexpr (P == Policy::FIFO) return SCHED_FIFO;
        return SCHED_RR;
    }

    template <typename F, typename... Args>
    void launch(int cpu, F&& f, Args&&... args) {
        thread_ = std::thread([this, cpu,
                               func = std::forward<F>(f),
                               tup  = std::make_tuple(std::forward<Args>(args)...)]() mutable {
            try {
                applyInsideThread(cpu, std::move(func), std::move(tup));
            } catch (...) {
                std::lock_guard<std::mutex> lk(exc_m_);
                exc_ = std::current_exception();
            }
        });
    }

    template <typename F, typename Tuple>
    void applyInsideThread(int cpu, F&& f, Tuple tup) {
        pinAndSched(cpu);
        [&]<size_t... I>(std::index_sequence<I...>) {
            std::invoke(std::forward<F>(f), std::get<I>(std::move(tup))...);
        }(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
    }

    void pinAndSched(int cpu) {
        if (cpu < 0 || cpu >= CPU_SETSIZE) throw std::runtime_error("cpu index out of range for CPU_SETSIZE");
        cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cpu, &cs);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cs), &cs) != 0) throw sysError("pthread_setaffinity_np");

#ifdef __linux__
        if constexpr (kIsDeadline) {
#  if defined(SCHED_DEADLINE) && defined(__NR_sched_setattr)
            struct sched_attr attr{};
            attr.size            = sizeof(attr);
            attr.sched_policy    = SCHED_DEADLINE;
            attr.sched_flags     = dl_params_.flags;
            attr.sched_runtime   = dl_params_.runtime_ns;
            attr.sched_deadline  = dl_params_.deadline_ns;
            attr.sched_period    = dl_params_.period_ns;
            long rc = syscall(__NR_sched_setattr, 0, &attr, 0);
            if (rc != 0) throw sysError("sched_setattr");
#  else
            throw std::runtime_error("SCHED_DEADLINE not supported on this platform");
#  endif
        } else
#endif
        {
            int pol = policy();
            sched_param sp{};
            int pmax = sched_get_priority_max(pol);
            sp.sched_priority = (pmax > 0) ? pmax - 1 : 1; // conservative default
            if (pthread_setschedparam(pthread_self(), pol, &sp) != 0) throw sysError("pthread_setschedparam");
        }
    }
};

/* -------------------------------------------------------------------------- */
/* 4.  Convenience pool                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Pool of RtThreads, one per CPU.
 */
template <Policy P = Policy::FIFO, typename F, typename... Args>
class RtThreadPool {
public:
    RtThreadPool(const std::vector<int>& cpus, F f, Args... args) {
        for (int c : cpus) threads_.emplace_back(c, f, args...);
    }
    // Overload for DEADLINE params
    RtThreadPool(const std::vector<int>& cpus, DeadlineParams dl, F f, Args... args)
    requires(P == Policy::DEADLINE)
    {
        for (int c : cpus) threads_.emplace_back(c, dl, f, args...);
    }
    /** Join all threads. */
    void joinAll() { for (auto& t : threads_) t.join(); }
private:
    std::vector<RtThread<P>> threads_;
};

} // namespace rt

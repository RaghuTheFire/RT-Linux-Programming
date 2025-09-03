/**
 * @file rt_runtime.cpp
 * @brief Out-of-line implementations for non-template rt-runtime APIs.
 */

#include "rt_runtime.hpp"
#include <set>

namespace rt {

RtMemoryLock::RtMemoryLock() {
    throwIf(getrlimit(RLIMIT_MEMLOCK, &old_) == -1, std::strerror(errno));
    struct rlimit rl{RLIM_INFINITY, RLIM_INFINITY};
    if (setrlimit(RLIMIT_MEMLOCK, &rl) == -1) throw sysError("setrlimit");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        setrlimit(RLIMIT_MEMLOCK, &old_);
        throw sysError("mlockall");
    }
}

RtMemoryLock::~RtMemoryLock() {
    munlockall();
    setrlimit(RLIMIT_MEMLOCK, &old_);
}

CpuSet::CpuSet() { CPU_ZERO(&set_); }
CpuSet::CpuSet(const std::vector<int>& cpus) : CpuSet() { add(cpus); }

void CpuSet::add(int cpu) {
    if (cpu < 0 || cpu >= CPU_SETSIZE) throw std::runtime_error("cpu index out of range for CPU_SETSIZE");
    CPU_SET(cpu, &set_);
}

void CpuSet::add(const std::vector<int>& cpus) { for (int c : cpus) add(c); }

std::vector<int> onlineCpus() {
    std::ifstream f("/sys/devices/system/cpu/online");
    std::vector<int> res;
    if (!f) return res;
    std::string s; std::getline(f, s);

    size_t pos = 0;
    while (pos < s.size()) {
        size_t next = s.find(',', pos);
        size_t dash = s.find('-', pos);
        if (dash != std::string::npos && (next == std::string::npos || dash < next)) {
            int lo = std::stoi(s.substr(pos, dash - pos));
            int hi = std::stoi(s.substr(dash + 1, (next == std::string::npos) ? std::string::npos : next - dash - 1));
            for (int i = lo; i <= hi; ++i) res.push_back(i);
        } else {
            res.push_back(std::stoi(s.substr(pos, (next == std::string::npos) ? std::string::npos : next - pos)));
        }
        pos = (next == std::string::npos) ? s.size() : next + 1;
    }
    return res;
}

std::vector<int> currentAffinityCpus() {
    std::vector<int> cpus;
    cpu_set_t cs; CPU_ZERO(&cs);
    if (sched_getaffinity(0, sizeof(cs), &cs) != 0) return cpus;
    for (int i = 0; i < CPU_SETSIZE; ++i) if (CPU_ISSET(i, &cs)) cpus.push_back(i);
    return cpus;
}

std::vector<int> allowedOnlineCpus() {
    auto on = onlineCpus();
    auto aff = currentAffinityCpus();
    std::sort(on.begin(), on.end());
    std::sort(aff.begin(), aff.end());
    std::vector<int> out; out.reserve(std::min(on.size(), aff.size()));
    std::set_intersection(on.begin(), on.end(), aff.begin(), aff.end(), std::back_inserter(out));
    return out;
}

void DeadlineParams::validate() const {
    if (!(runtime_ns && deadline_ns && period_ns)) throw std::runtime_error("deadline params must be non-zero");
    if (!(runtime_ns <= deadline_ns && deadline_ns <= period_ns)) throw std::runtime_error("require runtime <= deadline <= period");
}

} // namespace rt

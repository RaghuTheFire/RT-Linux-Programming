# rt-runtime

Utilities for Linux real-time work. Clean, minimal, and practical.

## What you get

- **RtMemoryLock** — RAII wrapper for `mlockall` with limit restore.
- **CPU helpers** — `onlineCpus`, `currentAffinityCpus`, `allowedOnlineCpus`, and a `CpuSet` wrapper.
- **RtThread** — run a callable on a fixed CPU with FIFO/RR/DEADLINE. Exceptions propagate to `join()`.
- **RtThreadPool** — spin one `RtThread` per CPU from a list.

Targets: modern Linux. Non-Linux builds, but DEADLINE is disabled.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

Run the demo:

```bash
# Caps are required for realtime and memlock, otherwise you'll see EPERM
sudo setcap cap_sys_nice,cap_ipc_lock+ep build/demo

./build/demo
```

## Doxygen docs

```bash
doxygen Doxyfile
# Open docs/html/index.html
```

## Quickstart

```cpp
#include "rt/rt_runtime.hpp"
#include <iostream>

void work(const char* name, int iters) {
  for (int i = 0; i < iters; ++i) {
    std::cout << "[" << name << "] " << i << std::endl;
    rt::RtThread<>::yield();
  }
}

int main() {
  // Best-effort: may throw if your memlock limits/caps aren't set
  try { rt::RtMemoryLock _lock; } catch (...) {}

  auto cpus = rt::allowedOnlineCpus();
  if (cpus.empty()) cpus = {0};

  // FIFO thread
  rt::RtThread<rt::Policy::FIFO> t0(cpus.front(), work, "fifo", 10);

  // RR pool on up to two CPUs
  std::vector<int> rr{cpus.front()};
  if (cpus.size() > 1) rr.push_back(cpus.back());
  rt::RtThreadPool<rt::Policy::RR, decltype(&work), const char*, int> pool(rr, &work, "rr", 6);

#if defined(SCHED_DEADLINE) && defined(__NR_sched_setattr)
  // DEADLINE thread (kernel/libc support required)
  rt::DeadlineParams dl{}; // 100us runtime, 1ms deadline/period
  try {
    rt::RtThread<rt::Policy::DEADLINE> tdl(cpus.back(), dl, work, "deadline", 8);
    tdl.join();
  } catch (const std::exception& e) {
    std::cerr << "DEADLINE failed: " << e.what() << std::endl;
  }
#endif

  t0.join();
  pool.joinAll();
}
```

## API surface

- `class rt::RtMemoryLock` — constructor raises `RLIMIT_MEMLOCK`, calls `mlockall`; destructor unlocks and restores.
- `class rt::CpuSet` — add CPUs, pass `.native()` to POSIX calls.
- `std::vector<int> onlineCpus()` — parse `/sys/devices/system/cpu/online`.
- `std::vector<int> currentAffinityCpus()` — from `sched_getaffinity`.
- `std::vector<int> allowedOnlineCpus()` — intersection of online and allowed.
- `enum class Policy { FIFO, RR, DEADLINE }`.
- `struct DeadlineParams { runtime_ns, deadline_ns, period_ns, flags; }` with `validate()`.
- `template<Policy P> class RtThread`
  - constructors: `(cpu, F&&, Args&&...)` and for DEADLINE `(cpu, DeadlineParams, F&&, Args&&...)`
  - `void setPriority(int)` (ignored for DEADLINE)
  - `void join()` (rethrows)
  - `static void yield()`
- `template<Policy P, typename F, typename... Args> class RtThreadPool`
  - constructors: `(cpus, F, Args...)` and DEADLINE `(cpus, DeadlineParams, F, Args...)`
  - `void joinAll()`

## Requirements

- Linux, g++/clang++ with C++20, pthreads.
- For graphs in docs: Graphviz (`dot`).
- Caps for runtime:
  - `cap_sys_nice` for RT scheduling (FIFO/RR/DEADLINE)
  - `cap_ipc_lock` or `ulimit -l unlimited` for `mlockall`

## Troubleshooting

- `pthread_setschedparam: Operation not permitted` → missing `cap_sys_nice` or policy/priority out of range.
- `sched_setattr: Function not implemented` → kernel/libc lacks SCHED_DEADLINE syscall; build will still run FIFO/RR.
- `cpu index out of range for CPU_SETSIZE` → you passed a CPU ID beyond the mask; use `allowedOnlineCpus()`.
- Demo prints but doesn’t look “real-time” → you probably lack caps/limits, or your cgroup restricts RT.

## Layout

```
.
├── include/rt/rt_runtime.hpp
├── src/rt_runtime.cpp
├── examples/main.cpp
├── CMakeLists.txt
├── Doxyfile
└── README.md
```

## License

Choose what you want. If in doubt, MIT is a good default.

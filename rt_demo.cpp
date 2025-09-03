/**
* @file rt_demo.cpp
* @brief Example usage of rt::RtThread and rt::RtThreadPool.
*/


#include <iostream>
#include <chrono>
#include <thread>
#include "rt_runtime.hpp"

using namespace std::chrono_literals;

static void work(const char* name, int iters) 
{
  for (int i = 0; i < iters; ++i) 
  {
  int cpu = -1;
  #ifdef __linux__
  cpu = sched_getcpu();
  #endif
  std::cout << "[" << name << "] iteration " << i
  << " on cpu " << cpu << std::endl;
  std::this_thread::sleep_for(2ms);
  rt::RtThread<>::yield();
  }
}


int main() 
{
try {
rt::RtMemoryLock _lock; // best-effort; may throw if privileges missing
} catch (const std::exception& e) {
std::cerr << "RtMemoryLock failed: " << e.what() << " (continuing without lock)\n";
}


auto cpus = rt::allowedOnlineCpus();
if (cpus.empty()) cpus = {0};


std::cout << "using cpus:";
for (int c : cpus) std::cout << ' ' << c;
std::cout << std::endl;


// FIFO example
rt::RtThread<rt::Policy::FIFO> t_fifo(cpus.front(), work, "fifo", 10);


// RR pool across up to 2 CPUs (if available)
std::vector<int> rr_cpus{cpus.front()};
if (cpus.size() > 1) rr_cpus.push_back(cpus.back());
rt::RtThreadPool<rt::Policy::RR, decltype(&work), const char*, int> pool(rr_cpus, &work, "rr", 6);


#if defined(SCHED_DEADLINE) && defined(__NR_sched_setattr)
// DEADLINE example (only if supported by kernel/libc)
rt::DeadlineParams dl{}; // defaults: 100us runtime, 1ms deadline/period
try {
int dl_cpu = cpus.back();
rt::RtThread<rt::Policy::DEADLINE> t_dl(dl_cpu, dl, work, "deadline", 8);
t_dl.join();
} catch (const std::exception& e) {
std::cerr << "DEADLINE start failed: " << e.what() << " (skipping)\n";
}
#else
std::cout << "SCHED_DEADLINE not supported on this platform; skipping DL demo\n";
#endif


t_fifo.join();
pool.joinAll();


std::cout << "done" << std::endl;
return 0;
}

#ifndef __PLANNING_TIMER_H__
#define __PLANNING_TIMER_H__

#include <chrono>
#include <iostream>
#include <string>

namespace common {

class Timer {
public:
  Timer() { begin(); }

  void begin() { start_ = std::chrono::steady_clock::now(); }

  double end() {
    end_ = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::ratio<1, 1000>> use_time_ms =
        end_ - start_;
    return use_time_ms.count();
  }

  static double timeDiff(const std::chrono::steady_clock::time_point& now, const std::chrono::steady_clock::time_point& before) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - before).count();
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> start_, end_;
};

} // namespace planner
#endif
